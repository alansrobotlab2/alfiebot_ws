#!/usr/bin/env python3
"""
GStreamer-based low-latency camera node for ROS2
Uses GStreamer pipeline to capture MJPEG and publish directly to compressed topic
Much lower latency than usb_cam

Now includes integrated WebRTC streaming for ultra-low latency VR display
"""

import asyncio
import os
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage, CameraInfo
from std_msgs.msg import Header
import gi
gi.require_version('Gst', '1.0')
from gi.repository import Gst, GLib
import numpy as np
import threading
import time
import fractions
from typing import Optional

import cv2
from av import VideoFrame

# WebRTC imports
from aiohttp import web
from aiortc import RTCPeerConnection, RTCSessionDescription, VideoStreamTrack
from aiortc.rtcrtpsender import RTCRtpSender


# WebRTC imports
from aiohttp import web
from aiortc import RTCPeerConnection, RTCSessionDescription, VideoStreamTrack
from aiortc.rtcrtpsender import RTCRtpSender


# WebRTC globals
webrtc_node: Optional['GStreamerCameraNode'] = None
webrtc_pcs = set()
webrtc_frame_event: Optional[asyncio.Event] = None
webrtc_frame_loop: Optional[asyncio.AbstractEventLoop] = None


class StereoVideoTrack(VideoStreamTrack):
    """Video track for WebRTC streaming"""
    
    kind = "video"
    
    def __init__(self, node: 'GStreamerCameraNode'):
        super().__init__()
        self.node = node
        self._timestamp = 0
        
    async def recv(self):
        """Receive the next stereo frame"""
        global webrtc_frame_event
        
        # Wait for new frame signal
        if webrtc_frame_event:
            try:
                await asyncio.wait_for(webrtc_frame_event.wait(), timeout=0.02)
                webrtc_frame_event.clear()
            except asyncio.TimeoutError:
                pass
        
        pts, time_base = self._next_timestamp()
        
        # Get latest frame
        frame_rgb = self.node.get_webrtc_frame()
        
        if frame_rgb is not None:
            try:
                frame = VideoFrame.from_ndarray(frame_rgb, format="rgb24")
                frame.pts = pts
                frame.time_base = time_base
                return frame
            except Exception as e:
                print(f"WebRTC frame error: {e}")
        
        # Return black frame if no data
        black_frame = np.zeros((self.node.height, self.node.width, 3), dtype=np.uint8)
        frame = VideoFrame.from_ndarray(black_frame, format="rgb24")
        frame.pts = pts
        frame.time_base = time_base
        return frame
    
    def _next_timestamp(self):
        self._timestamp += int(90000 / self.node.framerate)
        return self._timestamp, fractions.Fraction(1, 90000)


class GStreamerCameraNode(Node):
    def __init__(self):
        super().__init__('gstreamer_camera')
        
        # Declare parameters
        self.declare_parameter('device', '/dev/video0')
        self.declare_parameter('width', 2560)
        self.declare_parameter('height', 720)
        self.declare_parameter('framerate', 15)  # Reduced from 30 to save CPU
        self.declare_parameter('flip_vertical', True)
        self.declare_parameter('camera_frame_id', 'stereo_camera_link')
        self.declare_parameter('enable_webrtc', True)
        self.declare_parameter('webrtc_port', 8084)
        self.declare_parameter('webrtc_bitrate', 3000000)  # 3 Mbps
        self.declare_parameter('use_hardware_accel', True)  # Use Jetson hardware encoder/decoder
        self.declare_parameter('jpeg_quality', 85)  # JPEG quality (0-100, lower = smaller files, faster)
        
        # Get parameters
        self.device = self.get_parameter('device').get_parameter_value().string_value
        self.width = self.get_parameter('width').get_parameter_value().integer_value
        self.height = self.get_parameter('height').get_parameter_value().integer_value
        self.framerate = self.get_parameter('framerate').get_parameter_value().integer_value
        self.flip_vertical = self.get_parameter('flip_vertical').get_parameter_value().bool_value
        self.frame_id = self.get_parameter('camera_frame_id').get_parameter_value().string_value
        self.enable_webrtc = self.get_parameter('enable_webrtc').get_parameter_value().bool_value
        self.webrtc_port = self.get_parameter('webrtc_port').get_parameter_value().integer_value
        self.webrtc_bitrate = self.get_parameter('webrtc_bitrate').get_parameter_value().integer_value
        self.use_hardware_accel = self.get_parameter('use_hardware_accel').get_parameter_value().bool_value
        self.jpeg_quality = self.get_parameter('jpeg_quality').get_parameter_value().integer_value
        
        # WebRTC frame buffer (RGB for WebRTC)
        self._webrtc_frame = None
        self._webrtc_lock = threading.Lock()
        
        # Publishers
        self.image_pub = self.create_publisher(
            CompressedImage,
            'image_raw/compressed',
            1  # Queue size 1 for minimal latency
        )
        
        self.info_pub = self.create_publisher(
            CameraInfo,
            'camera_info',
            1
        )
        
        # Frame counter
        self.frame_count = 0
        self.last_log_time = self.get_clock().now()
        
        # Initialize GStreamer
        Gst.init(None)
        
        # Check for hardware acceleration support
        self.get_logger().info(f'Hardware acceleration requested: {self.use_hardware_accel}')
        self.hardware_available = self._check_hardware_support()
        if self.use_hardware_accel and not self.hardware_available:
            self.get_logger().warning('⚠️  Hardware acceleration requested but not available, falling back to CPU')
            self.use_hardware_accel = False
        elif self.use_hardware_accel and self.hardware_available:
            self.get_logger().info('🚀 Using hardware acceleration')
        
        # Build GStreamer pipeline
        self.pipeline = self._create_pipeline()
        
        # Start pipeline in separate thread
        self.pipeline_thread = threading.Thread(target=self._run_pipeline, daemon=True)
        self.pipeline_thread.start()
        
        self.get_logger().info(f'GStreamer camera started: {self.device} @ {self.width}x{self.height} {self.framerate}fps')
        if self.flip_vertical:
            self.get_logger().info('Vertical flip enabled')
        
        # Start WebRTC server if enabled
        if self.enable_webrtc:
            global webrtc_node
            webrtc_node = self
            self.webrtc_thread = threading.Thread(
                target=self._run_webrtc_server,
                daemon=True
            )
            self.webrtc_thread.start()
            self.get_logger().info(f'🎥 WebRTC server enabled on port {self.webrtc_port}')
            self.get_logger().info(f'   Bitrate: {self.webrtc_bitrate/1000000:.1f} Mbps')
        else:
            self.get_logger().info('WebRTC server disabled')
    
    def _check_hardware_support(self):
        """Check if Nvidia hardware acceleration plugins are available"""
        required_elements = ['nvv4l2decoder', 'nvvidconv', 'nvjpegenc']
        
        for elem_name in required_elements:
            elem = Gst.ElementFactory.find(elem_name)
            if elem is None:
                self.get_logger().warning(f'Missing GStreamer element: {elem_name}')
                return False
        
        self.get_logger().info('✅ Hardware acceleration elements available')
        return True
    
    def _create_pipeline(self):
        """Create GStreamer pipeline for low-latency MJPEG capture"""
        
        if self.use_hardware_accel:
            return self._create_hardware_pipeline()
        else:
            return self._create_software_pipeline()
    
    def _create_software_pipeline(self):
        """Create CPU-based pipeline (fallback)"""
        # Pipeline elements:
        # v4l2src: Capture from camera with minimal buffering
        # image/jpeg: Force MJPEG format (no decode!)
        # appsink: Pull frames into our application
        
        pipeline_str = (
            f'v4l2src device={self.device} ! '
            f'image/jpeg,width={self.width},height={self.height},framerate={self.framerate}/1 ! '
            f'appsink name=sink emit-signals=true max-buffers=1 drop=true sync=false'
        )
        
        self.get_logger().info(f'🖥️  CPU Pipeline: {pipeline_str}')
        
        pipeline = Gst.parse_launch(pipeline_str)
        appsink = pipeline.get_by_name('sink')
        appsink.connect('new-sample', self._on_new_sample_software)
        
        return pipeline
    
    def _create_hardware_pipeline(self):
        """Create hardware-accelerated pipeline using Jetson NVDEC"""
        # Smart hardware pipeline:
        # - If NO flip needed: Pass JPEG directly (zero-copy, fastest)
        # - If flip needed: Use nvv4l2decoder + nvvidconv + nvjpegenc (all hardware!)
        # - Always: Use tee to split for WebRTC when needed
        
        if not self.flip_vertical:
            # No flip needed - pass JPEG directly for ROS, decode only for WebRTC
            # This is the FASTEST path - zero decode/encode for ROS!
            pipeline_str = (
                f'v4l2src device={self.device} ! '
                f'image/jpeg,width={self.width},height={self.height},framerate={self.framerate}/1 ! '
                f'tee name=t ! '
                # ROS branch: Direct JPEG passthrough (zero copy!)
                f'queue max-size-buffers=1 leaky=downstream ! '
                f'appsink name=ros_sink emit-signals=true max-buffers=1 drop=true sync=false '
                # WebRTC branch: Decode to RGBA only when needed
                f't. ! queue max-size-buffers=1 leaky=downstream ! '
                f'nvv4l2decoder mjpeg=1 ! '
                f'nvvidconv ! '
                f'video/x-raw,format=RGBA ! '  # RGBA output from hardware
                f'appsink name=webrtc_sink emit-signals=true max-buffers=1 drop=true sync=false'
            )
            self.get_logger().info(f'🚀 Hardware Pipeline (No-Flip, Zero-Copy JPEG): {pipeline_str}')
        else:
            # Flip needed - use FULL hardware pipeline: decode→flip→encode (all on GPU!)
            flip_method = 2  # vertical flip in nvvidconv
            
            pipeline_str = (
                f'v4l2src device={self.device} ! '
                f'image/jpeg,width={self.width},height={self.height},framerate={self.framerate}/1 ! '
                f'nvv4l2decoder mjpeg=1 ! '
                f'nvvidconv flip-method={flip_method} ! '
                f'video/x-raw(memory:NVMM),format=I420 ! '
                f'tee name=t ! '
                # ROS branch: Hardware JPEG encode
                f'queue max-size-buffers=1 leaky=downstream ! '
                f'nvjpegenc quality={self.jpeg_quality} ! '
                f'appsink name=ros_sink emit-signals=true max-buffers=1 drop=true sync=false '
                # WebRTC branch: Hardware convert to RGBA (nvvidconv supports RGBA, not RGB)
                f't. ! queue max-size-buffers=1 leaky=downstream ! '
                f'nvvidconv ! '
                f'video/x-raw,format=RGBA ! '  # RGBA output from hardware
                f'appsink name=webrtc_sink emit-signals=true max-buffers=1 drop=true sync=false'
            )
            self.get_logger().info(f'🚀 Hardware Pipeline (Full GPU: Decode→Flip→Encode): {pipeline_str}')
        
        try:
            pipeline = Gst.parse_launch(pipeline_str)
            
            # Connect ROS sink
            ros_sink = pipeline.get_by_name('ros_sink')
            ros_sink.connect('new-sample', self._on_new_sample_hardware_jpeg)
            
            # Connect WebRTC sink
            webrtc_sink = pipeline.get_by_name('webrtc_sink')
            webrtc_sink.connect('new-sample', self._on_new_sample_hardware_webrtc_i420)
            
            return pipeline
            
        except Exception as e:
            self.get_logger().error(f'Failed to create hardware pipeline: {e}')
            self.get_logger().warning('Falling back to CPU pipeline')
            self.use_hardware_accel = False
            return self._create_software_pipeline()
    
    def _on_new_sample_software(self, sink):
        """Callback for CPU pipeline - decode and process"""
        global webrtc_frame_event, webrtc_frame_loop
        
        sample = sink.emit('pull-sample')
        if sample is None:
            return Gst.FlowReturn.OK
        
        # Get buffer
        buf = sample.get_buffer()
        
        # Get current time FIRST for accurate latency measurement
        receive_time = self.get_clock().now()
        
        # Extract JPEG data
        success, map_info = buf.map(Gst.MapFlags.READ)
        if not success:
            return Gst.FlowReturn.OK
        
        try:
            # Get JPEG bytes
            jpeg_data = map_info.data
            
            # Decode once for both ROS and WebRTC
            nparr = np.frombuffer(jpeg_data, np.uint8)
            img = cv2.imdecode(nparr, cv2.IMREAD_COLOR)
            
            if img is None:
                return Gst.FlowReturn.OK
            
            # Handle vertical flip if needed
            if self.flip_vertical:
                img = cv2.flip(img, 0)  # Vertical flip
            
            # Re-encode for ROS topic (JPEG)
            _, encoded = cv2.imencode('.jpg', img, [cv2.IMWRITE_JPEG_QUALITY, self.jpeg_quality])
            jpeg_data_final = encoded.tobytes()
            
            # Convert to RGB for WebRTC (only if enabled AND clients connected)
            if self.enable_webrtc and len(webrtc_pcs) > 0:
                img_rgb = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
                with self._webrtc_lock:
                    self._webrtc_frame = img_rgb.copy()
                
                # Signal WebRTC that new frame is ready
                if webrtc_frame_event and webrtc_frame_loop:
                    try:
                        webrtc_frame_loop.call_soon_threadsafe(webrtc_frame_event.set)
                    except RuntimeError:
                        pass
            
            # Create ROS message - use receive_time (NOW) for freshest timestamp
            msg = CompressedImage()
            msg.header = Header()
            msg.header.stamp = receive_time.to_msg()
            msg.header.frame_id = self.frame_id
            msg.format = 'jpeg'
            msg.data = bytes(jpeg_data_final)
            
            # Publish to ROS
            self.image_pub.publish(msg)
            
            # Publish camera info (basic)
            info_msg = CameraInfo()
            info_msg.header = msg.header
            info_msg.width = self.width
            info_msg.height = self.height
            self.info_pub.publish(info_msg)
            
            # Stats
            self.frame_count += 1
            now = self.get_clock().now()
            elapsed = (now - self.last_log_time).nanoseconds / 1e9
            if elapsed >= 5.0:
                fps = self.frame_count / elapsed
                self.get_logger().info(f'Publishing at {fps:.1f} FPS')
                self.frame_count = 0
                self.last_log_time = now
            
        finally:
            buf.unmap(map_info)
        
        return Gst.FlowReturn.OK
    
    def _on_new_sample_hardware_jpeg(self, sink):
        """Callback for hardware pipeline JPEG passthrough (no flip) - ZERO COPY!"""
        sample = sink.emit('pull-sample')
        if sample is None:
            return Gst.FlowReturn.OK
        
        buf = sample.get_buffer()
        receive_time = self.get_clock().now()
        
        success, map_info = buf.map(Gst.MapFlags.READ)
        if not success:
            return Gst.FlowReturn.OK
        
        try:
            # Direct JPEG passthrough - no decode/encode!
            jpeg_data = bytes(map_info.data)
            
            # Create ROS message
            msg = CompressedImage()
            msg.header = Header()
            msg.header.stamp = receive_time.to_msg()
            msg.header.frame_id = self.frame_id
            msg.format = 'jpeg'
            msg.data = jpeg_data
            
            # Publish to ROS
            self.image_pub.publish(msg)
            
            # Publish camera info
            info_msg = CameraInfo()
            info_msg.header = msg.header
            info_msg.width = self.width
            info_msg.height = self.height
            self.info_pub.publish(info_msg)
            
            # Stats
            self.frame_count += 1
            now = self.get_clock().now()
            elapsed = (now - self.last_log_time).nanoseconds / 1e9
            if elapsed >= 5.0:
                fps = self.frame_count / elapsed
                mode = "HW-JPEG" if self.use_hardware_accel and self.flip_vertical else "HW-ZeroCopy"
                self.get_logger().info(f'[{mode}] Publishing at {fps:.1f} FPS')
                self.frame_count = 0
                self.last_log_time = now
        
        finally:
            buf.unmap(map_info)
        
        return Gst.FlowReturn.OK
    
    def _on_new_sample_hardware_webrtc_i420(self, sink):
        """Callback for hardware pipeline WebRTC branch - RGB directly from nvvidconv"""
        global webrtc_frame_event, webrtc_frame_loop
        
        # Only process if WebRTC is enabled and clients are connected
        if not self.enable_webrtc or len(webrtc_pcs) == 0:
            # Drop frame - don't even pull it
            sample = sink.emit('pull-sample')
            return Gst.FlowReturn.OK
        
        sample = sink.emit('pull-sample')
        if sample is None:
            return Gst.FlowReturn.OK
        
        buf = sample.get_buffer()
        caps = sample.get_caps()
        struct = caps.get_structure(0)
        width = struct.get_value('width')
        height = struct.get_value('height')
        
        success, map_info = buf.map(Gst.MapFlags.READ)
        if not success:
            return Gst.FlowReturn.OK
        
        try:
            # RGBA data from hardware - strip alpha channel for RGB
            img_rgba = np.frombuffer(map_info.data, dtype=np.uint8)
            img_rgba = img_rgba.reshape((height, width, 4))
            
            # Strip alpha channel (RGBA -> RGB)
            img_rgb = img_rgba[:, :, :3]
            
            # Store for WebRTC (need copy for thread safety)
            with self._webrtc_lock:
                self._webrtc_frame = img_rgb.copy()
            
            # Signal WebRTC
            if webrtc_frame_event and webrtc_frame_loop:
                try:
                    webrtc_frame_loop.call_soon_threadsafe(webrtc_frame_event.set)
                except RuntimeError:
                    pass
        
        finally:
            buf.unmap(map_info)
        
        return Gst.FlowReturn.OK
    
    def _run_pipeline(self):
        """Run GStreamer pipeline in separate thread"""
        # Set to playing
        self.pipeline.set_state(Gst.State.PLAYING)
        
        # Create main loop
        loop = GLib.MainLoop()
        
        # Handle bus messages
        bus = self.pipeline.get_bus()
        bus.add_signal_watch()
        bus.connect('message', self._on_bus_message, loop)
        
        # Run loop
        try:
            loop.run()
        except KeyboardInterrupt:
            pass
        
        # Cleanup
        self.pipeline.set_state(Gst.State.NULL)
    
    def _on_bus_message(self, bus, message, loop):
        """Handle GStreamer bus messages"""
        t = message.type
        if t == Gst.MessageType.ERROR:
            err, debug = message.parse_error()
            self.get_logger().error(f'GStreamer error: {err}, {debug}')
            loop.quit()
        elif t == Gst.MessageType.EOS:
            self.get_logger().info('End of stream')
            loop.quit()
        elif t == Gst.MessageType.WARNING:
            err, debug = message.parse_warning()
            self.get_logger().warn(f'GStreamer warning: {err}')
        
        return True
    
    def get_webrtc_frame(self) -> Optional[np.ndarray]:
        """Get the latest frame for WebRTC"""
        with self._webrtc_lock:
            return self._webrtc_frame
    
    def _run_webrtc_server(self):
        """Run WebRTC server in a new event loop"""
        try:
            loop = asyncio.new_event_loop()
            asyncio.set_event_loop(loop)
            loop.run_until_complete(self._webrtc_server_async())
        except Exception as e:
            self.get_logger().error(f'WebRTC server error: {e}')
            import traceback
            traceback.print_exc()
    
    async def _webrtc_server_async(self):
        """Async WebRTC signaling server"""
        global webrtc_frame_event, webrtc_frame_loop
        
        webrtc_frame_event = asyncio.Event()
        webrtc_frame_loop = asyncio.get_running_loop()
        
        app = web.Application()
        
        async def cors_middleware(app, handler):
            async def middleware_handler(request):
                if request.method == "OPTIONS":
                    response = web.Response()
                else:
                    try:
                        response = await handler(request)
                    except web.HTTPException as ex:
                        response = ex
                response.headers["Access-Control-Allow-Origin"] = "*"
                response.headers["Access-Control-Allow-Methods"] = "POST, OPTIONS"
                response.headers["Access-Control-Allow-Headers"] = "Content-Type"
                return response
            return middleware_handler
        
        app.middlewares.append(cors_middleware)
        app.router.add_post("/offer", self._webrtc_offer)
        app.on_shutdown.append(self._webrtc_shutdown)
        
        ssl_context = self._create_ssl_context()
        
        runner = web.AppRunner(app)
        await runner.setup()
        site = web.TCPSite(runner, "0.0.0.0", self.webrtc_port, ssl_context=ssl_context)
        await site.start()
        
        protocol = "https" if ssl_context else "http"
        print(f"WebRTC signaling: {protocol}://0.0.0.0:{self.webrtc_port}")
        
        while True:
            await asyncio.sleep(3600)
    
    async def _webrtc_offer(self, request):
        """Handle WebRTC offer from client"""
        global webrtc_pcs
        
        params = await request.json()
        offer = RTCSessionDescription(sdp=params["sdp"], type=params["type"])
        
        pc = RTCPeerConnection()
        webrtc_pcs.add(pc)
        
        @pc.on("connectionstatechange")
        async def on_connectionstatechange():
            print(f"WebRTC connection state: {pc.connectionState}")
            if pc.connectionState == "failed" or pc.connectionState == "closed":
                await pc.close()
                webrtc_pcs.discard(pc)
        
        # Add video track
        video_track = StereoVideoTrack(self)
        pc.addTrack(video_track)
        
        # Select VP8 codec for best performance
        try:
            capabilities = RTCRtpSender.getCapabilities("video")
            if capabilities:
                preferred_codecs = [c for c in capabilities.codecs if c.mimeType == "video/VP8"]
                if preferred_codecs:
                    transceiver = pc.getTransceivers()[0]
                    transceiver.setCodecPreferences(preferred_codecs)
                    print("Using VP8 codec")
        except Exception as e:
            print(f"Could not set codec preferences: {e}")
        
        await pc.setRemoteDescription(offer)
        answer = await pc.createAnswer()
        await pc.setLocalDescription(answer)
        
        # Optimize SDP for low latency
        sdp = pc.localDescription.sdp
        bitrate_kbps = self.webrtc_bitrate // 1000
        sdp_lines = sdp.split('\n')
        new_lines = []
        for line in sdp_lines:
            new_lines.append(line)
            if line.startswith('m=video'):
                new_lines.append(f'b=AS:{bitrate_kbps}')
                new_lines.append(f'b=TIAS:{self.webrtc_bitrate}')
            if line.startswith('a=rtpmap') and 'VP8' in line:
                payload_type = line.split()[0].split(':')[1]
                new_lines.append(
                    f'a=fmtp:{payload_type} '
                    f'x-google-min-bitrate={bitrate_kbps};'
                    f'x-google-max-bitrate={bitrate_kbps};'
                    f'x-google-start-bitrate={bitrate_kbps}'
                )
        sdp = '\n'.join(new_lines)
        
        return web.json_response({
            "sdp": sdp,
            "type": pc.localDescription.type
        })
    
    async def _webrtc_shutdown(self, app):
        """Cleanup on WebRTC server shutdown"""
        global webrtc_pcs
        coros = [pc.close() for pc in webrtc_pcs]
        await asyncio.gather(*coros)
        webrtc_pcs.clear()
    
    def _create_ssl_context(self):
        """Create SSL context for HTTPS"""
        import ssl
        
        ssl_keyfile = "key.pem"
        ssl_certfile = "cert.pem"
        
        if not (os.path.exists(ssl_keyfile) and os.path.exists(ssl_certfile)):
            module_dir = os.path.dirname(os.path.abspath(__file__))
            ssl_keyfile = os.path.join(module_dir, "key.pem")
            ssl_certfile = os.path.join(module_dir, "cert.pem")
        
        if os.path.exists(ssl_keyfile) and os.path.exists(ssl_certfile):
            ssl_context = ssl.SSLContext(ssl.PROTOCOL_TLS_SERVER)
            ssl_context.minimum_version = ssl.TLSVersion.TLSv1_2
            ssl_context.maximum_version = ssl.TLSVersion.TLSv1_3
            ssl_context.set_ciphers(
                "ECDHE+AESGCM:ECDHE+CHACHA20:DHE+AESGCM:DHE+CHACHA20:"
                "ECDH+AESGCM:DH+AESGCM:ECDH+AES:DH+AES:RSA+AESGCM:RSA+AES:!aNULL:!eNULL:!MD5"
            )
            ssl_context.load_cert_chain(ssl_certfile, ssl_keyfile)
            print(f"Using SSL: {ssl_certfile}")
            return ssl_context
        else:
            print("Warning: SSL certificates not found")
            return None
    
    def destroy_node(self):
        """Cleanup on shutdown"""
        if hasattr(self, 'pipeline'):
            self.pipeline.set_state(Gst.State.NULL)
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = GStreamerCameraNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
