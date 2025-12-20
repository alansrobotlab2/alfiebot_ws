#!/usr/bin/env python3
"""
GStreamer-based low-latency camera node for ROS2 with Hardware-Accelerated WebRTC

Uses GStreamer pipeline to capture MJPEG and publish directly to compressed topic.
WebRTC streaming uses GStreamer's webrtcbin with Jetson hardware H264 encoder
for minimal CPU usage.

Key difference from aiortc version:
- Uses nvv4l2h264enc (GPU) instead of libvpx (CPU) for WebRTC encoding
- Entire video pipeline stays on GPU until final RTP packetization
"""

import asyncio
import json
import os
import ssl
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage, CameraInfo
from std_msgs.msg import Header
import gi
gi.require_version('Gst', '1.0')
gi.require_version('GstWebRTC', '1.0')
gi.require_version('GstSdp', '1.0')
from gi.repository import Gst, GLib, GstWebRTC, GstSdp
import threading
from typing import Optional, Dict, Any

from aiohttp import web


class GStreamerCameraNode(Node):
    def __init__(self):
        super().__init__('gstreamer_camera')
        
        # Declare parameters
        self.declare_parameter('device', '/dev/video0')
        self.declare_parameter('width', 2560)
        self.declare_parameter('height', 720)
        self.declare_parameter('framerate', 15)
        self.declare_parameter('flip_vertical', True)
        self.declare_parameter('camera_frame_id', 'stereo_camera_link')
        self.declare_parameter('enable_webrtc', True)
        self.declare_parameter('webrtc_port', 8084)
        self.declare_parameter('webrtc_bitrate', 3000000)  # 3 Mbps
        self.declare_parameter('use_hardware_accel', True)
        self.declare_parameter('jpeg_quality', 85)
        
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
        
        # WebRTC state
        self._webrtc_peers: Dict[str, Any] = {}  # peer_id -> {pipeline, webrtcbin, ...}
        self._webrtc_lock = threading.Lock()
        self._pending_ice = {}  # peer_id -> list of ice candidates
        self._webrtc_loop = None
        
        # Publishers
        self.image_pub = self.create_publisher(
            CompressedImage,
            'image_raw/compressed',
            1
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
            self.get_logger().warning('⚠️  Hardware acceleration not available, falling back to CPU')
            self.use_hardware_accel = False
        elif self.use_hardware_accel and self.hardware_available:
            self.get_logger().info('🚀 Using hardware acceleration (including WebRTC H264!)')
        
        # Build GStreamer pipeline for ROS (camera capture only, no WebRTC)
        self.pipeline = self._create_ros_pipeline()
        
        # Start pipeline in separate thread
        self.pipeline_thread = threading.Thread(target=self._run_pipeline, daemon=True)
        self.pipeline_thread.start()
        
        self.get_logger().info(f'GStreamer camera started: {self.device} @ {self.width}x{self.height} {self.framerate}fps')
        if self.flip_vertical:
            self.get_logger().info('Vertical flip enabled')
        
        # Start WebRTC signaling server if enabled
        if self.enable_webrtc:
            self.webrtc_thread = threading.Thread(
                target=self._run_webrtc_server,
                daemon=True
            )
            self.webrtc_thread.start()
            self.get_logger().info(f'🎥 WebRTC server (HW H264) enabled on port {self.webrtc_port}')
            self.get_logger().info(f'   Bitrate: {self.webrtc_bitrate/1000000:.1f} Mbps')
        else:
            self.get_logger().info('WebRTC server disabled')
    
    def _check_hardware_support(self):
        """Check if Nvidia hardware acceleration plugins are available"""
        required_elements = ['nvv4l2decoder', 'nvvidconv', 'nvjpegenc', 'nvv4l2h264enc', 'webrtcbin']
        
        for elem_name in required_elements:
            elem = Gst.ElementFactory.find(elem_name)
            if elem is None:
                self.get_logger().warning(f'Missing GStreamer element: {elem_name}')
                return False
        
        self.get_logger().info('✅ All hardware acceleration elements available (incl. H264 encoder)')
        return True
    
    def _create_ros_pipeline(self):
        """Create GStreamer pipeline for ROS only (JPEG passthrough or flip)"""
        
        if self.use_hardware_accel and self.flip_vertical:
            # Hardware decode, flip, and re-encode for ROS
            flip_method = 2  # vertical flip
            pipeline_str = (
                f'v4l2src device={self.device} ! '
                f'image/jpeg,width={self.width},height={self.height},framerate={self.framerate}/1 ! '
                f'nvv4l2decoder mjpeg=1 ! '
                f'nvvidconv flip-method={flip_method} ! '
                f'video/x-raw(memory:NVMM),format=I420 ! '
                f'nvjpegenc quality={self.jpeg_quality} ! '
                f'appsink name=ros_sink emit-signals=true max-buffers=1 drop=true sync=false'
            )
            self.get_logger().info(f'🚀 ROS Pipeline: HW Decode→Flip→Encode')
        elif self.use_hardware_accel:
            # No flip - direct JPEG passthrough (zero copy)
            pipeline_str = (
                f'v4l2src device={self.device} ! '
                f'image/jpeg,width={self.width},height={self.height},framerate={self.framerate}/1 ! '
                f'appsink name=ros_sink emit-signals=true max-buffers=1 drop=true sync=false'
            )
            self.get_logger().info(f'🚀 ROS Pipeline: Zero-copy JPEG passthrough')
        else:
            # Software fallback
            pipeline_str = (
                f'v4l2src device={self.device} ! '
                f'image/jpeg,width={self.width},height={self.height},framerate={self.framerate}/1 ! '
                f'appsink name=ros_sink emit-signals=true max-buffers=1 drop=true sync=false'
            )
            self.get_logger().info(f'🖥️  ROS Pipeline: Software (JPEG passthrough)')
        
        pipeline = Gst.parse_launch(pipeline_str)
        ros_sink = pipeline.get_by_name('ros_sink')
        ros_sink.connect('new-sample', self._on_ros_sample)
        
        return pipeline
    
    def _on_ros_sample(self, sink):
        """Handle ROS JPEG frames"""
        sample = sink.emit('pull-sample')
        if sample is None:
            return Gst.FlowReturn.OK
        
        buf = sample.get_buffer()
        receive_time = self.get_clock().now()
        
        success, map_info = buf.map(Gst.MapFlags.READ)
        if not success:
            return Gst.FlowReturn.OK
        
        try:
            jpeg_data = bytes(map_info.data)
            
            # If software mode with flip needed, we need to decode/flip/encode
            # (This shouldn't happen if hardware_accel is working)
            
            msg = CompressedImage()
            msg.header = Header()
            msg.header.stamp = receive_time.to_msg()
            msg.header.frame_id = self.frame_id
            msg.format = 'jpeg'
            msg.data = jpeg_data
            
            self.image_pub.publish(msg)
            
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
                mode = "HW" if self.use_hardware_accel else "SW"
                webrtc_clients = len(self._webrtc_peers)
                self.get_logger().info(f'[{mode}] {fps:.1f} FPS, WebRTC clients: {webrtc_clients}')
                self.frame_count = 0
                self.last_log_time = now
        
        finally:
            buf.unmap(map_info)
        
        return Gst.FlowReturn.OK
    
    def _run_pipeline(self):
        """Run GStreamer pipeline in separate thread"""
        self.pipeline.set_state(Gst.State.PLAYING)
        
        loop = GLib.MainLoop()
        bus = self.pipeline.get_bus()
        bus.add_signal_watch()
        bus.connect('message', self._on_bus_message, loop)
        
        try:
            loop.run()
        except KeyboardInterrupt:
            pass
        
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
    
    # =========== WebRTC with Hardware H264 ===========
    
    def _create_webrtc_pipeline(self, peer_id: str):
        """Create a per-client WebRTC pipeline with hardware H264 encoding"""
        
        bitrate_kbps = self.webrtc_bitrate // 1000
        flip_method = 2 if self.flip_vertical else 0
        
        self.get_logger().info(f'🎥 WebRTC pipeline for peer {peer_id}: HW H264 @ {bitrate_kbps} kbps')
        
        # Create pipeline manually - webrtcbin requires dynamic pad linking
        pipeline = Gst.Pipeline.new(f'webrtc-{peer_id}')
        
        # Create elements
        v4l2src = Gst.ElementFactory.make('v4l2src', 'src')
        v4l2src.set_property('device', self.device)
        
        jpegcaps = Gst.Caps.from_string(
            f'image/jpeg,width={self.width},height={self.height},framerate={self.framerate}/1'
        )
        capsfilter1 = Gst.ElementFactory.make('capsfilter', 'jpegcaps')
        capsfilter1.set_property('caps', jpegcaps)
        
        decoder = Gst.ElementFactory.make('nvv4l2decoder', 'decoder')
        decoder.set_property('mjpeg', 1)
        
        vidconv = Gst.ElementFactory.make('nvvidconv', 'vidconv')
        vidconv.set_property('flip-method', flip_method)
        
        nvmmcaps = Gst.Caps.from_string('video/x-raw(memory:NVMM),format=I420')
        capsfilter2 = Gst.ElementFactory.make('capsfilter', 'nvmmcaps')
        capsfilter2.set_property('caps', nvmmcaps)
        
        encoder = Gst.ElementFactory.make('nvv4l2h264enc', 'encoder')
        encoder.set_property('bitrate', self.webrtc_bitrate)
        encoder.set_property('preset-level', 1)  # UltraFast
        encoder.set_property('maxperf-enable', True)
        encoder.set_property('insert-sps-pps', True)
        encoder.set_property('idrinterval', self.framerate)  # Keyframe every second
        
        h264parse = Gst.ElementFactory.make('h264parse', 'parse')
        h264parse.set_property('config-interval', -1)
        
        rtppay = Gst.ElementFactory.make('rtph264pay', 'pay')
        rtppay.set_property('config-interval', -1)
        rtppay.set_property('pt', 96)
        
        # Queue before webrtcbin for thread safety
        queue = Gst.ElementFactory.make('queue', 'queue')
        queue.set_property('max-size-buffers', 1)
        queue.set_property('leaky', 2)  # downstream
        
        webrtcbin = Gst.ElementFactory.make('webrtcbin', 'webrtc')
        webrtcbin.set_property('bundle-policy', 3)  # max-bundle
        webrtcbin.set_property('stun-server', 'stun://stun.l.google.com:19302')
        
        # Add all elements to pipeline
        for elem in [v4l2src, capsfilter1, decoder, vidconv, capsfilter2, 
                     encoder, h264parse, rtppay, queue, webrtcbin]:
            pipeline.add(elem)
        
        # Link elements up to queue
        v4l2src.link(capsfilter1)
        capsfilter1.link(decoder)
        decoder.link(vidconv)
        vidconv.link(capsfilter2)
        capsfilter2.link(encoder)
        encoder.link(h264parse)
        h264parse.link(rtppay)
        rtppay.link(queue)
        
        # Link queue to webrtcbin via request pad
        queue_src = queue.get_static_pad('src')
        webrtc_sink = webrtcbin.request_pad_simple('sink_%u')
        if webrtc_sink is None:
            # Fallback for older GStreamer
            webrtc_sink = webrtcbin.get_request_pad('sink_%u')
        
        if queue_src and webrtc_sink:
            queue_src.link(webrtc_sink)
            self.get_logger().info(f'Linked queue to webrtcbin sink pad')
        else:
            self.get_logger().error(f'Failed to link to webrtcbin')
        
        # Connect signals
        webrtcbin.connect('on-negotiation-needed', self._on_negotiation_needed, peer_id)
        webrtcbin.connect('on-ice-candidate', self._on_ice_candidate, peer_id)
        webrtcbin.connect('notify::connection-state', self._on_connection_state, peer_id)
        
        return pipeline, webrtcbin
    
    def _on_negotiation_needed(self, webrtcbin, peer_id):
        """Called when webrtcbin needs to negotiate - create offer"""
        self.get_logger().debug(f'Negotiation needed for peer {peer_id}')
        promise = Gst.Promise.new_with_change_func(self._on_offer_created, webrtcbin, peer_id)
        webrtcbin.emit('create-offer', None, promise)
    
    def _on_offer_created(self, promise, webrtcbin, peer_id):
        """Called when offer is created"""
        promise.wait()
        reply = promise.get_reply()
        offer = reply.get_value('offer')
        
        # Set local description
        promise2 = Gst.Promise.new()
        webrtcbin.emit('set-local-description', offer, promise2)
        promise2.interrupt()
        
        # Store offer for signaling
        sdp_text = offer.sdp.as_text()
        with self._webrtc_lock:
            if peer_id in self._webrtc_peers:
                self._webrtc_peers[peer_id]['local_sdp'] = sdp_text
                self._webrtc_peers[peer_id]['local_sdp_ready'].set()
    
    def _on_ice_candidate(self, webrtcbin, mline_index, candidate, peer_id):
        """Called when ICE candidate is generated"""
        with self._webrtc_lock:
            if peer_id in self._webrtc_peers:
                ice_list = self._webrtc_peers[peer_id].get('ice_candidates', [])
                ice_list.append({'candidate': candidate, 'sdpMLineIndex': mline_index})
                self._webrtc_peers[peer_id]['ice_candidates'] = ice_list
    
    def _on_connection_state(self, webrtcbin, pspec, peer_id):
        """Called when connection state changes"""
        state = webrtcbin.get_property('connection-state')
        self.get_logger().info(f'WebRTC peer {peer_id} state: {state}')
        
        if state in [GstWebRTC.WebRTCPeerConnectionState.FAILED,
                     GstWebRTC.WebRTCPeerConnectionState.CLOSED]:
            self._cleanup_peer(peer_id)
    
    def _cleanup_peer(self, peer_id):
        """Clean up a peer's WebRTC resources"""
        with self._webrtc_lock:
            if peer_id in self._webrtc_peers:
                peer = self._webrtc_peers.pop(peer_id)
                pipeline = peer.get('pipeline')
                if pipeline:
                    pipeline.set_state(Gst.State.NULL)
                self.get_logger().info(f'Cleaned up peer {peer_id}')
    
    # =========== HTTP Signaling Server ===========
    
    def _run_webrtc_server(self):
        """Run WebRTC signaling server"""
        try:
            loop = asyncio.new_event_loop()
            asyncio.set_event_loop(loop)
            self._webrtc_loop = loop
            loop.run_until_complete(self._webrtc_server_async())
        except Exception as e:
            self.get_logger().error(f'WebRTC server error: {e}')
            import traceback
            traceback.print_exc()
    
    async def _webrtc_server_async(self):
        """Async WebRTC signaling server"""
        app = web.Application()
        
        # CORS middleware
        @web.middleware
        async def cors_middleware(request, handler):
            if request.method == "OPTIONS":
                response = web.Response()
            else:
                try:
                    response = await handler(request)
                except web.HTTPException as ex:
                    response = ex
            response.headers["Access-Control-Allow-Origin"] = "*"
            response.headers["Access-Control-Allow-Methods"] = "POST, GET, OPTIONS"
            response.headers["Access-Control-Allow-Headers"] = "Content-Type"
            return response
        
        app.middlewares.append(cors_middleware)
        app.router.add_post("/offer", self._handle_offer)
        app.router.add_post("/answer", self._handle_answer)
        app.router.add_get("/ice/{peer_id}", self._get_ice_candidates)
        
        ssl_context = self._create_ssl_context()
        
        runner = web.AppRunner(app)
        await runner.setup()
        site = web.TCPSite(runner, "0.0.0.0", self.webrtc_port, ssl_context=ssl_context)
        await site.start()
        
        protocol = "https" if ssl_context else "http"
        self.get_logger().info(f'WebRTC signaling: {protocol}://0.0.0.0:{self.webrtc_port}')
        
        while True:
            await asyncio.sleep(3600)
    
    async def _handle_offer(self, request):
        """
        Client sends offer, we respond with our answer.
        For GstWebRTC, we actually create the offer (server-initiated).
        This endpoint triggers pipeline creation and returns our offer.
        """
        import uuid
        peer_id = str(uuid.uuid4())[:8]
        
        # Create event for SDP ready
        local_sdp_ready = asyncio.Event()
        
        # Create pipeline
        pipeline, webrtcbin = self._create_webrtc_pipeline(peer_id)
        
        with self._webrtc_lock:
            self._webrtc_peers[peer_id] = {
                'pipeline': pipeline,
                'webrtcbin': webrtcbin,
                'local_sdp': None,
                'local_sdp_ready': local_sdp_ready,
                'ice_candidates': []
            }
        
        # Start pipeline - this triggers negotiation-needed
        pipeline.set_state(Gst.State.PLAYING)
        
        # Wait for offer to be created
        try:
            await asyncio.wait_for(local_sdp_ready.wait(), timeout=5.0)
        except asyncio.TimeoutError:
            self._cleanup_peer(peer_id)
            return web.json_response({'error': 'Timeout creating offer'}, status=500)
        
        with self._webrtc_lock:
            local_sdp = self._webrtc_peers[peer_id].get('local_sdp')
        
        if not local_sdp:
            self._cleanup_peer(peer_id)
            return web.json_response({'error': 'Failed to create offer'}, status=500)
        
        return web.json_response({
            'peer_id': peer_id,
            'sdp': local_sdp,
            'type': 'offer'
        })
    
    async def _handle_answer(self, request):
        """Client sends their answer to our offer"""
        try:
            params = await request.json()
            peer_id = params.get('peer_id')
            sdp = params.get('sdp')
            
            if not peer_id or not sdp:
                return web.json_response({'error': 'Missing peer_id or sdp'}, status=400)
            
            with self._webrtc_lock:
                peer = self._webrtc_peers.get(peer_id)
                if not peer:
                    return web.json_response({'error': 'Unknown peer'}, status=404)
                webrtcbin = peer['webrtcbin']
            
            # Parse and set remote description (the answer)
            res, sdpmsg = GstSdp.SDPMessage.new()
            GstSdp.sdp_message_parse_buffer(bytes(sdp, 'utf-8'), sdpmsg)
            answer = GstWebRTC.WebRTCSessionDescription.new(
                GstWebRTC.WebRTCSDPType.ANSWER, sdpmsg
            )
            
            promise = Gst.Promise.new()
            webrtcbin.emit('set-remote-description', answer, promise)
            promise.interrupt()
            
            # Add any pending ICE candidates from client
            pending = params.get('ice_candidates', [])
            for ice in pending:
                webrtcbin.emit('add-ice-candidate',
                              ice.get('sdpMLineIndex', 0),
                              ice.get('candidate', ''))
            
            return web.json_response({'status': 'ok'})
            
        except Exception as e:
            self.get_logger().error(f'Error handling answer: {e}')
            return web.json_response({'error': str(e)}, status=500)
    
    async def _get_ice_candidates(self, request):
        """Return ICE candidates for a peer"""
        peer_id = request.match_info['peer_id']
        
        with self._webrtc_lock:
            peer = self._webrtc_peers.get(peer_id)
            if not peer:
                return web.json_response({'error': 'Unknown peer'}, status=404)
            
            candidates = peer.get('ice_candidates', [])
            # Clear after returning
            peer['ice_candidates'] = []
        
        return web.json_response({'candidates': candidates})
    
    def _create_ssl_context(self):
        """Create SSL context for HTTPS"""
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
            self.get_logger().info(f'Using SSL: {ssl_certfile}')
            return ssl_context
        else:
            self.get_logger().warning('SSL certificates not found, using HTTP')
            return None
    
    def destroy_node(self):
        """Cleanup on shutdown"""
        # Cleanup all WebRTC peers
        with self._webrtc_lock:
            for peer_id in list(self._webrtc_peers.keys()):
                self._cleanup_peer(peer_id)
        
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
