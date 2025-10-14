import rclpy
from rclpy.node import Node
from std_msgs.msg import Header
from alfie_msgs.msg import AudioFrame
import sounddevice as sd
import numpy as np
from rclpy.qos import QoSProfile, ReliabilityPolicy
import time
from alfie_mic.usb_4_mic_array.tuning import Tuning
from alfie_mic.pixel_ring import pixel_ring
import usb.core

SAMPLE_RATE = 16000  # ReSpeaker default sample rate
BLOCKSIZE = 512  # 512 samples per frame to match AudioFrame message
CHANNELS = 1
STREAM_RESET_INTERVAL = 3600  # seconds, configurable

class AudioPublisher(Node):
    def __init__(self):
        super().__init__('audio_publisher')

        self.mic = usb.core.find(idVendor=0x2886, idProduct=0x0018)
        #print dev
        if self.mic:
            mic_info = str(self.mic).split('\n')[0]
            self.get_logger().info(f'Alfie Mic found: {mic_info}')
            Mic_tuning = Tuning(self.mic)
            Mic_tuning.write("AGCONOFF",1)
            Mic_tuning.write("AGCGAIN",250)
            Mic_tuning.write("ECHOONOFF",1)
            Mic_tuning.write("AGCMAXGAIN",250)
            Mic_tuning.write("AGCDESIREDLEVEL",20)

        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.publisher_ = self.create_publisher(AudioFrame, 'audio_frames', qos)
        
        # Find and use the ReSpeaker 4 Mic Array device
        self.audio_device = None
        devices = sd.query_devices()
        for i, device in enumerate(devices):
            if 'ReSpeaker 4 Mic Array' in device['name']:
                self.audio_device = i
                self.get_logger().info(f'Found ReSpeaker device at index {i}: {device["name"]}')
                break
        
        if self.audio_device is None:
            self.get_logger().warn('ReSpeaker 4 Mic Array not found, using default audio device')
        
        self.stream = sd.InputStream(
            device=self.audio_device,
            samplerate=SAMPLE_RATE,
            blocksize=BLOCKSIZE,
            channels=CHANNELS,
            dtype='int16',
            callback=self.audio_callback
        )
        self.stream.start()
        self.last_stream_reset = time.time()

    def audio_callback(self, indata, frames, time_info, status):
        if status:
            self.get_logger().warn(f'Stream status: {status}')
        
        msg = AudioFrame()
        # Convert and ensure we have exactly 512 samples
        audio_data = indata.copy().flatten().astype(np.int16)
        
        # Debug: log audio data info occasionally
        if hasattr(self, '_debug_counter'):
            self._debug_counter += 1
        else:
            self._debug_counter = 0
            
        #if self._debug_counter % 100 == 0:  # Log every 100th callback
        #    self.get_logger().info(f'Audio data: shape={audio_data.shape}, min={audio_data.min()}, max={audio_data.max()}, mean={audio_data.mean():.2f}')
        
        if len(audio_data) >= 512:
            msg.audioframe = audio_data[:512].tolist()
        else:
            # Pad with zeros if we have fewer samples
            padded_data = np.zeros(512, dtype=np.int16)
            padded_data[:len(audio_data)] = audio_data
            msg.audioframe = padded_data.tolist()
            
        self.publisher_.publish(msg)
        # Check if it's time to reset the stream
        now = time.time()
        if now - self.last_stream_reset > STREAM_RESET_INTERVAL:
            self.get_logger().info('Resetting audio input stream to prevent overflow.')
            self.stream.stop()
            self.stream.close()
            self.stream = sd.InputStream(
                device=self.audio_device,
                samplerate=SAMPLE_RATE,
                blocksize=BLOCKSIZE,
                channels=CHANNELS,
                dtype='int16',
                callback=self.audio_callback
            )
            self.stream.start()
            self.last_stream_reset = now


def main(args=None):
    rclpy.init(args=args)
    node = AudioPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stream.stop()
        node.stream.close()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
