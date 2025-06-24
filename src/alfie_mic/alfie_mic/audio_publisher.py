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

SAMPLE_RATE = 8000
BLOCKSIZE = 256  # 512 samples per frame
CHANNELS = 1
STREAM_RESET_INTERVAL = 3600  # seconds, configurable

class AudioPublisher(Node):
    def __init__(self):
        super().__init__('audio_publisher')

        self.mic = usb.core.find(idVendor=0x2886, idProduct=0x0018)
        #print dev
        if self.mic:
            Mic_tuning = Tuning(self.mic)
            Mic_tuning.write("AGCONOFF",1)
            Mic_tuning.write("AGCGAIN",250)
            Mic_tuning.write("ECHOONOFF",1)
            Mic_tuning.write("AGCMAXGAIN",250)
            Mic_tuning.write("AGCDESIREDLEVEL",20)

        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.publisher_ = self.create_publisher(AudioFrame, 'audio_frames', qos)
        self.stream = sd.InputStream(
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
        msg.audioframe = indata.copy().flatten().tolist()
        self.publisher_.publish(msg)
        # Check if it's time to reset the stream
        now = time.time()
        if now - self.last_stream_reset > STREAM_RESET_INTERVAL:
            self.get_logger().info('Resetting audio input stream to prevent overflow.')
            self.stream.stop()
            self.stream.close()
            self.stream = sd.InputStream(
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
