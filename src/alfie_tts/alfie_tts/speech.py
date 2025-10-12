import sys
import threading
import time
import subprocess
import os

import sounddevice as sd
from piper.voice import PiperVoice
from piper.config import SynthesisConfig
import alsaaudio

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from ament_index_python.packages import get_package_share_directory

from alfie_msgs.msg import SpeechRequest
from alfie_msgs.msg import Speaking


class AlfieTTS(Node):
    def __init__(self):

        super().__init__('alfie_speech')

        # Get package voices directory
        package_share_directory = get_package_share_directory('alfie_tts')
        voices_dir = os.path.join(package_share_directory, 'voices')
        
        self.model_path = os.path.join(voices_dir, 'en_US-libritts_r-medium.onnx')
        self.config_path = os.path.join(voices_dir, 'en_US-libritts_r-medium.onnx.json')
        self.speaker_id = 65
        self.length_scale = None
        self.noise_scale = None
        self.noise_w = None
        self.use_cuda = False
        self.output_device = 0
        self.latency = 0.15
        self.speaking = False

        self.speaking_pub = self.create_publisher(
            Speaking,
            'speaking',
            QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        )

        self.voice = PiperVoice.load(self.model_path, self.config_path, use_cuda=self.use_cuda)

        try:
            subprocess.run(["pactl", "set-default-sink", "0"], check=True)
            self.get_logger().info("Set default sink to 0 using pactl.")
        except Exception as e:
            self.get_logger().warn(f"Failed to set default sink: {e}")

        self.speech_request_subscriber = self.create_subscription(
            SpeechRequest,
            'speechrequest',
            self.speech_request_callback,
            QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        )
        self.speech_request_subscriber  # prevent unused variable warning

        # Publish initial state (not speaking)
        self.publish_speaking(False)

    def publish_speaking(self, is_speaking):
        msg = Speaking()
        msg.is_speaking = is_speaking
        self.speaking = is_speaking
        self.speaking_pub.publish(msg)

    def speech_request_callback(self, msg):
        self.get_logger().info(f"Received speech request: {msg.text}")
        self.publish_speaking(True)
        alsaaudio.PCM(cardindex=self.output_device)
        alsaaudio.Mixer("Master").setvolume(msg.volume)
        
        # Create synthesis config
        syn_config = SynthesisConfig(
            speaker_id=self.speaker_id,
            length_scale=self.length_scale,
            noise_scale=self.noise_scale,
            noise_w_scale=self.noise_w,
        )
        
        with sd.RawOutputStream(
            samplerate=self.voice.config.sample_rate,
            channels=1,
            dtype='int16',
            latency=self.latency,
        ) as output_stream:
            for audio_chunk in self.voice.synthesize(msg.text, syn_config):
                output_stream.write(audio_chunk.audio_int16_bytes)
        self.publish_speaking(False)



def main(args=None):
    rclpy.init(args=args)
    alfietts = AlfieTTS()

    try:
        rclpy.spin(alfietts)
    except KeyboardInterrupt:
        pass
    finally:
        alfietts.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()