import sys
import threading
import time
import subprocess

import sounddevice as sd
from piper.voice import PiperVoice
import alsaaudio

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

from alfie_msgs.msg import SpeechRequest
from alfie_msgs.msg import Speaking


class AlfieTTS(Node):
    def __init__(self):

        super().__init__('alfie_speech')

        self.model_path = "/home/alfie/Documents/piper/voices/en_US-libritts_r-medium.onnx"
        self.config_path = "/home/alfie/Documents/piper/voices/en_US-libritts_r-medium.onnx.json"
        self.speaker_id = 65
        self.length_scale = None
        self.noise_scale = None
        self.noise_w = None
        self.sentence_silence = 0.15
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
        with sd.RawOutputStream(
            samplerate=self.voice.config.sample_rate,
            channels=1,
            dtype='int16',
            latency=self.latency,
        ) as output_stream:
            for chunk in self.voice.synthesize_stream_raw(
                msg.text,
                speaker_id=self.speaker_id,
                length_scale=self.length_scale,
                noise_scale=self.noise_scale,
                noise_w=self.noise_w,
                sentence_silence=self.sentence_silence,
            ):
                output_stream.write(chunk)
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