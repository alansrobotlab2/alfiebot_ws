import threading
import time
import rclpy
from rclpy.node import Node
from alfie_msgs.msg import AudioFrame, ASRResult
from alfie_msgs.msg import Speaking
import numpy as np
import onnx_asr
from silero_vad import load_silero_vad, VADIterator
from rclpy.qos import QoSProfile, ReliabilityPolicy
import os
from ament_index_python.packages import get_package_share_directory
import shutil

SAMPLE_RATE = 16000  # Updated to match audio publisher
BLOCKSIZE = 512   # Updated to match AudioFrame message
VAD_BLOCK = 512   # VAD requires minimum 512 samples (32ms at 16kHz)
FRAMES_PER_VAD = VAD_BLOCK // BLOCKSIZE  # 2 frames needed for VAD


class ASRNode(Node):
    def __init__(self):
        super().__init__('asr_node')
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.speaking = False
        self.audio_buffer = bytearray()
        self.prev_samples = np.zeros(BLOCKSIZE, dtype=np.int16)
        self.micstate = 'IDLE'
        self.silero_model = load_silero_vad(onnx=True)
        self.vad_iterator = VADIterator(
            self.silero_model,
            sampling_rate=SAMPLE_RATE,
            min_silence_duration_ms=500,
            speech_pad_ms=30,
            threshold=0.35,
        )

        self.asr_model = onnx_asr.load_model("istupakov/parakeet-tdt-0.6b-v2-onnx", quantization="int8")
        
        self.publisher_ = self.create_publisher(ASRResult, 'asrresult', qos)

        self.subscription = self.create_subscription(
            AudioFrame,
            'audio_frames',
            self.audio_callback,
            qos)

        self.speaking_sub = self.create_subscription(
            Speaking,
            'speaking',
            self.speaking_callback,
            qos)

        self.get_logger().info('ASRNode initialized.')

    def speaking_callback(self, msg):
        self.speaking = msg.is_speaking

    def audio_callback(self, msg):
        # only process the frames if tts isn't speaking
        if self.speaking == False:
            samples = np.array(msg.audioframe, dtype=np.int16)
            # Use the full 256-sample frame for VAD
            vad_input = samples.astype(np.float32) / 32768.0
            speech = self.vad_iterator(vad_input)
            #self.get_logger().info(f'VAD output: {speech}')
            if speech:
                key, = speech
                print(key, " ", speech)
            else:
                key = "speaking"

            if self.micstate == 'IDLE' and key == 'start':
                self.get_logger().info('Speech start detected')            
                self.audio_buffer = bytearray()
                self.audio_buffer.extend(samples.tobytes())
                self.micstate = 'SPEECH'
            elif self.micstate == 'SPEECH' and key == 'speaking':
                self.audio_buffer.extend(samples.tobytes())
            elif self.micstate == 'SPEECH' and key == 'end':
                self.get_logger().info('Speech end detected, transcribing...')
                self.audio_buffer.extend(samples.tobytes())
                audio_np = np.frombuffer(self.audio_buffer, dtype=np.int16)
                result = self.asr_model.recognize(audio_np, sample_rate=SAMPLE_RATE)
                asr_msg = ASRResult()
                asr_msg.asrresult = str(result)
                self.get_logger().info('Speech detected, publishing result: ' + asr_msg.asrresult)
                self.publisher_.publish(asr_msg)
                self.micstate = 'IDLE'
                self.audio_buffer = bytearray()


def main(args=None):
    rclpy.init(args=args)
    node = ASRNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
