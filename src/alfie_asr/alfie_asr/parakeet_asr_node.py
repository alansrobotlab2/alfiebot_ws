import os
import sys

# Set ONNX Runtime options BEFORE importing any ONNX-related libraries
os.environ['ORT_DISABLE_AFFINITY'] = '1'  # Disable thread affinity
os.environ['OMP_NUM_THREADS'] = '4'  # Set number of OpenMP threads

import threading
import time
import numpy as np
import onnxruntime as ort
import onnx_asr
import gc
import psutil

# IMPORTANT: Load ASR model BEFORE importing PyTorch/Silero to avoid cuBLAS conflict
# ONNX Runtime and PyTorch both try to create cuBLAS handles, but ONNX Runtime fails
# if PyTorch initializes CUDA first
print("Pre-loading Parakeet ASR model with GPU (before PyTorch)...")
ort.set_default_logger_severity(3)
_sess_options = ort.SessionOptions()
_sess_options.intra_op_num_threads = 4
_sess_options.inter_op_num_threads = 4
_sess_options.execution_mode = ort.ExecutionMode.ORT_SEQUENTIAL
_sess_options.graph_optimization_level = ort.GraphOptimizationLevel.ORT_ENABLE_ALL
_providers = ['CUDAExecutionProvider', 'CPUExecutionProvider']
_asr_model = onnx_asr.load_model(
    "istupakov/parakeet-tdt-0.6b-v2-onnx",
    quantization="int8",
    sess_options=_sess_options,
    providers=_providers
)
print("ASR model pre-loaded successfully")

# Now safe to import PyTorch-based libraries
import rclpy
from rclpy.node import Node
from alfie_msgs.msg import AudioFrame, ASRResult
from alfie_msgs.msg import Speaking
from silero_vad import load_silero_vad, VADIterator
from rclpy.qos import QoSProfile, ReliabilityPolicy
from ament_index_python.packages import get_package_share_directory
import shutil

SAMPLE_RATE = 16000  # Updated to match audio publisher
BLOCKSIZE = 512   # Updated to match AudioFrame message
VAD_BLOCK = 512   # VAD requires minimum 512 samples (32ms at 16kHz)
FRAMES_PER_VAD = VAD_BLOCK // BLOCKSIZE  # 2 frames needed for VAD
MAX_SPEECH_SECONDS = 10 # Max speech segment length
MAX_AUDIO_BUFFER_SIZE = SAMPLE_RATE * MAX_SPEECH_SECONDS  # max buffer (safety limit)


class ASRNode(Node):
    def __init__(self):
        super().__init__('asr_node')
        
        # Configure ONNX Runtime for GPU usage
        ort.set_default_logger_severity(3)  # Reduce logging verbosity
        
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.speaking = False
        self.audio_buffer = bytearray()
        self.micstate = 'IDLE'

        # Memory monitoring
        self.process = psutil.Process()
        self.callback_count = 0
        self.last_memory_log = 0
        
        # Use the pre-loaded ASR model (loaded at module level before PyTorch)
        self.asr_model = _asr_model
        self.get_logger().info('Using pre-loaded Parakeet ASR model')

        # Load VAD model (PyTorch-based, safe to load after ONNX Runtime CUDA is initialized)
        self.get_logger().info('Loading Silero VAD model...')
        self.silero_model = load_silero_vad(onnx=True)
        self.vad_iterator = VADIterator(
            self.silero_model,
            sampling_rate=SAMPLE_RATE,
            min_silence_duration_ms=500,
            speech_pad_ms=30,
            threshold=0.35,
        )
        
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
        # Memory monitoring (every 1000 callbacks)
        # self.callback_count += 1
        # if self.callback_count % 1000 == 0:
        #     mem_info = self.process.memory_info()
        #     mem_mb = mem_info.rss / 1024 / 1024
        #     self.get_logger().info(f'Memory usage: {mem_mb:.1f} MB | Buffer size: {len(self.audio_buffer)} bytes')
        #     self.last_memory_log = mem_mb

        # only process the frames if tts isn't speaking
        if self.speaking == False:
            samples = np.frombuffer(msg.audioframe, dtype=np.int16)
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
                # Safety check: prevent buffer overflow
                if len(self.audio_buffer) < MAX_AUDIO_BUFFER_SIZE * 2:
                    self.audio_buffer.extend(samples.tobytes())
                else:
                    self.get_logger().warn('Audio buffer exceeded maximum size, resetting')
                    self.micstate = 'IDLE'
                    self.audio_buffer = bytearray()
                    self.vad_iterator.reset_states()
            elif self.micstate == 'SPEECH' and key == 'end':
                self.get_logger().info('Speech end detected, transcribing...')
                self.audio_buffer.extend(samples.tobytes())
                audio_np = np.frombuffer(self.audio_buffer, dtype=np.int16)
                result = self.asr_model.recognize(audio_np, sample_rate=SAMPLE_RATE)
                asr_msg = ASRResult()
                asr_msg.asrresult = str(result)
                self.get_logger().info('Speech detected, publishing result: ' + asr_msg.asrresult)
                self.publisher_.publish(asr_msg)

                # Explicit cleanup
                del audio_np
                self.audio_buffer = bytearray()
                self.micstate = 'IDLE'
                self.vad_iterator.reset_states()

                # Force garbage collection after transcription
                gc.collect()


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
