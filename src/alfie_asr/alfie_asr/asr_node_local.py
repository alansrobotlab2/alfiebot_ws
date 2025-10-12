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

def setup_local_vad_model(package_models_dir):
    """Setup local VAD model by copying it to the expected location"""
    import silero_vad
    
    # Get the expected data directory for silero_vad
    silero_data_dir = os.path.join(os.path.dirname(silero_vad.__file__), 'data')
    if not os.path.exists(silero_data_dir):
        os.makedirs(silero_data_dir)
    
    # Copy our local model to the expected location if it doesn't exist
    expected_model_path = os.path.join(silero_data_dir, 'silero_vad_16k_op15.onnx')
    local_model_path = os.path.join(package_models_dir, 'silero_vad_16k_op15.onnx')
    
    if not os.path.exists(expected_model_path) and os.path.exists(local_model_path):
        shutil.copy2(local_model_path, expected_model_path)
    
    return expected_model_path

class ASRNode(Node):
    def __init__(self):
        super().__init__('asr_node')
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.speaking = False
        self.audio_buffer = bytearray()
        self.vad_buffer = np.zeros(0, dtype=np.int16)  # Buffer for VAD processing
        self.frame_count = 0  # Counter for accumulating frames
        self.micstate = 'IDLE'
        
        # Get the package directory and model paths
        package_share_directory = get_package_share_directory('alfie_asr')
        models_dir = os.path.join(package_share_directory, 'models')
        
        self.get_logger().info(f'Loading models from package directory: {models_dir}')
        
        # Setup local VAD model
        setup_local_vad_model(models_dir)
        
        # Load VAD model (will now use the local copy)
        self.get_logger().info('Loading Silero VAD model from package...')
        self.silero_model = load_silero_vad(onnx=True)
        self.vad_iterator = VADIterator(
            self.silero_model,
            sampling_rate=SAMPLE_RATE,  # Now 16000 Hz
            min_silence_duration_ms=500,
            speech_pad_ms=30,
            threshold=0.35,
        )

        # Load ASR model from local files
        self.get_logger().info('Loading Parakeet ASR model from package...')
        self.asr_model = onnx_asr.load_model("nemo-parakeet-tdt-0.6b-v2", path=models_dir, quantization="int8")
        
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

        self.get_logger().info('ASRNode initialized with local models from package.')

    def speaking_callback(self, msg):
        self.speaking = msg.is_speaking

    def audio_callback(self, msg):
        # only process the frames if tts isn't speaking
        if self.speaking == False:
            # AudioFrame now contains exactly 256 int16 samples
            samples = np.array(msg.audioframe, dtype=np.int16)
            
            # Accumulate samples in VAD buffer
            self.vad_buffer = np.concatenate([self.vad_buffer, samples])
            self.frame_count += 1
            
            # Process VAD when we have enough samples (512 samples = 2 frames)
            if len(self.vad_buffer) >= VAD_BLOCK:
                # Take the required number of samples for VAD
                vad_samples = self.vad_buffer[:VAD_BLOCK]
                
                # Normalize for VAD (convert to float32 in range [-1, 1])
                vad_input = vad_samples.astype(np.float32) / 32768.0
                speech = self.vad_iterator(vad_input)
                
                # Shift buffer to remove processed samples (overlap by half for smoother detection)
                overlap = VAD_BLOCK // 2
                self.vad_buffer = self.vad_buffer[overlap:]
                
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
            
            # Always add current samples to speech buffer if we're in speech mode
            if self.micstate == 'SPEECH':
                self.audio_buffer.extend(samples.tobytes())


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
