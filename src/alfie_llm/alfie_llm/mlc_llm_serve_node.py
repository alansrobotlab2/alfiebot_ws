import rclpy
from rclpy.node import Node
import subprocess
import threading
import signal
import sys
import time

class MLCLLMServeNode(Node):
    def __init__(self):
        super().__init__('mlc_llm_serve_node')
        self.container_name = f'mlc-llm-server-{int(time.time())}'
        self.container_id = None
        self.process = None
        self.get_logger().info('Starting mlc llm server...')
        self.mlc_thread = threading.Thread(target=self.run_mlc_llm_serve, daemon=True)
        self.mlc_thread.start()

        self.model = '/data/qwen3-1.7b-q4f16_1-MLC'
        self.lib = '/data/qwen3-1.7b-q4f16_1-MLC/lib.so'
        
    def run_mlc_llm_serve(self):
        try:
            # Direct docker run command without interactive flags (replacing jetson-containers run)
            # Based on jetson-containers run.sh for tegra-aarch64 but removing -it flags
            process = subprocess.Popen([
                'docker', 'run',
                '--runtime', 'nvidia',
                '--env', 'NVIDIA_DRIVER_CAPABILITIES=compute,utility,graphics',
                '--rm',
                '--network', 'host',
                '--shm-size=8g',
                '--volume', '/tmp/argus_socket:/tmp/argus_socket',
                '--volume', '/etc/enctune.conf:/etc/enctune.conf',
                '--volume', '/etc/nv_tegra_release:/etc/nv_tegra_release',
                '--volume', '/tmp/nv_jetson_model:/tmp/nv_jetson_model',
                '--volume', '/var/run/dbus:/var/run/dbus',
                '--volume', '/var/run/avahi-daemon/socket:/var/run/avahi-daemon/socket',
                '--volume', '/var/run/docker.sock:/var/run/docker.sock',
                '--volume', '/home/alfie/repos/jetson-containers/data:/data',
                '-v', '/etc/localtime:/etc/localtime:ro',
                '-v', '/etc/timezone:/etc/timezone:ro',
                '--device', '/dev/snd',
                '-e', 'PULSE_SERVER=unix:/run/user/1000/pulse/native',
                '-v', '/run/user/1000/pulse:/run/user/1000/pulse',
                '--device', '/dev/bus/usb',
                '--device', '/dev/i2c-0',
                '--device', '/dev/i2c-1', 
                '--device', '/dev/i2c-2',
                '--device', '/dev/i2c-4',
                '--device', '/dev/i2c-5',
                '--device', '/dev/i2c-7',
                '--device', '/dev/i2c-9',
                '-v', '/run/jtop.sock:/run/jtop.sock',
                '--name', self.container_name,
                'dustynv/mlc:0.20.0-r36.4.0',
                'mlc_llm', 'serve', self.model,
                '--model-lib', self.lib,
                '--host', '0.0.0.0',
                '--mode', 'interactive',
                '--device', 'cuda'
            ])
            process.wait()
        except Exception as e:
            self.get_logger().error(f'Failed to launch mlc llm server: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = MLCLLMServeNode()
  
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('KeyboardInterrupt received, shutting down...')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
