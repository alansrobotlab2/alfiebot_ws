"""llama_serve_node — ROS2 wrapper that launches the pre-built llama.cpp server
with the Qwen3.6-35B-A3B config tuned for Jetson Orin AGX 64GB.

Settings match alfie_llama.py: Q4_K_S + mmproj, 256k ctx, Q8 KV, flash-attn,
0.0.0.0:8001. The node assumes llama.cpp is already built at ~/llama.cpp/build
and the model files are present at ~/models/qwen3.6-35b-a3b — no downloads,
no docker. Use alfie_llama.py directly for quick local testing; this node
exists so the robot's launch file can start the LLM alongside other services.
"""

import os
import signal
import subprocess
import threading
from pathlib import Path

import rclpy
from rclpy.node import Node


class LlamaServeNode(Node):
    def __init__(self):
        super().__init__('llama_serve_node')

        self.declare_parameter('llama_dir', str(Path.home() / 'llama.cpp'))
        self.declare_parameter('model_dir', str(Path.home() / 'models' / 'qwen3.6-35b-a3b'))
        self.declare_parameter('model_file', 'Qwen3.6-35B-A3B-UD-Q4_K_S.gguf')
        self.declare_parameter('mmproj_file', 'mmproj-F16.gguf')
        self.declare_parameter('host', '0.0.0.0')
        self.declare_parameter('port', 8001)
        self.declare_parameter('ctx_size', 262144)
        self.declare_parameter('use_mmproj', True)
        self.declare_parameter('api_key', '')

        self.process = None

        self.get_logger().info('Starting llama.cpp server node...')
        self.thread = threading.Thread(target=self._run, daemon=True)
        self.thread.start()

    def _p(self, name):
        return self.get_parameter(name).value

    def _run(self):
        llama_dir = Path(self._p('llama_dir')).expanduser()
        bin_path = llama_dir / 'build' / 'bin' / 'llama-server'
        lib_dir = llama_dir / 'build' / 'bin'
        model_dir = Path(self._p('model_dir')).expanduser()
        model = model_dir / self._p('model_file')
        mmproj = model_dir / self._p('mmproj_file')

        for required in (bin_path, model):
            if not required.exists():
                self.get_logger().error(f'missing: {required}')
                return
        use_mmproj = self._p('use_mmproj')
        if use_mmproj and not mmproj.exists():
            self.get_logger().error(f'missing mmproj: {mmproj} (set use_mmproj:=false for text-only)')
            return

        cmd = [
            str(bin_path),
            '--model', str(model),
            '-ngl', '99',
            '--ctx-size', str(self._p('ctx_size')),
            '-fa', 'on',
            '-ctk', 'q8_0', '-ctv', 'q8_0',
            '--temp', '0.6', '--top-p', '0.95', '--top-k', '20', '--min-p', '0.0',
            '--host', self._p('host'), '--port', str(self._p('port')),
        ]
        if use_mmproj:
            cmd += ['--mmproj', str(mmproj)]
        else:
            cmd += ['--cache-reuse', '256']
        api_key = self._p('api_key')
        if api_key:
            cmd += ['--api-key', api_key]

        env = os.environ.copy()
        env['LD_LIBRARY_PATH'] = f"{lib_dir}:{env.get('LD_LIBRARY_PATH', '')}".rstrip(':')

        self.get_logger().info(f'launching llama-server on {self._p("host")}:{self._p("port")}')
        try:
            self.process = subprocess.Popen(cmd, env=env)
            self.process.wait()
        except Exception as e:
            self.get_logger().error(f'Failed to launch llama-server: {e}')

    def destroy_node(self):
        if self.process and self.process.poll() is None:
            self.get_logger().info('stopping llama-server...')
            self.process.send_signal(signal.SIGTERM)
            try:
                self.process.wait(timeout=10)
            except subprocess.TimeoutExpired:
                self.process.kill()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = LlamaServeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('KeyboardInterrupt received, shutting down...')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
