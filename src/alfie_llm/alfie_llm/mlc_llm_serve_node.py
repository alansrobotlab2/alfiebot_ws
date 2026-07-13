"""
mlc_llm_serve_node — runs the local (custom-built) MLC-LLM OpenAI server.

Wraps `.venv/bin/python -m mlc_llm serve` from the source tree at ~/mlc-llm,
serving the converted Qwen3.6 35B-A3B model. The env comes from .envrc.local
(vendored TVM + mlc_llm on PYTHONPATH). The explicit --model-lib is mandatory:
without it the JIT cache re-resolves to a FlashInfer variant and segfaults on
sm_87 (Orin).

Publishes a latched `llm/ready` Bool once the /v1/models endpoint answers, so
consumers can wait for the model (a 35B MoE takes ~30 s to load) before entering
the conversation loop. All paths are ROS parameters so a different build/model
can be selected at launch.
"""
import os
import signal
import subprocess
import threading
import time
import urllib.request

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from std_msgs.msg import Bool


class MLCLLMServeNode(Node):
    def __init__(self):
        super().__init__('mlc_llm_serve_node')

        self.mlc_dir = self.declare_parameter('mlc_dir', '/home/alfie/mlc-llm').value
        self.model = self.declare_parameter(
            'model', 'dist/qwen3_6-35B-A3B-q4f16_1').value
        self.model_lib = self.declare_parameter(
            'model_lib', 'dist/qwen3_6-35B-A3B-q4f16_1/lib.so').value
        self.device = self.declare_parameter('device', 'cuda:0').value
        self.mode = self.declare_parameter('mode', 'interactive').value
        self.host = self.declare_parameter('host', '0.0.0.0').value
        self.port = int(self.declare_parameter('port', 8000).value)
        # Memory-footprint tuning: cap the KV-cache context, bound conversation
        # history, and hold the GPU memory pool to a fraction of VRAM.
        self.context_window_size = int(
            self.declare_parameter('context_window_size', 32768).value)
        self.max_history_size = int(
            self.declare_parameter('max_history_size', 4).value)
        self.gpu_memory_utilization = float(
            self.declare_parameter('gpu_memory_utilization', 0.5).value)

        # Latched so a consumer that subscribes after the model is up still sees it.
        latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.ready_pub = self.create_publisher(Bool, 'llm/ready', latched)
        self._publish_ready(False)

        self.proc = None
        self._stop = threading.Event()
        threading.Thread(target=self._run_server, daemon=True).start()
        threading.Thread(target=self._wait_ready, daemon=True).start()

    def _publish_ready(self, value):
        msg = Bool()
        msg.data = bool(value)
        self.ready_pub.publish(msg)

    def _serve_cmd(self):
        # Source .envrc.local (vendored TVM/mlc_llm env) then exec the server.
        return (
            f'cd {self.mlc_dir} && source .envrc.local && '
            f'exec .venv/bin/python -m mlc_llm serve {self.model} '
            f'--model-lib {self.model_lib} --device {self.device} '
            f'--mode {self.mode} --host {self.host} --port {self.port} '
            f'--overrides "context_window_size={self.context_window_size};'
            f'max_history_size={self.max_history_size};'
            f'gpu_memory_utilization={self.gpu_memory_utilization}"'
        )

    def _run_server(self):
        self.get_logger().info(
            f'Starting MLC-LLM server: {self.model} (lib {self.model_lib}, '
            f'{self.device}, mode {self.mode}, port {self.port})')
        try:
            # Own session/process group so we can tear down mlc's child processes.
            self.proc = subprocess.Popen(
                ['bash', '-c', self._serve_cmd()], start_new_session=True)
            self.proc.wait()
            if not self._stop.is_set():
                self.get_logger().error(
                    f'MLC-LLM server exited (code {self.proc.returncode}).')
                self._publish_ready(False)
        except Exception as e:
            self.get_logger().error(f'Failed to launch MLC-LLM server: {e}')

    def _wait_ready(self):
        url = f'http://localhost:{self.port}/v1/models'
        while not self._stop.is_set():
            try:
                with urllib.request.urlopen(url, timeout=2) as r:
                    if r.status == 200:
                        self.get_logger().info('MLC-LLM server is ready.')
                        self._publish_ready(True)
                        return
            except Exception:
                pass
            time.sleep(2.0)

    def destroy_node(self):
        self._stop.set()
        if self.proc and self.proc.poll() is None:
            try:
                pgid = os.getpgid(self.proc.pid)
                os.killpg(pgid, signal.SIGINT)
                for _ in range(20):
                    if self.proc.poll() is not None:
                        break
                    time.sleep(0.25)
                if self.proc.poll() is None:
                    os.killpg(pgid, signal.SIGKILL)
            except Exception:
                pass
        super().destroy_node()


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
