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
        # Memory-footprint tuning: cap the KV-cache context and hold the GPU memory
        # pool to a fraction of VRAM.
        self.context_window_size = int(
            self.declare_parameter('context_window_size', 32768).value)
        # `max_history_size` is NOT conversation history — in this hybrid (GDN)
        # build it sizes the rnn_state recurrent-history ring buffer, which is what
        # bounds prefix-cache reuse. It is MEMORY-EXPENSIVE: the rnn_state slab is
        # ~0.3 GB per slot for this 35B model (16 -> ~5 GB, 32 -> ~9.8 GB, 256 OOMs
        # at util 0.5). 16 comfortably covers the warm parent's ~13-token divergent
        # tail (see agent_node._warm_prefix); larger buys nothing here because real
        # conversation tails already exceed it and fall back to full prefill. See
        # the prefix-caching note below.
        self.max_history_size = int(
            self.declare_parameter('max_history_size', 16).value)
        self.gpu_memory_utilization = float(
            self.declare_parameter('gpu_memory_utilization', 0.5).value)
        # NOTE on prefix caching (re-investigated 2026-07-13): first turns used to
        # re-prefill the whole ~800-token system prompt (~2.1s). Root cause: Qwen3.6
        # 35B-A3B is a HYBRID (attention + GDN linear-attention) model, so reusing an
        # interior prefix (the shared system prompt) requires rolling the recurrent
        # rnn_state back by `pop_n = parent_len - match_offset` tokens, and that fork
        # is only allowed when `pop_n <= max_history_size` (mlc cpp/serve/prefix_cache.cc
        # ~L138). With the old max_history_size=4 the fork was always skipped -> full
        # re-prefill. Raising it to 16 lets a real first turn FORK from a short-tailed
        # "warm" parent (see agent_node._warm_prefix, which keeps [episode_prompt,'hi',
        # <1tok>] resident); measured first-turn TTFT then drops ~2.1s -> ~0.3-0.6s.
        # In-conversation follow-ups after a non-tool turn are fast (~0.3s) via pop_n=0
        # continuation reuse. CAVEAT: --mode interactive keeps only ONE prefix-cache
        # slot, so each turn evicts the warm parent (the agent re-warms after every
        # turn and on idle) and a tool turn breaks the next turn's continuation (the
        # cache holds raw <tool_call> text, history holds the clean reply). The robust
        # fix (pin the constant system prefix + snapshot its recurrent state so forks
        # need pop_n=0, no rnn history buffer) is engine-side; see
        # docs/mlc_prefix_cache_problem_statement.md.
        # Do NOT pass prefix_cache_max_num_recycling_seqs=-1 ("infinite"): the KV cache
        # sizes `max_num_sequence + N` slots (mlc cpp/serve/model.cc), so -1 -> 0 slots
        # and the native engine reload deadlocks (server never binds :8000).

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
