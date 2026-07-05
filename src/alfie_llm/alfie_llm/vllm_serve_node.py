"""vllm_serve_node — ROS2 wrapper that runs Qwen3.6-35B-A3B-AWQ-4bit behind
the Jetson-native vLLM container and exposes an OpenAI-compatible endpoint.

Model is fetched from HF on first start (~20 GB), then served offline.
Follows the same attached-container + `--rm` pattern as mlc_llm_serve_node
so the container lifecycle is tied to the ROS2 node — no `--restart`
policy, no lingering container on node exit.
"""

import signal
import subprocess
import threading
import time
from pathlib import Path

import rclpy
from rclpy.node import Node


class VllmServeNode(Node):
    def __init__(self):
        super().__init__('vllm_serve_node')

        self.declare_parameter('model_repo', 'cyankiwi/Qwen3.6-35B-A3B-AWQ-4bit')
        self.declare_parameter('model_dir', str(Path.home() / 'models' / 'Qwen3.6-35B-A3B-AWQ-4bit'))
        self.declare_parameter('log_dir', str(Path.home() / 'vllm-logs'))
        self.declare_parameter('image', 'ghcr.io/nvidia-ai-iot/vllm:r36.4-tegra-aarch64-cu126-22.04')
        self.declare_parameter(
            'wheel_url',
            'https://huggingface.co/thehighnotes/vllm-jetson-orin/resolve/main/vllm-0.17.0%2Bcu126-cp310-cp310-linux_aarch64.whl',
        )
        self.declare_parameter('container_name', 'vllm-qwen')
        self.declare_parameter('served_model_name', 'qwen3.6-35b-a3b')
        self.declare_parameter('port', 8000)
        self.declare_parameter('api_key', '')
        self.declare_parameter('max_model_len', 262144)
        # empty = let vLLM auto-detect from model config.json.
        # cyankiwi/Qwen3.6-35B-A3B-AWQ-4bit is packaged as compressed-tensors
        # (not awq), so forcing --quantization here causes a validation error.
        self.declare_parameter('quantization', '')
        self.declare_parameter('gpu_memory_utilization', 0.88)
        self.declare_parameter('max_num_seqs', 2)
        # Must be >= the attention block size vLLM picks. For hybrid Mamba/Attention
        # models (Qwen3.6-35B-A3B has linear_attn layers), this is driven by the
        # mamba page size — on this model vLLM lands on 2096, so 2048 is too small.
        self.declare_parameter('max_num_batched_tokens', 4096)
        self.declare_parameter('swap_space_gb', 4)

        self.process = None
        self.container_name = self.get_parameter('container_name').value

        self.get_logger().info('Starting vllm server node...')
        self.thread = threading.Thread(target=self._run, daemon=True)
        self.thread.start()

    def _p(self, name):
        return self.get_parameter(name).value

    def _ensure_model(self) -> Path:
        model_dir = Path(self._p('model_dir')).expanduser()
        repo = self._p('model_repo')
        if (model_dir / 'config.json').exists():
            self.get_logger().info(f'model already present at {model_dir}')
            return model_dir
        self.get_logger().info(f'downloading {repo} -> {model_dir} (~20 GB, first run only)')
        model_dir.mkdir(parents=True, exist_ok=True)
        subprocess.check_call([
            'huggingface-cli', 'download', repo,
            '--local-dir', str(model_dir),
        ])
        return model_dir

    def _stop_existing_container(self):
        subprocess.run(
            ['docker', 'rm', '-f', self.container_name],
            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
        )

    def _run(self):
        try:
            model_dir = self._ensure_model()
        except Exception as e:
            self.get_logger().error(f'model download failed: {e}')
            return

        log_dir = Path(self._p('log_dir')).expanduser()
        log_dir.mkdir(parents=True, exist_ok=True)

        api_key = self._p('api_key')
        if not api_key:
            api_key = f'sk-vllm-{int(time.time())}'
            self.get_logger().warn(f'api_key param empty; generated: {api_key}')

        self._stop_existing_container()

        port = self._p('port')
        # The Jetson-patched wheel is not baked into the image, so we reinstall
        # it on every container start. First run populates pip cache; subsequent
        # starts are fast but still need network to HF.
        inner = (
            'source /opt/venv/bin/activate && '
            f'pip install --quiet --force-reinstall --no-deps {self._p("wheel_url")} && '
            'vllm serve /models/qwen '
            '--host 0.0.0.0 '
            f'--port {port} '
            f'--served-model-name {self._p("served_model_name")} '
            + (f'--quantization {self._p("quantization")} ' if self._p('quantization') else '')
            + f'--max-model-len {self._p("max_model_len")} '
            f'--gpu-memory-utilization {self._p("gpu_memory_utilization")} '
            '--kv-cache-dtype fp8 '
            '--calculate-kv-scales '
            '--enable-prefix-caching '
            '--enable-chunked-prefill '
            f'--max-num-batched-tokens {self._p("max_num_batched_tokens")} '
            f'--max-num-seqs {self._p("max_num_seqs")} '
            '--enable-auto-tool-choice '
            '--tool-call-parser hermes '
            '--reasoning-parser qwen3 '
            f'--swap-space {self._p("swap_space_gb")} '
            '--api-key $VLLM_API_KEY '
            '--uvicorn-log-level info 2>&1 | tee /var/log/vllm/server.log'
        )

        cmd = [
            'docker', 'run',
            '--runtime', 'nvidia',
            '--network', 'host',
            '--ipc=host',
            '--ulimit', 'memlock=-1',
            '--ulimit', 'stack=67108864',
            '--shm-size=8g',
            '--rm',
            '-v', f'{model_dir}:/models/qwen:ro',
            '-v', f'{Path.home() / ".cache" / "huggingface"}:/root/.cache/huggingface',
            '-v', f'{log_dir}:/var/log/vllm',
            '-e', 'NVIDIA_VISIBLE_DEVICES=all',
            '-e', 'NVIDIA_DRIVER_CAPABILITIES=compute,utility',
            '-e', 'HF_HUB_OFFLINE=1',
            '-e', f'VLLM_API_KEY={api_key}',
            '--name', self.container_name,
            self._p('image'),
            'bash', '-c', inner,
        ]

        self.get_logger().info(f'launching container {self.container_name} on port {port}')
        try:
            self.process = subprocess.Popen(cmd)
            self.process.wait()
        except Exception as e:
            self.get_logger().error(f'Failed to launch vllm container: {e}')

    def destroy_node(self):
        self.get_logger().info(f'stopping container {self.container_name}...')
        subprocess.run(
            ['docker', 'stop', self.container_name],
            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, timeout=30,
        )
        if self.process and self.process.poll() is None:
            self.process.send_signal(signal.SIGTERM)
            try:
                self.process.wait(timeout=10)
            except subprocess.TimeoutExpired:
                self.process.kill()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = VllmServeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('KeyboardInterrupt received, shutting down...')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
