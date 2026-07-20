import threading

import rclpy
from rclpy.node import Node

from alfie_llm.proc_util import (
    spawn_supervised, terminate_group, install_sigterm_shutdown)


class OllamaServeNode(Node):
    def __init__(self):
        super().__init__('ollama_serve_node')
        self.get_logger().info('Starting ollama server...')
        self.proc = None
        self._stop = threading.Event()
        self.ollama_thread = threading.Thread(target=self.run_ollama, daemon=True)
        self.ollama_thread.start()

    def run_ollama(self):
        try:
            # Own session + PR_SET_PDEATHSIG so `ollama serve` is torn down with
            # this node, even on SIGKILL/crash (see proc_util).
            self.proc = spawn_supervised(['ollama', 'serve'])
            self.proc.wait()
            if not self._stop.is_set():
                self.get_logger().error(
                    f'ollama server exited (code {self.proc.returncode}).')
        except Exception as e:
            self.get_logger().error(f'Failed to launch ollama server: {e}')

    def destroy_node(self):
        self._stop.set()
        terminate_group(self.proc, grace=5.0, logger=self.get_logger())
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    # Route SIGTERM through the KeyboardInterrupt path so `destroy_node()` runs
    # and the ollama server is not orphaned.
    install_sigterm_shutdown()
    node = OllamaServeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
