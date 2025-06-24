import rclpy
from rclpy.node import Node
import subprocess
import threading

class OllamaServeNode(Node):
    def __init__(self):
        super().__init__('ollama_serve_node')
        self.get_logger().info('Starting ollama serve...')
        self.ollama_thread = threading.Thread(target=self.run_ollama, daemon=True)
        self.ollama_thread.start()

    def run_ollama(self):
        try:
            process = subprocess.Popen(['ollama', 'serve'])
            process.wait()
        except Exception as e:
            self.get_logger().error(f'Failed to launch ollama serve: {e}')

def main(args=None):
    rclpy.init(args=args)
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
