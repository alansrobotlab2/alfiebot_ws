#!/usr/bin/env python3
"""GR00T N1.6 inference server ROS2 node wrapper.

This node wraps the standalone GR00T inference server, providing ROS2
parameter integration and status publishing.

The actual inference logic is in alfie_gr00t.scripts.groot_inference_server,
which can be run standalone without ROS2.
"""

import logging

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from alfie_gr00t.scripts.groot_inference_server import GrootInferenceServer


class ROS2LogHandler(logging.Handler):
    """Logging handler that forwards to ROS2 logger."""

    def __init__(self, ros_logger):
        super().__init__()
        self._ros_logger = ros_logger

    def emit(self, record):
        msg = self.format(record)
        if record.levelno >= logging.ERROR:
            self._ros_logger.error(msg)
        elif record.levelno >= logging.WARNING:
            self._ros_logger.warn(msg)
        elif record.levelno >= logging.INFO:
            self._ros_logger.info(msg)
        else:
            self._ros_logger.debug(msg)


class GrootServerNode(Node):
    """GR00T N1.6 inference server ROS2 node.

    This is a thin wrapper around GrootInferenceServer that:
    - Exposes configuration via ROS2 parameters
    - Publishes server status to a ROS2 topic
    - Integrates with ROS2 lifecycle (spin/shutdown)

    The actual ZMQ server and inference logic is in the standalone
    GrootInferenceServer class.
    """

    def __init__(self):
        super().__init__('groot_server')

        # Declare parameters
        self._declare_parameters()

        # Get parameters
        transport = self.get_parameter('transport').value
        bind_host = self.get_parameter('bind_host').value
        bind_port = self.get_parameter('bind_port').value
        ipc_path = self.get_parameter('ipc_path').value
        model_checkpoint = self.get_parameter('model_checkpoint').value
        embodiment_tag = self.get_parameter('embodiment_tag').value
        mock_mode = self.get_parameter('mock_mode').value
        action_horizon = self.get_parameter('action_horizon').value
        device = self.get_parameter('device').value
        dataset_path = self.get_parameter('dataset_path').value
        episode_index = self.get_parameter('episode_index').value
        stats_path = self.get_parameter('stats_path').value

        # Setup logger that forwards to ROS2
        logger = logging.getLogger('groot_inference_server')
        logger.setLevel(logging.DEBUG)
        logger.handlers.clear()
        logger.addHandler(ROS2LogHandler(self.get_logger()))

        # Create the inference server
        self._server = GrootInferenceServer(
            transport=transport,
            bind_host=bind_host,
            bind_port=bind_port,
            ipc_path=ipc_path,
            model_checkpoint=model_checkpoint,
            embodiment_tag=embodiment_tag,
            mock_mode=mock_mode,
            action_horizon=action_horizon,
            device=device,
            logger=logger,
            dataset_path=dataset_path,
            episode_index=episode_index,
            stats_path=stats_path,
        )

        # Start the server
        self._server.start()

        # Status publisher
        self.status_pub = self.create_publisher(String, '~/status', 10)
        self.status_timer = self.create_timer(1.0, self._publish_status)

        # Server loop timer - process requests in ROS2 spin
        self.create_timer(0.001, self._server_loop)

    def _declare_parameters(self):
        """Declare all ROS2 parameters."""
        # Transport configuration
        self.declare_parameter('transport', 'ipc')
        self.declare_parameter('bind_host', '*')  # Bind to all interfaces
        self.declare_parameter('bind_port', 5555)
        self.declare_parameter('ipc_path', '/tmp/groot_inference.sock')

        # Model configuration
        self.declare_parameter('model_checkpoint', '')
        self.declare_parameter('embodiment_tag', 'new_embodiment')
        self.declare_parameter('mock_mode', False)
        self.declare_parameter('action_horizon', 16)
        self.declare_parameter('device', 'cuda:0')

        # Replay configuration
        self.declare_parameter('dataset_path', '')
        self.declare_parameter('episode_index', 0)
        self.declare_parameter('stats_path', '')

    def _server_loop(self):
        """Process server requests."""
        self._server.spin_once()

    def _publish_status(self):
        """Publish server status."""
        stats = self._server.get_stats()
        msg = String()
        msg.data = str(stats)
        self.status_pub.publish(msg)

    def destroy_node(self):
        """Clean up resources on shutdown."""
        self._server.stop()
        super().destroy_node()


def main(args=None):
    """Main entry point."""
    rclpy.init(args=args)

    node = GrootServerNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
