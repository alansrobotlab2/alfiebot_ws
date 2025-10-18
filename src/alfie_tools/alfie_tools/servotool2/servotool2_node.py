"""
Servo Configuration Tool 2 - Web Interface entry point for Alfie Robot.

Usage:
    ros2 run alfie_tools servotool2
    
Then open browser to: http://localhost:7860
"""

import sys
import threading
import rclpy
from alfie_tools.servotool2.app.servotool_app import ServoToolApp


def main(args=None):
    """Main entry point for the ServoTool2 application."""
    # Initialize ROS2
    rclpy.init(args=args)
    node = rclpy.create_node("servotool2")
    
    # Create and launch web app
    app = ServoToolApp(node)
    
    # Spin ROS in background thread
    ros_thread = threading.Thread(
        target=lambda: rclpy.spin(node),
        daemon=True
    )
    ros_thread.start()
    
    # Launch Gradio interface (blocking)
    try:
        app.launch()
    except KeyboardInterrupt:
        print("\nShutting down ServoTool2...")
    finally:
        # Cleanup
        node.destroy_node()
        rclpy.shutdown()
        sys.exit(0)


if __name__ == "__main__":
    main()
