"""
Servo Configuration Tool entry point for Alfie Robot.

Usage:
    ros2 run alfie_tools servotool
"""

import sys
import threading
from PyQt5 import QtWidgets
import qdarktheme
import rclpy

from alfie_tools.servotool.gui.servotool_window import ServoToolWindow


def main(args=None):
    """Main entry point for the ServoTool application."""
    # Initialize Qt application
    app = QtWidgets.QApplication(sys.argv)
    
    # Apply dark theme
    palette = qdarktheme.load_palette(theme="dark")
    stylesheet = qdarktheme.load_stylesheet(theme="dark")
    app.setPalette(palette)
    app.setStyleSheet(stylesheet)
    
    # Initialize ROS2
    rclpy.init(args=args)
    node = rclpy.create_node("servotool")
    
    # Create window
    window = ServoToolWindow(node)
    window.show()
    
    # Spin ROS in background thread
    ros_thread = threading.Thread(
        target=lambda: rclpy.spin(node),
        daemon=True
    )
    ros_thread.start()
    
    # Run Qt event loop
    exit_code = app.exec()
    
    # Cleanup
    node.destroy_node()
    rclpy.shutdown()
    sys.exit(exit_code)


if __name__ == "__main__":
    main()
