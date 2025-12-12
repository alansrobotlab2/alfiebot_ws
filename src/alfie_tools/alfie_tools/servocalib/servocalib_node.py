"""
Servo Calibration TUI Tool - Terminal User Interface for Alfie Robot servo calibration.

Usage:
    ros2 run alfie_tools servocalib
    
This tool provides a TUI to calibrate servo position offsets for all 17 servos
(left arm 7, right arm 7, head 3). Navigate with Tab/Arrow keys, type values,
and they are written to servo EEPROM.
"""

import sys
import threading
import rclpy
from rclpy.executors import SingleThreadedExecutor
from alfie_tools.servocalib.app.servocalib_app import ServoCalibApp


def main(args=None):
    """Main entry point for the ServoCalib application."""
    # Initialize ROS2
    rclpy.init(args=args)
    node = rclpy.create_node("servocalib")
    
    # Create executor for background spinning
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    
    # Create TUI app
    app = ServoCalibApp(node)
    
    # Flag to signal shutdown
    shutdown_event = threading.Event()
    
    def spin_thread():
        """Spin ROS2 until shutdown is signaled."""
        while not shutdown_event.is_set():
            executor.spin_once(timeout_sec=0.1)
    
    # Spin ROS in background thread
    ros_thread = threading.Thread(target=spin_thread, daemon=True)
    ros_thread.start()
    
    # Launch TUI interface (blocking)
    try:
        app.run()
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"\nError: {e}")
    finally:
        # Signal shutdown and wait for thread
        print("\nShutting down ServoCalib...")
        shutdown_event.set()
        ros_thread.join(timeout=1.0)
        
        # Cleanup
        executor.shutdown()
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == "__main__":
    main()
