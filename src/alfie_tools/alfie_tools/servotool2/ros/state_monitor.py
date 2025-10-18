"""ROS2 state subscriber for real-time servo monitoring."""

from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from alfie_msgs.msg import GDBState


class ServoStateMonitor:
    """Monitors servo state updates from driver topics."""
    
    def __init__(self, node: Node):
        """Initialize state monitor with subscriptions.
        
        Args:
            node: ROS2 node instance
        """
        self.node = node
        self.driver0_state = None
        self.driver1_state = None
        
        # Setup QoS for best-effort real-time updates
        qos = QoSProfile(depth=10)
        qos.reliability = ReliabilityPolicy.BEST_EFFORT
        
        # Subscribe to both driver state topics
        self.driver0_sub = node.create_subscription(
            GDBState,
            "/alfie/gdb0state",
            self._gdb0_callback,
            qos
        )
        
        self.driver1_sub = node.create_subscription(
            GDBState,
            "/alfie/gdb1state",
            self._gdb1_callback,
            qos
        )
        
        self.node.get_logger().info("Servo state monitor initialized")
    
    def _gdb0_callback(self, msg: GDBState):
        """Callback for GDB0 state updates."""
        self.driver0_state = msg
    
    def _gdb1_callback(self, msg: GDBState):
        """Callback for GDB1 state updates."""
        self.driver1_state = msg
    
    def get_servo_state(self, bus: int, servo_id: int):
        """Get current state for specific servo.
        
        Args:
            bus: Servo bus number (0 or 1)
            servo_id: Servo ID (1-10)
            
        Returns:
            Dictionary with servo state fields or None if not available
        """
        # Get appropriate driver state
        driver_state = self.driver0_state if bus == 0 else self.driver1_state
        
        if driver_state is None:
            return None
        
        # Servo ID is 1-indexed, array is 0-indexed
        servo_index = servo_id - 1
        
        if servo_index >= len(driver_state.servo_state):
            return None
        
        servo = driver_state.servo_state[servo_index]
        
        # Return formatted state dictionary
        return {
            'torque_switch': servo.torqueswitch,
            'target_location': servo.targetlocation,
            'acceleration': servo.acceleration,
            'current_location': servo.currentlocation,
            'current_speed': servo.currentspeed,
            'current_voltage': servo.currentvoltage / 10.0,  # Convert to actual voltage
            'current_current': servo.currentcurrent * 6.5,    # Convert to mA
            'current_temperature': servo.currenttemperature,
            'servo_status': servo.servostatus,
            'mobile_sign': servo.mobilesign,
        }
    
    def is_data_available(self, bus: int) -> bool:
        """Check if data is available for given bus.
        
        Args:
            bus: Servo bus number (0 or 1)
            
        Returns:
            True if state data is available
        """
        driver_state = self.driver0_state if bus == 0 else self.driver1_state
        return driver_state is not None
