#!/usr/bin/env python3
"""
Alfie Back Node - Controls the shoulder linear actuator system.

This node manages the back shoulder actuators with 400mm travel range.
Position 0mm (0.0m) is at the bottom where the limit switch is located.
Maximum travel is 400mm (0.4m) at the top.
"""

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup, MutuallyExclusiveCallbackGroup
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
import time

from alfie_msgs.msg import BackCmd, BackState, GDBState
from alfie_msgs.srv import BackRequestCalibration
from alfie_back.mksservo42c_10 import MKSServo42C


class AlfieBackNode(Node):
    """Node to control Alfie's back shoulder linear actuators."""

    def __init__(self):
        super().__init__('alfie_back_node')
        
        # Callback groups for concurrent execution
        # Reentrant group allows subscription callbacks to run while service is executing
        self.reentrant_group = ReentrantCallbackGroup()
        # Service runs in its own group
        self.service_group = MutuallyExclusiveCallbackGroup()
        
        # State variables
        self.shoulder_limit_switch = False  # True when at bottom (0mm) position
        self.last_gdb1_msg_time = None  # Timestamp of last GDB1 state message
        self.calibrated = False  # True when calibration has been completed
        self.pulses_offset = 0  # Offset for encoder pulses (set during calibration)
        
        # Constants for motor control
        self.MAX_DOWN_SPEED = 0x78
        self.MAX_UP_SPEED = 0x7D
        
        # Constants for position calculations
        self.SUBDIVISION = 8
        self.PULLEYTEETH = 20
        self.GT2_PITCH = 2.0  # mm
        self.STEPPER_DEGREES = 1.8  # degrees
        self.STEPS_PER_MM = int(((360.0/self.STEPPER_DEGREES) * self.SUBDIVISION) / 
                                (self.GT2_PITCH * self.PULLEYTEETH))
        self.MAX_HEIGHT = 400  # mm
        
        # Initialize MKS Servo 42C motor driver
        # Connected to /dev/ttyTHS1 at 115200 baud
        try:
            self.servo = MKSServo42C(
                port='/dev/ttyTHS1',
                baudrate=115200,
                timeout=1.0,
                address=0xE0
            )
            self.get_logger().info('MKS Servo 42C initialized on /dev/ttyTHS1 at 115200 baud')
            
            # Test communication by reading motor shaft angle
            angle = self.servo.read_motor_shaft_angle()
            self.get_logger().info(f'Successfully read motor shaft angle: {angle}')
            self.get_logger().info('MKS Servo 42C communication verified')
            
        except Exception as e:
            self.get_logger().error(f'Failed to initialize MKS Servo 42C: {e}')
            raise
        
        # QoS profile for GDB1 state subscription (best effort to match publisher)
        gdb1_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        # Subscribe to GDB1 state for switch information
        self.gdb1_state_sub = self.create_subscription(
            GDBState,
            '/alfie/gdb1state',
            self.gdb1_state_callback,
            gdb1_qos,
            callback_group=self.reentrant_group
        )
        
        # Publisher for back shoulder state at 100Hz
        self.back_state_pub = self.create_publisher(
            BackState,
            'back_state',
            10
        )
        
        # Timer for publishing state at 100Hz (0.01 seconds)
        self.state_timer = self.create_timer(0.01, self.publish_state)
        
        # Service for calibration requests
        self.calibration_service = self.create_service(
            BackRequestCalibration,
            'back_request_calibration',
            self.calibration_callback,
            callback_group=self.service_group
        )
        
        self.get_logger().info('Alfie Back Node initialized')
        self.get_logger().info('Monitoring shoulder limit switch state')
        self.get_logger().info('Publishing back state at 100Hz')
        self.get_logger().info('Calibration service ready')

    def gdb1_state_callback(self, msg: GDBState):
        """
        Callback for GDB1 state updates.
        
        Stores the shoulder limit switch state which indicates when
        the shoulders are at the lowest position (0mm).
        
        Args:
            msg: GDBState message containing switch states
        """
        # Update timestamp of last received message
        self.last_gdb1_msg_time = self.get_clock().now()
        
        self.shoulder_limit_switch = msg.shoulder_limit_state
        
        # Log when switch state changes (for debugging)
        if hasattr(self, '_last_switch_state'):
            if self._last_switch_state != self.shoulder_limit_switch:
                if self.shoulder_limit_switch:
                    self.get_logger().info('Shoulder at bottom limit (0mm)')
                else:
                    self.get_logger().info('Shoulder moved away from bottom limit')
        
        self._last_switch_state = self.shoulder_limit_switch

    def get_height(self) -> float:
        """
        Get the height of the shoulder in mm.
        
        Returns:
            The height of the shoulder in mm.
        """
        # Get pulses
        pulses = self.servo.read_pulses_received()
        # Subtract offset
        pulses -= self.pulses_offset
        # Convert to mm
        height = pulses / self.STEPS_PER_MM
        return height

    def moveto_height(self, target_height: float, speed: int) -> bool:
        """
        Move the shoulder to the specified height using continuous movement.
        Monitors actual position and stops when target is reached.
        
        Args:
            target_height: The height to move to in mm (0-400)
            speed: Movement speed
            
        Returns:
            True if successful, False if out of range
        """
        if target_height < 0 or target_height > self.MAX_HEIGHT:
            self.get_logger().error(f'Height {target_height}mm out of range (0-{self.MAX_HEIGHT})')
            return False
        
        # Get current height
        current_height = self.get_height()
        # Calculate height difference
        height_diff = target_height - current_height
        
        self.get_logger().info(f'Moving from {current_height:.1f}mm to {target_height}mm')
        
        # Check if height difference is positive or negative
        if height_diff > 0:
            # Move up
            direction = "CW"
            speed = min(speed, self.MAX_UP_SPEED)
            
            # Move until height reaches target height
            while current_height <= target_height:
                self.servo.move(direction, speed)
                time.sleep(0.01)
                current_height = self.get_height()
                
            self.servo.stop()
            
        else:
            # Move down
            direction = "CCW"
            speed = min(speed, self.MAX_DOWN_SPEED)
            
            # Move until height reaches target height
            while current_height >= target_height:
                self.servo.move(direction, speed)
                time.sleep(0.01)
                current_height = self.get_height()
                
            self.servo.stop()
        
        final_height = self.get_height()
        self.get_logger().info(f'Reached {final_height:.1f}mm')
        
        return True

    def publish_state(self):
        """
        Publish the current back shoulder state at 100Hz.
        
        Populates BackState message with current state information.
        Currently most values are zeros except for the limit switch state.
        """
        state_msg = BackState()
        
        # Populate calibration status
        state_msg.calibrated = self.calibrated
        
        # Populate with current switch state
        state_msg.at_bottom_limit = self.shoulder_limit_switch
        
        # Initialize other fields to default values
        state_msg.enabled = False
        state_msg.is_moving = False
        state_msg.position = 0.0
        state_msg.velocity = 0.0
        state_msg.acceleration = 0.0
        state_msg.at_top_limit = False
        
        self.back_state_pub.publish(state_msg)

    def calibration_callback(self, request, response):
        """
        Handle calibration request service calls.
        
        Implements the shoulder calibration routine similar to initshoulder.py.
        Moves the shoulders to the limit switch position and sets the zero reference.
        
        Args:
            request: BackRequestCalibration request (empty)
            response: BackRequestCalibration response
            
        Returns:
            response with success field
        """
        self.get_logger().info('Calibration request received')
        
        # Check if we have received a GDB1 state message recently (within 100ms)
        if self.last_gdb1_msg_time is None:
            self.get_logger().error('Calibration failed: No GDB1 state messages received yet')
            response.success = False
            return response
        
        current_time = self.get_clock().now()
        time_since_last_msg = (current_time - self.last_gdb1_msg_time).nanoseconds / 1e6  # Convert to ms
        
        if time_since_last_msg > 100.0:
            self.get_logger().error(f'Calibration failed: Last GDB1 message was {time_since_last_msg:.1f}ms ago (max 100ms)')
            response.success = False
            return response
        
        # Begin calibration routine
        self.get_logger().info('Starting calibration routine...')
        
        # Check current limit switch state
        if self.shoulder_limit_switch:
            self.get_logger().info('Limit switch triggered - moving up to clear it')
            # Limit switch is triggered, move up until it clears
            self.servo.move("CW", self.MAX_UP_SPEED)
            
            while True:
                # Check for fresh GDB1 messages
                current_time = self.get_clock().now()
                time_since_last_msg = (current_time - self.last_gdb1_msg_time).nanoseconds / 1e6

                if time_since_last_msg > 100.0:
                    self.get_logger().error(f'Calibration failed: GDB1 message stale ({time_since_last_msg:.1f}ms > 100ms)')
                    self.servo.stop()
                    response.success = False
                    return response
                
                # Check if limit switch has cleared
                if not self.shoulder_limit_switch:
                    break
                    
                time.sleep(0.01)
                
            self.servo.stop()
            self.get_logger().info('Limit switch cleared')
        else:
            self.get_logger().info('Limit switch not triggered - moving down to find it')
            # Limit switch not triggered, move down to find it
            self.servo.move("CCW", self.MAX_DOWN_SPEED)
            
            while True:
                # Check for fresh GDB1 messages
                current_time = self.get_clock().now()
                time_since_last_msg = (current_time - self.last_gdb1_msg_time).nanoseconds / 1e6

                if time_since_last_msg > 100.0:
                    self.get_logger().error(f'Calibration failed: GDB1 message stale ({time_since_last_msg:.1f}ms > 100ms)')
                    self.servo.stop()
                    response.success = False
                    return response
                
                # Check if limit switch has been triggered
                if self.shoulder_limit_switch:
                    break
                    
                time.sleep(0.01)
                
            self.servo.stop()
            self.get_logger().info('Limit switch found')
        
        # Set the pulses offset to current position (this is our zero reference)
        self.pulses_offset = self.servo.read_pulses_received()
        self.get_logger().info(f'Calibration zero position set - pulses offset: {self.pulses_offset}')
        
        # Mark as calibrated
        self.calibrated = True
        
        # Perform test movements: 380mm -> 400mm -> 380mm
        self.get_logger().info('Starting calibration test movements...')
        
        # Move to 390mm
        self.get_logger().info('Moving to 390mm')
        self.moveto_height(390, self.MAX_UP_SPEED)
        time.sleep(0.5)
        
        # Move to 395 (shrug)
        self.get_logger().info('Moving to 395 (max)')
        self.moveto_height(395, self.MAX_DOWN_SPEED)
        current_height = self.get_height()
        self.get_logger().info(f'At max height: {current_height:.1f}mm')
        time.sleep(0.25)

        # Move back to 390mm
        self.get_logger().info('Moving back to 390mm')
        self.moveto_height(390, self.MAX_DOWN_SPEED)
        final_height = self.get_height()
        self.get_logger().info(f'Final height: {final_height:.1f}mm')
        
        self.get_logger().info('Calibration and test movements complete!')
        
        response.success = True
        return response


def main(args=None):
    """Main entry point for the node."""
    rclpy.init(args=args)
    
    node = AlfieBackNode()
    
    # Use MultiThreadedExecutor to allow callbacks to run concurrently
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        # Clean up servo connection
        if hasattr(node, 'servo'):
            node.servo.disconnect()
            node.get_logger().info('MKS Servo 42C disconnected')
        node.destroy_node()
        executor.shutdown()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
