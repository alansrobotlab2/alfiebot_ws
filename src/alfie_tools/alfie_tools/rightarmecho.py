#!/usr/bin/env python3
"""
Right Arm Echo Node

This node reads the positions of the right arm servos from robotlowstate
and writes them back as an ArmCmd on the command_mux input cmd/right_arm/hold
(low priority), effectively echoing the current positions as target positions.

Author: Alan's Robot Lab
License: Apache-2.0
"""

import rclpy
from rclpy.node import Node
from alfie_msgs.msg import RobotLowState, ArmCmd, ServoCmd
from rclpy.qos import QoSProfile, ReliabilityPolicy


# ============================================================================
# Constants and Configuration
# ============================================================================

# Publishing configuration
PUBLISH_RATE_HZ = 100
PUBLISH_PERIOD_SEC = 1.0 / PUBLISH_RATE_HZ  # 0.01 seconds

# Servo indices for right arm (zero-indexed)
RIGHT_ARM_START_INDEX = 6
RIGHT_ARM_END_INDEX = 10  # inclusive
RIGHT_ARM_SERVO_COUNT = RIGHT_ARM_END_INDEX - RIGHT_ARM_START_INDEX + 1  # 6 servos

# Servo indices for left arm (zero-indexed)
LEFT_ARM_START_INDEX = 0
LEFT_ARM_END_INDEX = 5  # inclusive
LEFT_ARM_SERVO_COUNT = LEFT_ARM_END_INDEX - LEFT_ARM_START_INDEX + 1  # 6 servos

# Total number of servos in the system
TOTAL_SERVOS = 15


# ============================================================================
# Right Arm Echo Node Class
# ============================================================================

class RightArmEchoNode(Node):
    def __init__(self):
        super().__init__('right_arm_echo_node')
        
        # QoS profiles - use best effort for all communication
        qos_best_effort = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        
        # Subscribe to robot low state (use BEST_EFFORT to match publisher)
        self.state_sub = self.create_subscription(
            RobotLowState,
            '/alfie/robotlowstate',
            self.state_callback,
            qos_best_effort
        )
        
        # Publisher for the right-arm hold source into command_mux (BEST_EFFORT).
        self.cmd_pub = self.create_publisher(
            ArmCmd,
            '/alfie/cmd/right_arm/hold',
            qos_best_effort
        )
        
        # Initialize state storage
        self.latest_state = None
        
        # Create timer for 100Hz publishing
        self.timer = self.create_timer(PUBLISH_PERIOD_SEC, self.publish_command)
        
        self.get_logger().info(f'Right Arm Echo Node started - publishing at {PUBLISH_RATE_HZ}Hz')
        self.get_logger().info(f'Echoing right arm servos {RIGHT_ARM_START_INDEX}-{RIGHT_ARM_END_INDEX}')
    
    # ========================================================================
    # Callback Methods
    # ========================================================================
    
    def state_callback(self, msg: RobotLowState) -> None:
        """Callback for robotlowstate - store the latest state"""
        self.latest_state = msg
    
    # ========================================================================
    # Publishing Methods
    # ========================================================================
    
    def publish_command(self) -> None:
        """Publish an ArmCmd holding the right arm at its current measured pose.

        The right arm occupies indices 6-11 in RobotLowState.servo_state; ArmCmd
        carries 6 logical joints (0-5). This is a low-priority "hold" source in
        the mux, so any real right-arm commander (VR, GR00T) outranks it.
        """
        if self.latest_state is None:
            self.get_logger().warn('No robot state received yet', throttle_duration_sec=1.0)
            return

        cmd = ArmCmd()
        cmd.joint_cmd = []
        for i in range(RIGHT_ARM_SERVO_COUNT):
            servo = ServoCmd()
            servo.enabled = True
            servo.target_location = self.latest_state.servo_state[RIGHT_ARM_START_INDEX + i].current_location
            servo.target_speed = 0.0
            servo.target_acceleration = 0.0
            servo.target_torque = 0.0
            cmd.joint_cmd.append(servo)

        self.cmd_pub.publish(cmd)


# ============================================================================
# Main Entry Point
# ============================================================================

def main(args=None):
    rclpy.init(args=args)
    node = RightArmEchoNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
