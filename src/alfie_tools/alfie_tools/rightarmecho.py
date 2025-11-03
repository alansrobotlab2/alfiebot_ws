#!/usr/bin/env python3
"""
Right Arm Echo Node

This node reads the positions of the right arm servos from robotlowstate
and writes them back to the right arm in robotlowcmd, effectively echoing
the current positions as target positions.

Author: Alan's Robot Lab
License: Apache-2.0
"""

import rclpy
from rclpy.node import Node
from alfie_msgs.msg import RobotLowState, RobotLowCmd, ServoCmd
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
        
        # QoS profiles
        qos_reliable = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        qos_best_effort = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        
        # Subscribe to robot low state (use BEST_EFFORT to match publisher)
        self.state_sub = self.create_subscription(
            RobotLowState,
            '/alfie/robotlowstate',
            self.state_callback,
            qos_best_effort
        )
        
        # Publisher for robot low command (use RELIABLE)
        self.cmd_pub = self.create_publisher(
            RobotLowCmd,
            '/alfie/robotlowcmd',
            qos_reliable
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
        """Publish command with right arm positions from latest state"""
        if self.latest_state is None:
            self.get_logger().warn('No robot state received yet', throttle_duration_sec=1.0)
            return
        
        # Create command message
        cmd = RobotLowCmd()
        
        # Initialize all servo commands (15 servos total)
        for i in range(TOTAL_SERVOS):
            servo_cmd = ServoCmd()
            servo_cmd.enabled = False
            servo_cmd.acceleration = 0.0
            servo_cmd.target_location = 0.0
            cmd.servo_cmd.append(servo_cmd)

        cmd.servo_cmd[6].enabled = True
        cmd.servo_cmd[6].target_location = self.latest_state.servo_state[0].current_location

        cmd.servo_cmd[7].enabled = True
        cmd.servo_cmd[7].target_location = self.latest_state.servo_state[1].current_location

        cmd.servo_cmd[8].enabled = True
        cmd.servo_cmd[8].target_location = self.latest_state.servo_state[2].current_location

        cmd.servo_cmd[9].enabled = True
        cmd.servo_cmd[9].target_location = self.latest_state.servo_state[3].current_location

        cmd.servo_cmd[10].enabled = True
        cmd.servo_cmd[10].target_location = self.latest_state.servo_state[4].current_location

        cmd.servo_cmd[11].enabled = True
        cmd.servo_cmd[11].target_location = self.latest_state.servo_state[5].current_location


        # # Copy left arm positions from state to right arm positions in command
        # for i in range(LEFT_ARM_START_INDEX, LEFT_ARM_END_INDEX + 1):
        #     if self.latest_state and i < len(self.latest_state.servo_state):
        #         state_servo = self.latest_state.servo_state[i]
                
        #         # Enable servo and set target to current position
        #         if state_servo.current_location != 0.0:
        #             cmd.servo_cmd[(i + RIGHT_ARM_START_INDEX)].enabled = False
        #             cmd.servo_cmd[(i + RIGHT_ARM_START_INDEX)].target_location = state_servo.current_location

        #         self.get_logger().debug(
        #             f'Servo {i}: enabled={state_servo.enabled}, '
        #             f'target={state_servo.current_location:.3f} rad',
        #             throttle_duration_sec=1.0
        #         )
        
        # Set other fields to safe defaults
        cmd.eye_pwm = [0, 0]
        cmd.shoulder_height = 0.0
        cmd.cmd_vel.linear.x = 0.0
        cmd.cmd_vel.linear.y = 0.0
        cmd.cmd_vel.linear.z = 0.0
        cmd.cmd_vel.angular.x = 0.0
        cmd.cmd_vel.angular.y = 0.0
        cmd.cmd_vel.angular.z = 0.0
        
        # Publish the command
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
