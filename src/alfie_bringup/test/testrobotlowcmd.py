#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from alfie_msgs.msg import RobotLowCmd, ServoCmd
from rclpy.qos import QoSProfile, ReliabilityPolicy


class RobotLowCmdPublisher(Node):
    def __init__(self):
        super().__init__('test_robotlowcmd_publisher')
        
        # QoS profile matching the master_topics node
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        
        # Create publisher
        self.publisher = self.create_publisher(
            RobotLowCmd,
            'robotlowcmd',
            qos
        )
        
        # Create timer for 100Hz publishing (0.01 seconds = 10ms)
        self.timer = self.create_timer(0.01, self.publish_command)
        
        # Initialize counter for demonstration
        self.counter = 0
        
        self.get_logger().info('Test RobotLowCmd Publisher started - publishing at 100Hz')
    
    def publish_command(self):
        """Publish a test RobotLowCmd message with all zeros"""
        msg = RobotLowCmd()
        
        # Set eye PWM values to 0
        msg.eye_pwm = [1, 1]
        
        # Set driver PWM values to 0
        msg.driver_pwm = [1, 1]
        
        # Create 17 servo commands with all zeros
        for i in range(17):
            servo_cmd = ServoCmd()
            servo_cmd.torqueswitch = 0
            servo_cmd.acceleration = 0
            servo_cmd.targetlocation = 0
            msg.servo_cmd.append(servo_cmd)
        
        # Publish the message
        self.publisher.publish(msg)
        
        # Log periodically (every 100 messages = 1 second)
        if self.counter % 100 == 0:
            self.get_logger().info(f'Published command #{self.counter} (all zeros)')
        
        self.counter += 1
    
    def destroy_node(self):
        self.get_logger().info('Shutting down test publisher')
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = RobotLowCmdPublisher()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
