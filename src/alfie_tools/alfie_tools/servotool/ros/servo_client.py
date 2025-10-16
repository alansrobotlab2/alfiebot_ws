"""ROS2 service client wrapper for servo communication."""

import rclpy
from rclpy.node import Node
from alfie_msgs.srv import ServoService


class ServoServiceClient:
    """Wrapper for servo service communication."""
    
    def __init__(self, node: Node):
        """Initialize servo service clients.
        
        Args:
            node: ROS2 node instance to create clients from
        """
        self.node = node
        
        # Create service clients for both driver buses
        self.driver0 = node.create_client(ServoService, "/driver0servoservice")
        self.driver1 = node.create_client(ServoService, "/driver1servoservice")
        
        self._wait_for_services()
        
    def _wait_for_services(self):
        """Wait for both servo services to become available."""
        while not self.driver0.wait_for_service(timeout_sec=1.0):
            self.node.get_logger().info("Driver0 Service not available, waiting...")
            
        while not self.driver1.wait_for_service(timeout_sec=1.0):
            self.node.get_logger().info("Driver1 Service not available, waiting...")
    
    def read_memory_map(self, bus: int, servo_id: int):
        """Read complete memory map from servo.
        
        Args:
            bus: Servo bus number (0 or 1)
            servo_id: Servo ID (1-10)
            
        Returns:
            ServoMemoryMap object or None on timeout
        """
        request = ServoService.Request()
        request.servo = servo_id
        request.operation = ord('r')
        request.address = 0
        request.value = 0
        
        client = self.driver0 if bus == 0 else self.driver1
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=5.0)
        
        if future.done():
            return future.result().memorymap
        else:
            self.node.get_logger().error(
                f"Timeout reading from bus {bus}, servo {servo_id}"
            )
            return None
    
    def write_value(self, bus: int, servo_id: int, address: int, value: int) -> bool:
        """Write value to servo memory address.
        
        Args:
            bus: Servo bus number (0 or 1)
            servo_id: Servo ID (1-10)
            address: Memory address to write
            value: Value to write
            
        Returns:
            True if write succeeded, False otherwise
        """
        request = ServoService.Request()
        request.servo = servo_id
        request.operation = ord('W')
        request.address = address
        request.value = value
        
        client = self.driver0 if bus == 0 else self.driver1
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=1.0)
        
        return future.done()
