#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from alfie_msgs.srv import GDBServoService
from alfie_msgs.msg import GDBServoMemoryMap
import sys

def call_servo_service(servo_id=0, operation='r', address=0, value=0):
    """
    Call the driver0/servoservice to get servo memory map
    
    Args:
        servo_id: ID of the servo (default: 0)
        operation: Operation type (default: 'r' for read)
        address: Memory address (default: 0)
        value: Value to write (default: 0, not used for read operations)
    """
    
    # Initialize ROS2
    rclpy.init()
    node = rclpy.create_node('servo_memory_reader')
    
    # Wait for the service to be available
    service_name = '/alfie/driver0servoservice'
    print(f"Waiting for service {service_name}...")
    
    client = node.create_client(GDBServoService, service_name)
    
    while not client.wait_for_service(timeout_sec=1.0):
        node.get_logger().info(f'Service {service_name} not available, waiting again...')
        print(f'Service {service_name} not available, waiting again...')
    
    try:
        # Create request
        request = GDBServoService.Request()
        request.servo = servo_id
        request.operation = ord(operation)  # Convert char to int
        request.address = address
        request.value = value
        
        print(f"Calling service with servo={servo_id}, operation='{operation}', address={address}, value={value}")
        
        # Call the service
        future = client.call_async(request)
        rclpy.spin_until_future_complete(node, future, timeout_sec=5.0)
        
        if future.done():
            response = future.result()
            # Print the memory map
            if response is not None:
                print_memory_map(response.memorymap)
                # Cleanup
                node.destroy_node()
                rclpy.shutdown()
                return response.memorymap
            else:
                print("Failed to get memory map - response is None")
                # Cleanup
                node.destroy_node()
                rclpy.shutdown()
                return None
        else:
            print("Service call timed out")
            node.destroy_node()
            rclpy.shutdown()
            return None
        
    except Exception as e:
        print(f"Service call failed: {e}")
        node.destroy_node()
        rclpy.shutdown()
        return None

def print_memory_map(memory_map):
    """
    Print the servo memory map in a readable format
    """
    print("\n" + "="*60)
    print("SERVO MEMORY MAP")
    print("="*60)
    
    # Firmware information
    print(f"Firmware Version: {memory_map.firmwaremajor}.{memory_map.firmwaresub}")
    print(f"Servo Version: {memory_map.servomajor}.{memory_map.servosub}")
    print(f"Servo ID: {memory_map.servoid}")  
    print(f"Max Torque: {memory_map.maxtorque}")

def main():
    """
    Main function - parse command line arguments and call service
    """
    # Default values
    servo_id = 8
    operation = 'R'  # write operation
    address = 0
    value = 0
    
    print(f"Servo Memory Reader")
    print(f"Usage: python3 servo_memory_reader.py [servo_id] [operation] [address] [value]")
    print(f"Current parameters: servo_id={servo_id}, operation='{operation}', address={address}, value={value}")
    print()
    
    # Call the service
    memory_map = call_servo_service(servo_id, operation, address, value)
    
    if memory_map is None:
        print("Failed to retrieve memory map")
        sys.exit(1)
    else:
        print("\nMemory map retrieved successfully!")

if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        print("Program interrupted by user")