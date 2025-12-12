#!/usr/bin/env python3
"""
Analyze TF message CDR format to understand byte layout.
This helps debug the JavaScript CDR decoder.
"""

import rclpy
from rclpy.node import Node
from rclpy.serialization import serialize_message, deserialize_message
from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import TransformStamped
import struct


class TFAnalyzer(Node):
    def __init__(self):
        super().__init__('tf_analyzer')
        
        self.subscription = self.create_subscription(
            TFMessage,
            '/alfie/tf',
            self.tf_callback,
            10
        )
        self.message_count = 0
        self.get_logger().info('TF Analyzer started, listening to /alfie/tf')
        
    def tf_callback(self, msg: TFMessage):
        self.message_count += 1
        
        if self.message_count > 3:
            return  # Only analyze first few messages
            
        self.get_logger().info(f'\n{"="*60}')
        self.get_logger().info(f'TFMessage #{self.message_count}: {len(msg.transforms)} transforms')
        
        # Serialize to CDR format
        cdr_bytes = serialize_message(msg)
        self.get_logger().info(f'Total CDR bytes: {len(cdr_bytes)}')
        
        # Print hex dump
        self.print_hex_dump(cdr_bytes, max_bytes=256)
        
        # Decode manually to verify
        self.manual_decode(cdr_bytes, msg)
        
    def print_hex_dump(self, data: bytes, max_bytes: int = 128):
        """Print hex dump with ASCII representation"""
        self.get_logger().info('\nHex dump:')
        for i in range(0, min(len(data), max_bytes), 16):
            hex_part = ' '.join(f'{b:02x}' for b in data[i:i+16])
            ascii_part = ''.join(chr(b) if 32 <= b < 127 else '.' for b in data[i:i+16])
            self.get_logger().info(f'{i:04x}: {hex_part:<48} {ascii_part}')
            
    def manual_decode(self, data: bytes, original_msg: TFMessage):
        """Manually decode CDR to verify offsets"""
        self.get_logger().info('\n--- Manual CDR decode ---')
        
        offset = 0
        
        # CDR encapsulation header (4 bytes)
        encap = data[0:4]
        little_endian = encap[0] == 0x00 and encap[1] == 0x01
        self.get_logger().info(f'Offset {offset}: CDR header = {encap.hex()} (little_endian={little_endian})')
        offset = 4
        
        endian = '<' if little_endian else '>'
        
        # TFMessage has one field: TransformStamped[] transforms
        transform_count = struct.unpack_from(f'{endian}I', data, offset)[0]
        self.get_logger().info(f'Offset {offset}: transform_count = {transform_count}')
        offset += 4
        
        # Just decode first transform carefully
        self.get_logger().info(f'\n  Transform #0:')
        
        # Header: stamp
        stamp_sec = struct.unpack_from(f'{endian}i', data, offset)[0]  # int32
        self.get_logger().info(f'  Offset {offset}: stamp.sec = {stamp_sec}')
        offset += 4
        
        stamp_nsec = struct.unpack_from(f'{endian}I', data, offset)[0]  # uint32
        self.get_logger().info(f'  Offset {offset}: stamp.nanosec = {stamp_nsec}')
        offset += 4
        
        # frame_id string
        str_len = struct.unpack_from(f'{endian}I', data, offset)[0]
        self.get_logger().info(f'  Offset {offset}: frame_id length = {str_len}')
        offset += 4
        frame_id = data[offset:offset + str_len - 1].decode('utf-8')
        self.get_logger().info(f'  Offset {offset}: frame_id = "{frame_id}"')
        offset += str_len
        self.get_logger().info(f'  After frame_id string, offset = {offset}')
        
        # Align to 4 after string
        if offset % 4 != 0:
            pad = 4 - (offset % 4)
            self.get_logger().info(f'  Padding {pad} bytes (offset {offset} -> {offset + pad})')
            offset += pad
        
        # child_frame_id string
        str_len = struct.unpack_from(f'{endian}I', data, offset)[0]
        self.get_logger().info(f'  Offset {offset}: child_frame_id length = {str_len}')
        offset += 4
        child_frame_id = data[offset:offset + str_len - 1].decode('utf-8')
        self.get_logger().info(f'  Offset {offset}: child_frame_id = "{child_frame_id}"')
        offset += str_len
        self.get_logger().info(f'  After child_frame_id string, offset = {offset}')
        
        # Align to 4 after string  
        if offset % 4 != 0:
            pad = 4 - (offset % 4)
            self.get_logger().info(f'  Padding {pad} bytes for 4-align (offset {offset} -> {offset + pad})')
            offset += pad
            
        self.get_logger().info(f'  After 4-byte alignment, offset = {offset}')
        self.get_logger().info(f'  Bytes at offset: {data[offset:offset+24].hex()}')
        
        # Check if there's a DHEADER (struct delimiter) for Transform
        # XCDR2 can have a 4-byte size header for mutable/appendable structs
        potential_dheader = struct.unpack_from(f'{endian}I', data, offset)[0]
        self.get_logger().info(f'  Potential DHEADER at offset {offset}: {potential_dheader} (0x{potential_dheader:08x})')
        
        # Let's try reading at various offsets
        for test_off in [48, 52, 56]:
            try:
                test_tx = struct.unpack_from(f'{endian}d', data, test_off)[0]
                self.get_logger().info(f'  float64 at offset {test_off} = {test_tx}')
            except:
                pass
        
        # Now we need 8-byte alignment for float64
        if offset % 8 != 0:
            pad = 8 - (offset % 8)
            self.get_logger().info(f'  Padding {pad} bytes for 8-align (offset {offset} -> {offset + pad})')
            offset += pad
        
        self.get_logger().info(f'  After 8-byte alignment, offset = {offset}')
        self.get_logger().info(f'  Bytes at offset: {data[offset:offset+16].hex()}')
        
        # Now read translation (3 float64) and rotation (4 float64)
        tx = struct.unpack_from(f'{endian}d', data, offset)[0]
        self.get_logger().info(f'  Offset {offset}: translation.x = {tx}')
        offset += 8
        
        ty = struct.unpack_from(f'{endian}d', data, offset)[0]
        self.get_logger().info(f'  Offset {offset}: translation.y = {ty}')
        offset += 8
        
        tz = struct.unpack_from(f'{endian}d', data, offset)[0]
        self.get_logger().info(f'  Offset {offset}: translation.z = {tz}')
        offset += 8
        
        qx = struct.unpack_from(f'{endian}d', data, offset)[0]
        self.get_logger().info(f'  Offset {offset}: rotation.x = {qx}')
        offset += 8
        
        qy = struct.unpack_from(f'{endian}d', data, offset)[0]
        self.get_logger().info(f'  Offset {offset}: rotation.y = {qy}')
        offset += 8
        
        qz = struct.unpack_from(f'{endian}d', data, offset)[0]
        self.get_logger().info(f'  Offset {offset}: rotation.z = {qz}')
        offset += 8
        
        qw = struct.unpack_from(f'{endian}d', data, offset)[0]
        self.get_logger().info(f'  Offset {offset}: rotation.w = {qw}')
        offset += 8
        
        # Verify against original
        orig = original_msg.transforms[0]
        self.get_logger().info(f'  Original: {orig.header.frame_id} -> {orig.child_frame_id}')
        self.get_logger().info(f'  Original pos: {orig.transform.translation.x:.6f}, {orig.transform.translation.y:.6f}, {orig.transform.translation.z:.6f}')
        
        match = abs(tx - orig.transform.translation.x) < 1e-6
        self.get_logger().info(f'  *** translation.x MATCH: {match} ***')
        
        self.get_logger().info(f'\nNext transform would start at offset: {offset}')
        
    def read_string(self, data: bytes, offset: int, endian: str) -> tuple:
        """Read a CDR string and return (string, new_offset)"""
        # String length (includes null terminator)
        str_len = struct.unpack_from(f'{endian}I', data, offset)[0]
        self.get_logger().info(f'  Offset {offset}: string_len = {str_len}')
        offset += 4
        
        # String content (excluding null)
        if str_len > 0:
            string_bytes = data[offset:offset + str_len - 1]
            string_val = string_bytes.decode('utf-8')
            offset += str_len
        else:
            string_val = ''
            
        # Align to 4 bytes after string
        if offset % 4 != 0:
            padding = 4 - (offset % 4)
            self.get_logger().info(f'  String padding: {padding} bytes (offset {offset} -> {offset + padding})')
            offset += padding
            
        return string_val, offset


def main():
    rclpy.init()
    node = TFAnalyzer()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
