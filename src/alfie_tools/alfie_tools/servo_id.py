#!/usr/bin/env python3
"""
Servo ID Programming Tool
Programs the ID of a Waveshare serial bus servo (SMS/STS/SCSCL compatible)
"""

# Configuration variables
DEVICE = '/dev/ttyACM0'  # Serial port device
CURRENT_ID = 1           # Current servo ID to program (1-253)    
NEW_ID = 3               # New servo ID to program (1-253)



# Servo communication settings
BAUDRATE = 1000000
TIMEOUT = 0.5

# Memory addresses (for SMS_STS/SCSCL servos)
ADDR_ID = 5
ADDR_LOCK = 55

# Protocol constants
INST_PING = 0x01
INST_WRITE = 0x03
INST_WRITE_POS = 0x03
BROADCAST_ID = 0xFE

# Memory addresses for position control
ADDR_GOAL_POSITION_L = 42
ADDR_GOAL_POSITION_H = 43

import serial
import time

def calculate_checksum(data):
    """Calculate checksum for servo protocol"""
    return (~sum(data)) & 0xFF

def send_packet(ser, servo_id, instruction, address=None, data=None):
    """Send a packet to the servo"""
    if data is None:
        data = []
    
    if address is not None:
        # Write command with address
        length = len(data) + 3  # length + instruction + address + data
        packet = [0xFF, 0xFF, servo_id, length, instruction, address] + data
    else:
        # Ping command without address
        length = 2  # ID + length
        packet = [0xFF, 0xFF, servo_id, length, instruction]
    
    checksum = calculate_checksum(packet[2:])
    packet.append(checksum)
    
    ser.write(bytes(packet))
    print(f"Sent: {' '.join(f'{b:02X}' for b in packet)}")
    time.sleep(0.01)  # Small delay for servo to process

def read_response(ser, expected_length=100):
    """Read response from servo"""
    response = ser.read(expected_length)
    if response:
        print(f"Response: {' '.join(f'{b:02X}' for b in response)}")
        return response
    else:
        print("Response: (no response)")
        return None

def ping_servo(ser, servo_id):
    """Ping servo to check if it responds"""
    print(f"\nPinging servo ID {servo_id}...")
    ser.reset_input_buffer()
    send_packet(ser, servo_id, INST_PING)
    response = read_response(ser, 6)
    
    if response and len(response) >= 4:
        # Check for valid response header and matching ID
        if response[0] == 0xFF and response[1] == 0xFF and response[2] == servo_id:
            print(f"✓ Servo ID {servo_id} responded!")
            return True
    
    print(f"✗ No response from servo ID {servo_id}")
    return False

def unlock_eprom(ser, servo_id):
    """Unlock EPROM to allow writing"""
    print(f"\nUnlocking EPROM for servo ID {servo_id}...")
    ser.reset_input_buffer()
    send_packet(ser, servo_id, INST_WRITE, ADDR_LOCK, [0])
    read_response(ser)

def lock_eprom(ser, servo_id):
    """Lock EPROM to protect settings"""
    print(f"\nLocking EPROM for servo ID {servo_id}...")
    ser.reset_input_buffer()
    send_packet(ser, servo_id, INST_WRITE, ADDR_LOCK, [1])
    read_response(ser)

def write_id(ser, current_id, new_id):
    """Write new ID to servo"""
    print(f"\nWriting new ID {new_id} to servo...")
    ser.reset_input_buffer()
    send_packet(ser, current_id, INST_WRITE, ADDR_ID, [new_id])
    read_response(ser)

def set_servo_position(ser, servo_id, position):
    """Set servo to a specific position (0-4095)"""
    print(f"\nSetting servo ID {servo_id} to position {position}...")
    ser.reset_input_buffer()
    # Position is 2 bytes: low byte, high byte
    pos_low = position & 0xFF
    pos_high = (position >> 8) & 0xFF
    send_packet(ser, servo_id, INST_WRITE_POS, ADDR_GOAL_POSITION_L, [pos_low, pos_high])
    read_response(ser)

def main():
    print("="*60)
    print("Servo ID Programming Tool")
    print("="*60)
    print(f"Device: {DEVICE}")
    print(f"Baudrate: {BAUDRATE}")
    print(f"Current ID: {CURRENT_ID}")
    print(f"New ID: {NEW_ID}")
    print("="*60)
    
    try:
        # Open serial port
        print(f"\nOpening serial port {DEVICE}...")
        ser = serial.Serial(
            port=DEVICE,
            baudrate=BAUDRATE,
            timeout=TIMEOUT,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE
        )
        
        # Clear buffers
        ser.reset_input_buffer()
        ser.reset_output_buffer()
        
        # Step 1: Verify servo responds on current ID
        print("\n" + "="*60)
        print("STEP 1: Verify servo on current ID")
        print("="*60)
        if not ping_servo(ser, CURRENT_ID):
            print(f"\n✗ FAILED: Servo does not respond on current ID {CURRENT_ID}")
            print("  Check connections and verify current ID is correct")
            ser.close()
            return 1
        
        # Step 2: Program the servo ID
        print("\n" + "="*60)
        print("STEP 2: Program new ID")
        print("="*60)
        unlock_eprom(ser, CURRENT_ID)
        time.sleep(0.1)
        
        write_id(ser, CURRENT_ID, NEW_ID)
        time.sleep(0.1)
        
        lock_eprom(ser, NEW_ID)  # Use new ID after writing
        time.sleep(0.1)
        
        # Step 3: Verify servo responds on new ID
        print("\n" + "="*60)
        print("STEP 3: Verify servo on new ID")
        print("="*60)
        if not ping_servo(ser, NEW_ID):
            print(f"\n✗ FAILED: Servo does not respond on new ID {NEW_ID}")
            print("  Programming may have failed")
            ser.close()
            return 1
        
        # Step 4: Verify servo no longer responds on old ID
        print("\n" + "="*60)
        print("STEP 4: Verify old ID is no longer active")
        print("="*60)
        if ping_servo(ser, CURRENT_ID):
            print(f"\n⚠ WARNING: Servo still responds on old ID {CURRENT_ID}")
            print("  This is unexpected - ID may not have changed")
        else:
            print(f"✓ Servo no longer responds on old ID {CURRENT_ID}")
        
        print("\n" + "="*60)
        print(f"✓ SUCCESS: Servo ID changed from {CURRENT_ID} to {NEW_ID}")
        print("="*60)
        
        # Step 5: Set servo to center position
        print("\n" + "="*60)
        print("STEP 5: Set servo to center position")
        print("="*60)
        set_servo_position(ser, NEW_ID, 2048)
        time.sleep(0.5)  # Give servo time to move
        print("✓ Servo set to position 2048 (center)")
        
        # Close serial port
        ser.close()
        
    except serial.SerialException as e:
        print(f"\n✗ Serial error: {e}")
        print(f"  Make sure {DEVICE} exists and you have permissions")
        return 1
    except Exception as e:
        print(f"\n✗ Error: {e}")
        import traceback
        traceback.print_exc()
        return 1
    
    return 0

if __name__ == '__main__':
    exit(main())
