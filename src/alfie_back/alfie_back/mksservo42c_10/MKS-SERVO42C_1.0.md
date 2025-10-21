# MKS SERVO42C Serial Communication Guide (V1.0)

**Source**: [MKS-SERVO42C Wiki](https://github.com/makerbase-mks/MKS-SERVO42C/wiki/Serial-communication-description-V1.0)  
**Applicable to**: MKS-SERVO42C-V1.0  
**Last Updated**: Mar 28, 2022

---

## ⚠️ Important Setup Notes

- **Default device address**: `E0` (must be set before use)
- **Default baud rate**: 38400
- **All commands use HEX format**
- **Multi-byte values are BIG-ENDIAN** (MSB first)

---

## Quick Reference Table

| Command | Description | Parameters | Response | Notes |
|---------|-------------|------------|----------|-------|
| `E0 30` | Read encoder value | None | `E0 <uint16>` | 0x0000-0xFFFF per rotation |
| `E0 33` | Read pulse count | None | `E0 <int32>` | Cumulative pulses received |
| `E0 36` | Read motor angle | None | `E0 <int32>` | 0-65535 = 0-360° |
| `E0 39` | Read angle error | None | `E0 <int16>` | Target - actual angle |
| `E0 3A` | Read EN pin status | None | `E0 <status>` | 01=enabled, 02=disabled |
| `E0 3E` | Read motor status | None | `E0 <status>` | 01=blocked, 02=unblocked |
| `E0 84 XX` | Set subdivision | 1-256 | `E0 <result>` | Shows in MStep display |
| `E0 F3 XX` | Set EN pin | 01=enable, 00=disable | `E0 <result>` | Enable/disable motor |
| `E0 F6 XX` | Continuous speed | dir+speed | `E0 <result>` | See speed format below |
| `E0 F7 D7` | Stop motor | Fixed value D7 | `E0 <result>` | Emergency stop |
| `E0 FD XX XX XX` | Move to position | speed, pulses (2 bytes) | `E0 <result>` | Relative movement |
| `E0 FF XX` | Save/clear state | C8=save, CA=clear | `E0 <result>` | Persist F6 command |

---

## Data Format Reference

### Speed Byte Format (for F6 and FD commands)
```
Bit 7: Direction (0 = CW/forward, 1 = CCW/reverse)
Bits 6-0: Speed (0-127 decimal)

Examples:
0x01 = Forward, speed 1
0x50 = Forward, speed 80 (0x50 = 80 decimal)
0x81 = Reverse, speed 1  (0x80 | 0x01)
0xDA = Reverse, speed 90 (0x80 | 0x5A)
```

### Position/Angle Values
- **Encoder value**: 0x0000 to 0xFFFF (0-65535) represents one full rotation
- **Angle**: 0-65535 maps to 0-360°
- **Pulses**: Cumulative count, depends on subdivision setting

### Response Codes
- **Result byte**: `01` = success, `00` = failure
- **EN status**: `01` = enabled, `02` = disabled, `00` = error
- **Motor status**: `01` = blocked (stalled), `02` = unblocked, `00` = error

---

## Position Calculations

### Steps per Revolution
```
Steps per revolution = 200 (for 1.8° motor) × subdivision

Example with subdivision 16:
- Steps/rev = 200 × 16 = 3200
- For 90° rotation: 3200 / 4 = 800 pulses = 0x0320
- For 180° rotation: 3200 / 2 = 1600 pulses = 0x0640
- For 360° rotation: 3200 pulses = 0x0C80
```

### Calculating Pulses for Distance (Linear Actuator)
```
With GT2 belt (2mm pitch) and 20-tooth pulley:
- Distance per revolution = 2mm × 20 = 40mm
- Steps per mm = 3200 steps / 40mm = 80 steps/mm (at subdivision 16)
- For 100mm travel: 100mm × 80 = 8000 pulses = 0x1F40
```

---

## Command Details

### Read Encoder Value (Motor must be calibrated)
**Command**: `E0 30`  
**Returns**: `E0 <uint16>`  
**Range**: 0x0000 to 0xFFFF (one rotation)

**Example**:
```
Send:   E0 30
Return: E0 40 00
Meaning: Encoder at position 0x4000 (approximately 90°)
```

---

### Read Pulse Count
**Command**: `E0 33`  
**Returns**: `E0 <int32>` (4 bytes)  
**Note**: Cumulative pulses received since power-on or reset

**Example**:
```
Send:   E0 33
Return: E0 00 00 01 00
Meaning: 256 pulses (0x00000100)
```

---

### Read Motor Shaft Angle
**Command**: `E0 36`  
**Returns**: `E0 <int32>` (4 bytes)  
**Range**: 0-65535 corresponds to 0-360°

**Example**:
```
Send:   E0 36
Return: E0 00 00 40 00
Meaning: 16384 decimal = 90° (65536 / 4)
```

---

### Read Angle Error
**Command**: `E0 39`  
**Returns**: `E0 <int16>` (2 bytes)  
**Note**: Error = target angle - actual angle (0-65535 = 0-360°)

**Example**:
```
Send:   E0 39
Return: E0 00 B7
Meaning: Error of ~1° (183 decimal ≈ 65536/360)
```

---

### Read EN Pin Status
**Command**: `E0 3A`  
**Returns**: `E0 <status>`  
**Status**: 01 = enabled, 02 = disabled, 00 = error

**Example**:
```
Send:   E0 3A
Return: E0 01
Meaning: Motor is enabled
```

---

### Read Motor Shaft Status
**Command**: `E0 3E`  
**Returns**: `E0 <status>`  
**Status**: 01 = blocked (stalled), 02 = unblocked, 00 = error

**Example**:
```
Send:   E0 3E
Return: E0 02
Meaning: Motor is running normally (not blocked)
```

---

### Set Subdivision
**Command**: `E0 84 <subdivision>`  
**Parameter**: 0x01 to 0xFF (1-255 decimal), 0x00 = 256  
**Returns**: `E0 <result>`  
**Note**: Value appears in MStep display option

**Examples**:
```
Send: E0 84 07    → Set subdivision to 7
Send: E0 84 10    → Set subdivision to 16 (0x10 = 16)
Send: E0 84 4E    → Set subdivision to 78 (0x4E = 78)
Send: E0 84 00    → Set subdivision to 256
Return: E0 01     → Success
```

---

### Set EN Pin Status (Enable/Disable Motor)
**Command**: `E0 F3 <status>`  
**Parameter**: 01 = enable, 00 = disable  
**Returns**: `E0 <result>`  
**⚠️ Important**: Always stop motor (F7) before disabling

**Examples**:
```
Send: E0 F3 01    → Enable motor
Return: E0 01     → Success

Send: E0 F3 00    → Disable motor
Return: E0 01     → Success
```

---

### Run Motor at Constant Speed
**Command**: `E0 F6 <speed_byte>`  
**Parameter**: See speed byte format above  
**Returns**: `E0 <result>`  
**⚠️ Note**: Motor continues until stopped with F7 command

**Examples**:
```
Send: E0 F6 01    → Forward at speed 1
Send: E0 F6 10    → Forward at speed 16
Send: E0 F6 5A    → Forward at speed 90 (0x5A = 90)
Send: E0 F6 81    → Reverse at speed 1  (0x80 | 0x01)
Send: E0 F6 DA    → Reverse at speed 90 (0x80 | 0x5A)
Return: E0 01     → Success
```

---

### Stop Motor
**Command**: `E0 F7 D7`  
**Parameter**: Fixed value 0xD7  
**Returns**: `E0 <result>`  
**Note**: Immediate stop, use before disabling motor

**Example**:
```
Send: E0 F7 D7
Return: E0 01
Meaning: Motor stopped successfully
```

---

### Move to Position (Relative Movement)
**Command**: `E0 FD <speed_byte> <pulses_high> <pulses_low>`  
**Parameters**:
- Byte 3: Speed and direction (see speed format)
- Bytes 4-5: Number of pulses (uint16, big-endian)

**Returns**: `E0 <result>`  
**Note**: Motor moves specified pulses then stops

**Examples** (1.8° motor, subdivision 16):
```
Send: E0 FD 01 0C 80    → Forward 1 rotation at speed 1
                           (0x0C80 = 3200 pulses = 360°)

Send: E0 FD 86 0C 80    → Reverse 1 rotation at speed 6
                           (0x86 = reverse + speed 6)

Send: E0 FD 32 03 20    → Forward 90° at speed 50
                           (0x0320 = 800 pulses = 90°)

Return: E0 01           → Success
```

---

### Save/Clear Constant Speed State
**Command**: `E0 FF <action>`  
**Parameter**: C8 = save, CA = clear  
**Returns**: `E0 <result>`  
**Note**: Saves F6 command to run on power-up

**Example** (Auto-run on startup):
```
1. Send: E0 F6 10       → Set forward speed 16
   Return: E0 01
   
2. Send: E0 FF C8       → Save this state
   Return: E0 01
   
Result: Motor will run forward at speed 16 on every power-up
```

**To clear**:
```
Send: E0 FF CA          → Clear saved state
Return: E0 01
```

---

## Python Code Examples

### Basic Setup and Communication
```python
import serial
import time

# Connect to servo
ser = serial.Serial(
    port='/dev/ttyUSB0',
    baudrate=38400,
    timeout=1.0
)

# Enable motor
ser.write(bytes([0xE0, 0xF3, 0x01]))
response = ser.read(2)
print(f"Enable: {response.hex()}")  # Should show 'e001'

# Read pulse count
ser.write(bytes([0xE0, 0x33]))
response = ser.read(5)  # E0 + 4 bytes
pulses = int.from_bytes(response[1:5], 'big', signed=True)
print(f"Pulses: {pulses}")

# Move forward at speed 50
ser.write(bytes([0xE0, 0xF6, 0x32]))  # 0x32 = 50
response = ser.read(2)

# Wait 2 seconds
time.sleep(2)

# Stop motor
ser.write(bytes([0xE0, 0xF7, 0xD7]))
response = ser.read(2)

# Disable motor
ser.write(bytes([0xE0, 0xF3, 0x00]))
response = ser.read(2)
```

### Move to Specific Position
```python
def move_to_position(direction, speed, pulses):
    """
    Move motor a specific number of pulses.
    
    Args:
        direction: 'CW' or 'CCW'
        speed: 0-127
        pulses: 0-65535
    """
    # Create speed byte
    if direction == 'CCW':
        speed_byte = 0x80 | speed
    else:
        speed_byte = speed
    
    # Split pulses into high and low bytes
    pulses_high = (pulses >> 8) & 0xFF
    pulses_low = pulses & 0xFF
    
    # Send command
    ser.write(bytes([0xE0, 0xFD, speed_byte, pulses_high, pulses_low]))
    response = ser.read(2)
    return response[1] == 0x01  # True if successful

# Example: Move 90 degrees at speed 30 (subdivision 16)
move_to_position('CW', 30, 800)  # 3200 pulses/rotation / 4 = 800
```

### Read Position
```python
def get_pulses():
    """Read cumulative pulse count."""
    ser.write(bytes([0xE0, 0x33]))
    response = ser.read(5)
    return int.from_bytes(response[1:5], 'big', signed=True)

def get_angle():
    """Read motor angle (0-65535 = 0-360°)."""
    ser.write(bytes([0xE0, 0x36]))
    response = ser.read(5)
    angle_raw = int.from_bytes(response[1:5], 'big', signed=False)
    angle_degrees = (angle_raw / 65536) * 360
    return angle_degrees

# Usage
pulses = get_pulses()
angle = get_angle()
print(f"Position: {pulses} pulses, {angle:.1f} degrees")
```

---

## Troubleshooting

### Motor Not Responding?
1. **Check serial connection**
   - Verify baud rate (default 38400)
   - Confirm device address (default 0xE0)
   - Check TX/RX wiring

2. **Enable the motor**
   ```python
   ser.write(bytes([0xE0, 0xF3, 0x01]))
   ```

3. **Read motor status**
   ```python
   ser.write(bytes([0xE0, 0x3A]))  # Check EN status
   ser.write(bytes([0xE0, 0x3E]))  # Check motor status
   ```

### Motor Stuttering or Stalling?
- **Lower the speed**: Try speeds 10-50 instead of max (127)
- **Check power supply**: Ensure adequate voltage and current
- **Reduce load**: Motor may be overloaded
- **Check subdivision**: Higher subdivision = smoother but slower

### Position Tracking Inaccurate?
- **Use pulse count (0x33)**, not encoder (0x30) for absolute position
- **Calibrate encoder** if using encoder-based positioning
- **Check for missed steps**: Monitor motor status (0x3E) for blocking

### Motor Runs on Power-Up?
- **Saved state active**: Clear with `E0 FF CA`
- Check if F6 command was saved with F7 C8

---

## Safety Guidelines

### ⚠️ Critical Safety Rules

1. **Always stop before disable**
   ```python
   ser.write(bytes([0xE0, 0xF7, 0xD7]))  # Stop
   time.sleep(0.1)
   ser.write(bytes([0xE0, 0xF3, 0x00]))  # Disable
   ```

2. **Limit maximum speed**
   - Stay within 0-100 for most applications
   - Max value 127 may cause overheating or damage

3. **Monitor motor status**
   - Check for blocking (0x3E command)
   - Watch for overheating during continuous operation

4. **Subdivision changes**
   - Stop motor before changing subdivision
   - May require motor restart to take effect

5. **Position limits**
   - Implement software limits for linear actuators
   - Use limit switches for safety

---

## Additional Resources

- **GitHub Repository**: [MKS-SERVO42C](https://github.com/makerbase-mks/MKS-SERVO42C)
- **Wiki**: [Serial Communication Docs](https://github.com/makerbase-mks/MKS-SERVO42C/wiki)
- **Hardware Manual**: Check manufacturer website for wiring diagrams

---

## Connection Diagram

```
Serial Connection:
┌─────────────────┐         ┌──────────────┐
│   Controller    │         │ MKS SERVO42C │
│  (e.g. Jetson)  │         │              │
│                 │         │              │
│  TX ────────────┼────────►│ RX           │
│  RX ◄───────────┼─────────│ TX           │
│  GND ───────────┼─────────│ GND          │
│                 │         │              │
└─────────────────┘         │  VDC+ ◄──── Power Supply (+12-24V)
                            │  VDC- ◄──── Power Supply (GND)
                            │              │
                            │  EN  ◄────── Optional: Enable control
                            └──────────────┘

Power Requirements:
- Voltage: 12-24V DC
- Current: 1-3A (depending on load)
- Serial: 3.3V or 5V logic compatible

Optional Limit Switch Connection:
- Connect to separate GPIO/input pins on controller
- Use for homing and safety limits
- Active LOW or HIGH (configure in your code)
```

---

## Motor State Machine

```
Power Off
    │
    ├─ Power On
    ↓
┌──────────┐
│ Disabled │◄────────────────┐
└──────────┘                 │
    │                        │
    │ E0 F3 01 (Enable)     │ E0 F3 00 (Disable)
    ↓                        │
┌──────────┐                 │
│   Idle   │◄────────────────┤
└──────────┘                 │
    │                        │
    │ E0 F6 XX (Start)       │
    │ E0 FD XX XX XX         │
    ↓                        │
┌──────────┐                 │
│  Moving  │                 │
└──────────┘                 │
    │                        │
    │ E0 F7 D7 (Stop)        │
    │ Position reached (FD)  │
    └────────────────────────┘

States:
- Disabled: Motor unpowered, no holding torque
- Idle: Motor enabled, holding position
- Moving: Motor executing movement command
```

---

## Performance Characteristics

| Operation | Typical Time | Notes |
|-----------|-------------|-------|
| Read command | 5-10ms | Response time varies with baud rate |
| Write command | 2-5ms | Acknowledgment time |
| Position update | 10ms | Recommended for 100Hz control loop |
| Stop command | <1ms | Emergency stop (immediate) |
| Enable/Disable | 50-100ms | Motor ramp up/down time |

**Recommended polling rates:**
- Position monitoring: 50-100Hz (10-20ms intervals)
- Status checks: 10-20Hz (50-100ms intervals)
- High-speed control: Up to 200Hz (5ms intervals)

**Serial communication:**
- Baud rate: 38400 (default), can be changed
- Data bits: 8
- Parity: None
- Stop bits: 1

---

## Command Comparison Tables

### F6 (Continuous Speed) vs FD (Position Move)

| Feature | F6 (Continuous) | FD (Position) |
|---------|-----------------|---------------|
| Control type | Speed-based | Position-based |
| Stops automatically | No (requires F7 command) | Yes (at target) |
| Position feedback | External monitoring needed | Built-in tracking |
| Use case | Jogging, scanning, homing | Point-to-point moves |
| Precision | Lower (timing-dependent) | Higher (count-based) |
| Complexity | Simple | Medium |
| Best for | Variable speed, exploration | Repeatable positioning |

### Read Commands Comparison

| Command | Data Type | Use Case | Update Rate |
|---------|-----------|----------|-------------|
| 0x30 (Encoder) | uint16 | Absolute position within rotation | Real-time |
| 0x33 (Pulses) | int32 | Cumulative position tracking | Real-time |
| 0x36 (Angle) | int32 | Angular position (0-360°) | Real-time |
| 0x39 (Error) | int16 | Position control error | For tuning |

---

## Performance Tuning Guide

### For Smooth Operation
```
Configuration:
- Subdivision: 16-32 (good balance)
- Speed: 20-60 (moderate)
- Control rate: 50-100Hz

Benefits:
- Quiet operation
- Smooth motion
- Good precision
- Lower vibration

Trade-offs:
- Medium speed
- Medium power consumption
```

### For Maximum Speed
```
Configuration:
- Subdivision: 4-8 (coarser steps)
- Speed: 80-127 (fast)
- Control rate: 100-200Hz

Benefits:
- Highest speed
- Quick movements
- Shorter cycle times

Trade-offs:
- Noisier operation
- Less smooth
- Higher vibration
- May overshoot
```

### For Maximum Torque/Precision
```
Configuration:
- Subdivision: 1-4 (full steps)
- Speed: 10-30 (slow)
- Enable holding torque when stopped

Benefits:
- Maximum holding torque
- Highest precision
- Better for heavy loads
- Stall resistance

Trade-offs:
- Slower operation
- More vibration at low subdivisions
- Higher power consumption
```

### For Long-Running Applications
```
Configuration:
- Subdivision: 16 (standard)
- Speed: 30-50 (moderate)
- Duty cycle: <50% if possible

Benefits:
- Balanced performance
- Lower heat generation
- Extended motor life
- Reliable operation

Best practices:
- Monitor temperature
- Allow cooling periods
- Use adequate power supply
- Check for mechanical binding
```

---

## Common Application Patterns

### Linear Actuator Homing Routine
```python
def home_actuator(servo, limit_switch_pin):
    """
    Home linear actuator to bottom limit switch.
    
    Args:
        servo: MKSServo42C instance
        limit_switch_pin: Function to read limit switch state
    
    Returns:
        True if successful, False otherwise
    """
    # Check if already at limit
    if limit_switch_pin():
        # Move up slightly to clear switch
        servo.move("CW", 0x20)  # Move up at speed 32
        time.sleep(0.5)
        servo.stop()
        time.sleep(0.1)
    
    # Move down until limit switch triggers
    servo.move("CCW", 0x30)  # Move down at speed 48
    
    start_time = time.time()
    while not limit_switch_pin():
        if time.time() - start_time > 30:  # 30 second timeout
            servo.stop()
            return False
        time.sleep(0.01)
    
    servo.stop()
    time.sleep(0.1)
    
    # Save current position as zero reference
    offset_pulses = servo.read_pulses_received()
    
    return True
```

### Position-Based Movement with Feedback
```python
def move_to_position(servo, target_mm, speed, steps_per_mm):
    """
    Move to absolute position with real-time monitoring.
    
    Args:
        servo: MKSServo42C instance
        target_mm: Target position in millimeters
        speed: Movement speed (0-127)
        steps_per_mm: Calibrated steps per millimeter
    
    Returns:
        Final position in mm
    """
    # Get current position
    current_pulses = servo.read_pulses_received()
    current_mm = current_pulses / steps_per_mm
    
    # Calculate movement
    delta_mm = target_mm - current_mm
    delta_pulses = int(abs(delta_mm) * steps_per_mm)
    
    if delta_pulses == 0:
        return current_mm
    
    # Determine direction
    direction = "CW" if delta_mm > 0 else "CCW"
    
    # Start movement with position monitoring
    servo.move(direction, speed)
    
    while True:
        current_pulses = servo.read_pulses_received()
        current_mm = current_pulses / steps_per_mm
        
        remaining = abs(target_mm - current_mm)
        
        # Stop when close enough (0.5mm tolerance)
        if remaining < 0.5:
            servo.stop()
            break
            
        time.sleep(0.01)
    
    # Return final position
    final_pulses = servo.read_pulses_received()
    return final_pulses / steps_per_mm
```

### Continuous Monitoring Loop
```python
def monitor_servo(servo, callback, interval=0.01):
    """
    Continuously monitor servo status and call callback.
    
    Args:
        servo: MKSServo42C instance
        callback: Function to call with status dict
        interval: Update interval in seconds
    """
    while True:
        status = {
            'pulses': servo.read_pulses_received(),
            'angle': servo.read_motor_shaft_angle(),
            'enabled': servo.read_en_pin_status() == 0x01,
            'blocked': servo.read_motor_shaft_status() == 0x01,
            'timestamp': time.time()
        }
        
        callback(status)
        time.sleep(interval)
```

### Speed Ramping for Smooth Starts/Stops
```python
def ramp_speed(servo, direction, target_speed, ramp_time=1.0):
    """
    Gradually ramp motor speed up or down.
    
    Args:
        servo: MKSServo42C instance
        direction: "CW" or "CCW"
        target_speed: Final speed (0-127)
        ramp_time: Time to reach target speed (seconds)
    """
    steps = 20
    interval = ramp_time / steps
    
    for i in range(steps + 1):
        speed = int((target_speed * i) / steps)
        servo.move(direction, speed)
        time.sleep(interval)
```

---

## ROS 2 Integration Example

### Complete ROS 2 Node Implementation
```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Bool
from std_srvs.srv import Trigger

class MKSServoNode(Node):
    """ROS 2 node for MKS SERVO42C control."""
    
    def __init__(self):
        super().__init__('mks_servo_node')
        
        # Parameters
        self.declare_parameter('port', '/dev/ttyUSB0')
        self.declare_parameter('baudrate', 38400)
        self.declare_parameter('steps_per_mm', 80.0)
        
        port = self.get_parameter('port').value
        baudrate = self.get_parameter('baudrate').value
        self.steps_per_mm = self.get_parameter('steps_per_mm').value
        
        # Initialize servo
        self.servo = MKSServo42C(port, baudrate)
        self.servo.enable()
        
        # Publishers
        self.position_pub = self.create_publisher(
            Float32, 'position', 10)
        self.enabled_pub = self.create_publisher(
            Bool, 'enabled', 10)
        
        # Subscribers
        self.target_sub = self.create_subscription(
            Float32, 'target_position', 
            self.target_callback, 10)
        
        # Services
        self.home_service = self.create_service(
            Trigger, 'home', self.home_callback)
        self.enable_service = self.create_service(
            Trigger, 'enable', self.enable_callback)
        
        # Timer for status updates
        self.create_timer(0.01, self.update_status)
        
        self.get_logger().info('MKS Servo node initialized')
    
    def update_status(self):
        """Publish current position and status at 100Hz."""
        pulses = self.servo.read_pulses_received()
        position_mm = pulses / self.steps_per_mm
        
        enabled = self.servo.read_en_pin_status() == 0x01
        
        self.position_pub.publish(Float32(data=position_mm))
        self.enabled_pub.publish(Bool(data=enabled))
    
    def target_callback(self, msg):
        """Handle target position commands."""
        target_mm = msg.data
        self.move_to_position(target_mm, speed=50)
    
    def move_to_position(self, target_mm, speed):
        """Move to absolute position."""
        # Implementation from earlier example
        pass
    
    def home_callback(self, request, response):
        """Home the actuator."""
        success = self.home_actuator()
        response.success = success
        response.message = "Homing complete" if success else "Homing failed"
        return response
    
    def enable_callback(self, request, response):
        """Enable/disable motor."""
        self.servo.enable()
        response.success = True
        response.message = "Motor enabled"
        return response

def main(args=None):
    rclpy.init(args=args)
    node = MKSServoNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
```

---

## Glossary

- **Baud Rate**: Serial communication speed (default 38400 bits/second)
- **Big-Endian**: Multi-byte values sent MSB (Most Significant Byte) first
- **Blocking**: Motor stall condition when physical obstruction prevents movement
- **CW (Clockwise)**: Forward/positive direction rotation
- **CCW (Counter-Clockwise)**: Reverse/negative direction rotation
- **Encoder**: Internal position sensor providing 0-65535 counts per rotation
- **EN Pin**: Enable pin - must be active for motor to have holding torque
- **Holding Torque**: Force motor exerts when stationary to maintain position
- **Homing**: Process of finding a reference position (usually via limit switch)
- **Micro-stepping**: See Subdivision
- **Pulse**: Single step command sent to motor
- **Stall**: See Blocking
- **Steps per Revolution**: Total steps for one complete rotation (200 × subdivision for 1.8° motor)
- **Subdivision (Micro-stepping)**: Dividing each full step into smaller increments
  - Higher values = smoother, quieter, slower
  - Lower values = faster, more torque, noisier
  - Common values: 1, 2, 4, 8, 16, 32

---

## Error Recovery Procedures

### Motor Unresponsive
```python
def recover_unresponsive_motor(servo):
    """
    Attempt to recover from unresponsive motor state.
    """
    # 1. Send stop command
    try:
        servo.stop()
        time.sleep(0.1)
    except:
        pass
    
    # 2. Disable motor
    try:
        servo.disable()
        time.sleep(1.0)
    except:
        pass
    
    # 3. Re-enable motor
    try:
        servo.enable()
        time.sleep(0.5)
    except:
        return False
    
    # 4. Check status
    try:
        status = servo.read_en_pin_status()
        motor_status = servo.read_motor_shaft_status()
        
        if status == 0x01 and motor_status != 0x00:
            return True
    except:
        pass
    
    return False
```

### Position Tracking Lost
```python
def recover_lost_position(servo, home_function):
    """
    Recover from lost position tracking.
    
    Args:
        servo: MKSServo42C instance
        home_function: Function to home the actuator
    
    Returns:
        True if recovery successful
    """
    # 1. Stop any movement
    servo.stop()
    time.sleep(0.5)
    
    # 2. Run homing routine
    if not home_function():
        return False
    
    # 3. Verify with test movement
    initial_pulses = servo.read_pulses_received()
    
    # Move 10mm and back
    servo.move_to("CW", 50, 800)  # Assuming 80 steps/mm
    time.sleep(1.0)
    mid_pulses = servo.read_pulses_received()
    
    servo.move_to("CCW", 50, 800)
    time.sleep(1.0)
    final_pulses = servo.read_pulses_received()
    
    # Check if position tracking is consistent
    if abs(final_pulses - initial_pulses) < 50:  # Within 50 steps
        return True
    
    return False
```

### Serial Communication Errors
```python
def recover_serial_error(servo):
    """
    Recover from serial communication errors.
    """
    try:
        # Close and reopen serial connection
        servo.disconnect()
        time.sleep(0.5)
        servo.connect()
        time.sleep(0.5)
        
        # Test communication
        status = servo.read_en_pin_status()
        if status in [0x01, 0x02]:
            return True
    except:
        pass
    
    return False
```

---

## Quick Debug Checklist

### Initial Setup
- [ ] Serial baud rate correct (default 38400)
- [ ] Device address correct (default 0xE0)
- [ ] TX/RX connections not swapped
- [ ] Ground connection solid
- [ ] Power supply connected (12-24V DC)
- [ ] Power supply adequate (1-3A capacity)

### Motor Not Moving
- [ ] Motor enabled (send `E0 F3 01`, check `E0 3A` returns `01`)
- [ ] Movement command sent correctly
- [ ] Speed value not zero
- [ ] Direction bit set correctly (for F6/FD commands)
- [ ] No mechanical binding or obstacles
- [ ] Check motor status (`E0 3E`) for blocking

### Position Tracking Issues
- [ ] Subdivision setting matches expected value
- [ ] Steps per mm calculated correctly
- [ ] Pulse count being read regularly
- [ ] No missed steps (check for blocking)
- [ ] Encoder calibrated (if using encoder readings)
- [ ] Position offset stored and applied correctly

### Communication Problems
- [ ] Serial port opened successfully
- [ ] Correct serial parameters (8N1)
- [ ] Response timeout adequate (>100ms recommended)
- [ ] Commands formatted in hex correctly
- [ ] Multi-byte values in big-endian order
- [ ] Reading correct number of response bytes

### Performance Issues
- [ ] Speed not too high for load
- [ ] Subdivision appropriate for application
- [ ] Power supply voltage stable
- [ ] Control loop rate not too fast
- [ ] Mechanical system has no excessive friction
- [ ] Motor not overheating

---

## Troubleshooting Decision Tree

```
Motor not working?
│
├─ No response to commands?
│  ├─ Check serial connection (baud, TX/RX, GND)
│  ├─ Verify device address (E0)
│  └─ Test with simple command (E0 3A)
│
├─ Motor enabled but not moving?
│  ├─ Check power supply (voltage, current)
│  ├─ Verify movement command format
│  ├─ Check for mechanical binding
│  └─ Read motor status (E0 3E)
│
├─ Motor moves erratically?
│  ├─ Lower speed setting
│  ├─ Check subdivision setting
│  ├─ Verify power supply stability
│  └─ Check for electrical noise
│
├─ Position tracking inaccurate?
│  ├─ Verify steps/mm calculation
│  ├─ Check for missed steps
│  ├─ Run homing routine
│  └─ Monitor for blocking (E0 3E)
│
└─ Motor overheating?
   ├─ Reduce duty cycle
   ├─ Lower speed
   ├─ Check for mechanical resistance
   └─ Improve cooling/ventilation
```

---

## Revision History

- **V1.0** (Mar 28, 2022): Original documentation
- **Enhanced V2.0** (Oct 19, 2025): Added comprehensive examples, tables, calculations, safety info, diagrams, troubleshooting, ROS 2 integration, and application patterns

