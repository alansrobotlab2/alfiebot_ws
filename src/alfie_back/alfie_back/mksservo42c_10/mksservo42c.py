"""
MKS SERVO42C Driver Module

A comprehensive Python driver for the MKS SERVO42C closed-loop stepper motor.
Supports firmware version 1.0 with complete command set implementation.

Features:
    - Full serial command interface
    - Position tracking and control
    - Speed control (continuous and position-based)
    - Motor status monitoring
    - Error handling and recovery
    - Optional logging support

Author: Enhanced for ROS 2 Integration
Date: October 2025
Version: 2.0
"""

import serial
import time
import logging
from typing import Optional, Tuple, Union
from enum import IntEnum


class MotorStatus(IntEnum):
    """Motor shaft status codes."""
    ERROR = 0x00
    BLOCKED = 0x01      # Motor stalled/obstructed
    UNBLOCKED = 0x02    # Motor running normally


class EnableStatus(IntEnum):
    """Motor enable pin status codes."""
    ERROR = 0x00
    ENABLED = 0x01      # Motor powered with holding torque
    DISABLED = 0x02     # Motor unpowered


class Direction:
    """Motor rotation direction constants."""
    CW = "CW"           # Clockwise (typical: up/forward)
    CCW = "CCW"         # Counter-clockwise (typical: down/reverse)


class MKSServo42CException(Exception):
    """Base exception for MKS Servo42C errors."""
    pass


class CommunicationError(MKSServo42CException):
    """Raised when serial communication fails."""
    pass


class CommandError(MKSServo42CException):
    """Raised when a command returns an error."""
    pass


class MKSServo42C:
    """
    Driver class for MKS SERVO42C closed-loop stepper motor.
    
    This class provides a complete interface to the MKS SERVO42C motor controller
    via serial communication. It supports position control, speed control, status
    monitoring, and configuration.
    
    Command Protocol:
        All commands are sent as: [Address] [Command] [Parameters...]
        Responses are: [Address] [Data...]
        
    Attributes:
        port (str): Serial port device path
        baudrate (int): Serial communication speed
        address (int): Device address (default 0xE0)
        timeout (float): Serial read timeout in seconds
        
    Example:
        >>> servo = MKSServo42C('/dev/ttyUSB0', 38400)
        >>> servo.enable()
        >>> servo.set_subdivision(16)
        >>> servo.move_to(Direction.CW, 50, 3200)  # One rotation
        >>> servo.stop()
        >>> servo.disable()
    """
    
    # ==================== Command Codes ====================
    # Read Commands (0x30-0x3F)
    _READ_ENCODER_VALUE = 0x30              # Read encoder position (0-65535)
    _READ_NUMBER_OF_PULSES_RECEIVED = 0x33  # Read cumulative pulse count
    _READ_MOTOR_SHAFT_ANGLE = 0x36          # Read motor angle (0-65535 = 0-360°)
    _READ_MOTOR_SHAFT_ERROR_ANGLE = 0x39    # Read position error
    _READ_EN_PIN_STATUS = 0x3A              # Read enable pin status
    _READ_MOTOR_SHAFT_STATUS = 0x3E         # Read motor blocking status
    
    # Write Commands (0x80-0xFF)
    _WRITE_SET_SUBDIVISION = 0x84           # Set micro-stepping (1-256)
    _WRITE_SET_ACTIVE_EN_PIN = 0x85         # Set EN pin active level
    _WRITE_SET_EN_PIN_STATUS = 0xF3         # Enable/disable motor
    _WRITE_RUN_CONSTANT_SPEED = 0xF6        # Continuous speed mode
    _WRITE_STOP_MOTOR = 0xF7                # Emergency stop
    _WRITE_RUN_MOTOR_TO_POSITION = 0xFD     # Position-based movement
    _WRITE_SAVE_CLEAR_F6_STATUS = 0xFF      # Save/clear startup behavior
    
    # Constants
    DEFAULT_ADDRESS = 0xE0
    DEFAULT_BAUDRATE = 38400
    DEFAULT_TIMEOUT = 0.1  # 100ms
    MAX_SPEED = 127
    STOP_COMMAND_BYTE = 0xD7
    
    def __init__(
        self,
        port: str = '/dev/ttyUSB0',
        baudrate: int = DEFAULT_BAUDRATE,
        timeout: float = DEFAULT_TIMEOUT,
        address: int = DEFAULT_ADDRESS,
        logger: Optional[logging.Logger] = None
    ):
        """
        Initialize the MKS SERVO42C controller.
        
        Args:
            port: Serial port device path (e.g., '/dev/ttyUSB0', '/dev/ttyTHS1', 'COM3')
            baudrate: Serial communication speed (default 38400)
                     Common values: 9600, 38400, 115200
            timeout: Serial read timeout in seconds (default 0.1)
            address: Device address in hex (default 0xE0)
            logger: Optional Python logger instance for debug output
            
        Raises:
            serial.SerialException: If serial port cannot be opened
            CommunicationError: If initial connection verification fails
            
        Example:
            >>> import logging
            >>> logging.basicConfig(level=logging.DEBUG)
            >>> logger = logging.getLogger(__name__)
            >>> servo = MKSServo42C('/dev/ttyTHS1', 115200, logger=logger)
        """
        self._port = port
        self._address = address
        self._baudrate = baudrate
        self._timeout = timeout
        self._logger = logger or logging.getLogger(__name__)
        
        # Internal state tracking
        self._is_connected = False
        self._last_direction = None
        self._last_speed = 0
        
        # Initialize serial connection
        try:
            self._ser = serial.Serial(
                port=self._port,
                baudrate=self._baudrate,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                timeout=self._timeout,
                write_timeout=self._timeout
            )
            self._is_connected = True
            self._logger.info(f"MKS SERVO42C connected on {port} at {baudrate} baud")
            
        except serial.SerialException as e:
            self._logger.error(f"Failed to open serial port {port}: {e}")
            raise CommunicationError(f"Cannot open serial port {port}: {e}")
        
        # Verify connection with a simple read
        try:
            status = self.read_en_pin_status()
            self._logger.debug(f"Initial EN pin status: {status}")
        except Exception as e:
            self._logger.warning(f"Could not verify initial connection: {e}")
    
    # ==================== Connection Management ====================
    
    def connect(self) -> bool:
        """
        Open the serial connection to the motor.
        
        Returns:
            True if connection successful, False otherwise
            
        Note:
            Usually called automatically in __init__, but can be used
            to reconnect after disconnect().
        """
        if self._is_connected and self._ser.is_open:
            self._logger.debug("Serial port already open")
            return True
            
        try:
            if not self._ser.is_open:
                self._ser.open()
                self._is_connected = True
                self._logger.info(f"Serial port {self._port} opened successfully")
                return True
        except serial.SerialException as e:
            self._logger.error(f"Failed to open serial port: {e}")
            self._is_connected = False
            return False
    
    def disconnect(self) -> None:
        """
        Close the serial connection to the motor.
        
        Best practice: Call stop() before disconnect() to ensure motor is stopped.
        
        Example:
            >>> servo.stop()
            >>> time.sleep(0.1)
            >>> servo.disconnect()
        """
        if self._ser.is_open:
            self._ser.close()
            self._is_connected = False
            self._logger.info(f"Serial port {self._port} closed")
    
    def is_connected(self) -> bool:
        """
        Check if serial connection is active.
        
        Returns:
            True if connected and port is open
        """
        return self._is_connected and self._ser.is_open
    
    # ==================== Low-Level Communication ====================
    
    def _send_command(
        self,
        command: int,
        params: Optional[bytes] = None,
        response_length: int = 2
    ) -> bytes:
        """
        Send a command to the motor and read response.
        
        Internal method for building and sending command packets.
        
        Args:
            command: Command byte (from command constants)
            params: Optional parameter bytes to append
            response_length: Expected response length in bytes
            
        Returns:
            Raw response bytes from motor
            
        Raises:
            CommunicationError: If send/receive fails
        """
        # Build command packet
        packet = bytearray([self._address, command])
        if params:
            packet.extend(params)
        
        try:
            # Clear input buffer
            self._ser.reset_input_buffer()
            
            # Send command
            self._ser.write(bytes(packet))
            self._logger.debug(f"Sent: {packet.hex(' ')}")
            
            # Read response
            response = self._ser.read(response_length)
            
            if len(response) != response_length:
                raise CommunicationError(
                    f"Expected {response_length} bytes, received {len(response)}"
                )
            
            self._logger.debug(f"Received: {response.hex(' ')}")
            return response
            
        except serial.SerialException as e:
            self._logger.error(f"Serial communication error: {e}")
            raise CommunicationError(f"Serial error: {e}")
    
    @staticmethod
    def _encode_speed_direction(direction: str, speed: int) -> int:
        """
        Encode speed and direction into single byte.
        
        Bit 7: Direction (0=CCW, 1=CW)
        Bits 6-0: Speed (0-127)
        
        Args:
            direction: 'CW' or 'CCW'
            speed: Speed value 0-127
            
        Returns:
            Encoded byte value
            
        Raises:
            ValueError: If direction invalid or speed out of range
        """
        if direction not in [Direction.CW, Direction.CCW]:
            raise ValueError(f"Direction must be '{Direction.CW}' or '{Direction.CCW}'")
        
        if not 0 <= speed <= 127:
            raise ValueError(f"Speed must be 0-127, got {speed}")
        
        # CW sets bit 7, CCW keeps it clear
        return speed | 0x80 if direction == Direction.CW else speed
    
    # ==================== Read Commands ====================
    
    def read_encoder_value(self) -> int:
        """
        Read the encoder absolute position value.
        
        The encoder provides absolute position within one rotation.
        Motor must be calibrated for accurate readings.
        
        Returns:
            Encoder value (0x0000 to 0xFFFF)
            0x0000 = 0°
            0xFFFF = 359.99°
            
        Note:
            This is an absolute position sensor, resets each rotation.
            For cumulative tracking, use read_pulses_received().
            
        Example:
            >>> encoder = servo.read_encoder_value()
            >>> angle_deg = (encoder / 65536) * 360
            >>> print(f"Current angle: {angle_deg:.1f}°")
        """
        response = self._send_command(self._READ_ENCODER_VALUE, response_length=3)
        value = int.from_bytes(response[1:3], 'big', signed=False)
        self._logger.debug(f"Encoder value: {value} (0x{value:04X})")
        return value
    
    def read_pulses_received(self) -> int:
        """
        Read the cumulative pulse count since power-on or reset.
        
        This is the primary method for position tracking. The count
        accumulates all steps sent to the motor and can be positive
        or negative depending on direction.
        
        Returns:
            Pulse count (int32, signed)
            Positive = net CW movement
            Negative = net CCW movement
            
        Note:
            - Persists across movements
            - Can overflow at ±2,147,483,647
            - Use for position control and homing
            
        Example:
            >>> # Track position in mm
            >>> pulses = servo.read_pulses_received()
            >>> position_mm = pulses / steps_per_mm
        """
        response = self._send_command(
            self._READ_NUMBER_OF_PULSES_RECEIVED,
            response_length=5
        )
        pulses = int.from_bytes(response[1:5], 'big', signed=True)
        self._logger.debug(f"Pulses received: {pulses}")
        return pulses
    
    def read_motor_shaft_angle(self) -> int:
        """
        Read the current motor shaft angle.
        
        Returns angular position mapped to 0-65535 range.
        
        Returns:
            Angle value (0 to 65535)
            0 = 0°
            16384 = 90°
            32768 = 180°
            49152 = 270°
            65535 ≈ 360°
            
        Example:
            >>> angle_raw = servo.read_motor_shaft_angle()
            >>> angle_deg = (angle_raw / 65536) * 360
            >>> print(f"Motor angle: {angle_deg:.2f}°")
        """
        response = self._send_command(self._READ_MOTOR_SHAFT_ANGLE, response_length=5)
        angle = int.from_bytes(response[1:5], 'big', signed=False)
        self._logger.debug(f"Motor shaft angle: {angle}")
        return angle
    
    def read_motor_shaft_error_angle(self) -> int:
        """
        Read the position error (target - actual).
        
        The error represents how far the motor is from its commanded position.
        Useful for tuning and detecting issues.
        
        Returns:
            Error angle (int16, signed)
            Positive = motor behind target (needs to catch up)
            Negative = motor ahead of target
            0 = perfect position
            
        Note:
            Error of 182 ≈ 1° error (65536/360)
            
        Example:
            >>> error = servo.read_motor_shaft_error_angle()
            >>> error_deg = (error / 65536) * 360
            >>> if abs(error_deg) > 5:
            ...     print(f"Warning: Position error {error_deg:.1f}°")
        """
        response = self._send_command(
            self._READ_MOTOR_SHAFT_ERROR_ANGLE,
            response_length=3
        )
        error = int.from_bytes(response[1:3], 'big', signed=True)
        self._logger.debug(f"Motor shaft error: {error}")
        return error
    
    def read_en_pin_status(self) -> EnableStatus:
        """
        Read the current EN (enable) pin status.
        
        Returns:
            EnableStatus enum:
                ENABLED (0x01): Motor powered with holding torque
                DISABLED (0x02): Motor unpowered, free to move
                ERROR (0x00): Status read error
                
        Example:
            >>> status = servo.read_en_pin_status()
            >>> if status == EnableStatus.ENABLED:
            ...     print("Motor is enabled and holding position")
            >>> elif status == EnableStatus.DISABLED:
            ...     print("Motor is disabled (no holding torque)")
        """
        response = self._send_command(self._READ_EN_PIN_STATUS, response_length=2)
        status = EnableStatus(response[1])
        self._logger.debug(f"EN pin status: {status.name} (0x{status:02X})")
        return status
    
    def read_motor_shaft_status(self) -> MotorStatus:
        """
        Read motor shaft blocking/stall status.
        
        Indicates if the motor is mechanically blocked or running freely.
        Useful for detecting collisions, jams, or mechanical issues.
        
        Returns:
            MotorStatus enum:
                UNBLOCKED (0x02): Motor running normally
                BLOCKED (0x01): Motor stalled/obstructed
                ERROR (0x00): Status read error
                
        Example:
            >>> status = servo.read_motor_shaft_status()
            >>> if status == MotorStatus.BLOCKED:
            ...     servo.stop()
            ...     print("Warning: Motor blocked!")
        """
        response = self._send_command(self._READ_MOTOR_SHAFT_STATUS, response_length=2)
        status = MotorStatus(response[1])
        self._logger.debug(f"Motor shaft status: {status.name} (0x{status:02X})")
        return status
    
    # ==================== Write Commands ====================
    
    def set_subdivision(self, subdivision: int) -> bool:
        """
        Set the micro-stepping subdivision level.
        
        Higher subdivision = smoother, quieter, but slower
        Lower subdivision = faster, more torque, but noisier
        
        Args:
            subdivision: Micro-stepping level (1-256)
                1 = Full step (200 steps/rev for 1.8° motor)
                2 = Half step
                4, 8, 16, 32, etc. = Micro-stepping
                256 = Maximum micro-stepping (requires 0x00)
                
        Returns:
            True if successful, False otherwise
            
        Note:
            - Changes may require motor restart
            - Affects steps_per_revolution calculation
            - Value appears in MStep display option
            
        Example:
            >>> servo.set_subdivision(16)  # Common for smooth operation
            >>> # Steps per revolution = 200 * 16 = 3200
        """
        if not (0 <= subdivision <= 256):
            raise ValueError(f"Subdivision must be 0-256, got {subdivision}")
        
        # Special case: 256 is encoded as 0x00
        sub_byte = 0 if subdivision == 256 else subdivision
        
        response = self._send_command(
            self._WRITE_SET_SUBDIVISION,
            params=bytes([sub_byte]),
            response_length=2
        )
        
        success = response[1] == 0x01
        if success:
            self._logger.info(f"Subdivision set to {subdivision}")
        else:
            self._logger.error(f"Failed to set subdivision to {subdivision}")
        
        return success
    
    def set_en_pin_status(self, enable: bool) -> bool:
        """
        Enable or disable the motor.
        
        When enabled: Motor has holding torque and can move
        When disabled: Motor is unpowered and can be moved freely
        
        Args:
            enable: True to enable, False to disable
            
        Returns:
            True if successful, False otherwise
            
        Warning:
            Always call stop() before disabling to avoid sudden loss of torque!
            
        Example:
            >>> # Safe disable sequence
            >>> servo.stop()
            >>> time.sleep(0.1)
            >>> servo.disable()
            
            >>> # Enable for operation
            >>> servo.enable()
            >>> time.sleep(0.1)  # Allow motor to energize
        """
        status_byte = 0x01 if enable else 0x00
        
        response = self._send_command(
            self._WRITE_SET_EN_PIN_STATUS,
            params=bytes([status_byte]),
            response_length=3
        )
        
        success = response[1] == 0x01
        action = "enabled" if enable else "disabled"
        
        if success:
            self._logger.info(f"Motor {action}")
        else:
            self._logger.error(f"Failed to {action[:-1]} motor")
        
        return success
    
    def enable(self) -> bool:
        """
        Enable the motor (convenience wrapper).
        
        Returns:
            True if successful
            
        Example:
            >>> servo.enable()
        """
        return self.set_en_pin_status(True)
    
    def disable(self) -> bool:
        """
        Disable the motor (convenience wrapper).
        
        Warning: Call stop() first!
        
        Returns:
            True if successful
            
        Example:
            >>> servo.stop()
            >>> time.sleep(0.1)
            >>> servo.disable()
        """
        return self.set_en_pin_status(False)
    
    def move(self, direction: str, speed: int) -> bool:
        """
        Run motor at constant speed (continuous movement).
        
        Motor will continue running until stop() is called.
        Use this for jogging, scanning, or homing routines.
        
        Args:
            direction: Direction.CW or Direction.CCW
            speed: Speed level (0-127)
                0 = stopped
                1-50 = slow to moderate
                51-100 = moderate to fast
                101-127 = very fast (use with caution)
                
        Returns:
            True if command successful, False otherwise
            
        Warning:
            Motor will NOT stop automatically! Must call stop().
            
        Example:
            >>> # Move forward at moderate speed
            >>> servo.move(Direction.CW, 50)
            >>> time.sleep(2.0)  # Run for 2 seconds
            >>> servo.stop()
            
            >>> # Homing routine
            >>> servo.move(Direction.CCW, 30)
            >>> while not limit_switch_triggered():
            ...     time.sleep(0.01)
            >>> servo.stop()
        """
        speed_byte = self._encode_speed_direction(direction, speed)
        
        response = self._send_command(
            self._WRITE_RUN_CONSTANT_SPEED,
            params=bytes([speed_byte]),
            response_length=2
        )
        
        success = response[1] == 0x01
        
        if success:
            self._last_direction = direction
            self._last_speed = speed
            self._logger.info(f"Motor moving {direction} at speed {speed}")
        else:
            self._logger.error(f"Failed to move {direction} at speed {speed}")
        
        return success
    
    def move_to(self, direction: str, speed: int, pulses: int) -> bool:
        """
        Move motor a specific number of pulses then stop.
        
        This is position-based movement. Motor automatically stops
        when the specified pulse count is reached.
        
        Args:
            direction: Direction.CW or Direction.CCW
            speed: Speed level (0-127)
            pulses: Number of pulses to move (0-65535)
                For 1.8° motor with subdivision 16:
                - 3200 pulses = 360° (one revolution)
                - 800 pulses = 90°
                - 1600 pulses = 180°
                
        Returns:
            True if command successful, False otherwise
            
        Note:
            This is a relative movement from current position.
            Use read_pulses_received() to track absolute position.
            
        Example:
            >>> # Move one rotation at moderate speed
            >>> servo.move_to(Direction.CW, 50, 3200)
            >>> # Motor stops automatically after 3200 pulses
            
            >>> # Move specific distance (linear actuator)
            >>> # 80 steps/mm, move 100mm
            >>> servo.move_to(Direction.CW, 60, 8000)
        """
        if not 0 <= pulses <= 65535:
            raise ValueError(f"Pulses must be 0-65535, got {pulses}")
        
        speed_byte = self._encode_speed_direction(direction, speed)
        pulses_bytes = pulses.to_bytes(2, 'big')
        
        params = bytes([speed_byte]) + pulses_bytes
        
        response = self._send_command(
            self._WRITE_RUN_MOTOR_TO_POSITION,
            params=params,
            response_length=2
        )
        
        success = response[1] == 0x01
        
        if success:
            self._logger.info(
                f"Motor moving {direction} for {pulses} pulses at speed {speed}"
            )
        else:
            self._logger.error(
                f"Failed to move {direction} for {pulses} pulses"
            )
        
        return success
    
    def stop(self) -> bool:
        """
        Stop the motor immediately (emergency stop).
        
        Stops any ongoing movement, whether from move() or move_to().
        Motor retains holding torque after stopping.
        
        Returns:
            True if successful, False otherwise
            
        Example:
            >>> servo.move(Direction.CW, 80)
            >>> time.sleep(1.0)
            >>> servo.stop()  # Emergency stop
            
            >>> # Safe shutdown
            >>> servo.stop()
            >>> time.sleep(0.1)
            >>> servo.disable()
        """
        # Note: Stop command has a fixed parameter byte 0xD7
        response = self._send_command(
            self._WRITE_STOP_MOTOR,
            params=bytes([self.STOP_COMMAND_BYTE]),
            response_length=2
        )
        
        # Flush to ensure command is sent immediately
        self._ser.flush()
        
        success = response[1] == 0x01
        
        if success:
            self._logger.info("Motor stopped")
            self._last_speed = 0
        else:
            self._logger.error("Failed to stop motor")
        
        return success
    
    def save_startup_behavior(self) -> bool:
        """
        Save current move() settings to run on power-up.
        
        After calling this, the motor will automatically run at the
        last move() speed and direction every time it powers on.
        
        Returns:
            True if successful
            
        Warning:
            Use with caution! Motor will start moving on power-up.
            
        Example:
            >>> # Set motor to run on startup
            >>> servo.move(Direction.CW, 30)
            >>> servo.save_startup_behavior()
            >>> # Now motor runs forward at speed 30 on every power-up
            
            >>> # Clear startup behavior
            >>> servo.clear_startup_behavior()
        """
        response = self._send_command(
            self._WRITE_SAVE_CLEAR_F6_STATUS,
            params=bytes([0xC8]),  # 0xC8 = save
            response_length=2
        )
        
        success = response[1] == 0x01
        
        if success:
            self._logger.info("Startup behavior saved")
        else:
            self._logger.error("Failed to save startup behavior")
        
        return success
    
    def clear_startup_behavior(self) -> bool:
        """
        Clear saved startup behavior.
        
        Motor will no longer auto-run on power-up.
        
        Returns:
            True if successful
            
        Example:
            >>> servo.clear_startup_behavior()
        """
        response = self._send_command(
            self._WRITE_SAVE_CLEAR_F6_STATUS,
            params=bytes([0xCA]),  # 0xCA = clear
            response_length=2
        )
        
        success = response[1] == 0x01
        
        if success:
            self._logger.info("Startup behavior cleared")
        else:
            self._logger.error("Failed to clear startup behavior")
        
        return success
    
    # ==================== Helper Methods ====================
    
    def is_enabled(self) -> bool:
        """
        Check if motor is currently enabled.
        
        Returns:
            True if motor is enabled
            
        Example:
            >>> if not servo.is_enabled():
            ...     servo.enable()
        """
        return self.read_en_pin_status() == EnableStatus.ENABLED
    
    def is_blocked(self) -> bool:
        """
        Check if motor is currently blocked/stalled.
        
        Returns:
            True if motor is blocked
            
        Example:
            >>> if servo.is_blocked():
            ...     servo.stop()
            ...     print("Motor stalled!")
        """
        return self.read_motor_shaft_status() == MotorStatus.BLOCKED
    
    def get_status_dict(self) -> dict:
        """
        Get comprehensive motor status as dictionary.
        
        Returns:
            Dictionary with all status information
            
        Example:
            >>> status = servo.get_status_dict()
            >>> print(f"Position: {status['pulses']} pulses")
            >>> print(f"Enabled: {status['enabled']}")
            >>> print(f"Blocked: {status['blocked']}")
        """
        return {
            'pulses': self.read_pulses_received(),
            'angle': self.read_motor_shaft_angle(),
            'error': self.read_motor_shaft_error_angle(),
            'enabled': self.is_enabled(),
            'blocked': self.is_blocked(),
            'connected': self.is_connected(),
            'last_direction': self._last_direction,
            'last_speed': self._last_speed,
        }
    
    def __repr__(self) -> str:
        """String representation of the servo object."""
        return (
            f"MKSServo42C(port='{self._port}', "
            f"baudrate={self._baudrate}, "
            f"address=0x{self._address:02X}, "
            f"connected={self.is_connected()})"
        )
    
    def __enter__(self):
        """Context manager entry."""
        return self
    
    def __exit__(self, exc_type, exc_val, exc_tb):
        """Context manager exit - ensures clean disconnect."""
        self.stop()
        time.sleep(0.1)
        self.disable()
        self.disconnect()
