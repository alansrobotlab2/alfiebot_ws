"""
Watchdog Health Check Classes

Contains all health monitoring classes used by the Master Status Node
to monitor the health of critical subsystems.
"""

import time
from abc import ABC, abstractmethod
from collections import deque
from dataclasses import dataclass
from typing import Dict, Deque, List, Optional, Tuple


# ============================================================================
# Constants
# ============================================================================

# Voltage is reported in 0.1V units
VOLTAGE_SCALE = 10.0

# Time window for rolling averages (in seconds)
ROLLING_WINDOW_SECONDS = 1.0

# Percentage to trim from each end for trimmed mean (0.1 = 10%)
TRIM_PERCENT = 0.1


# ============================================================================
# Health Check Base Class
# ============================================================================

class HealthCheck(ABC):
    """Base class for all health checks"""
    
    def __init__(self, name: str):
        self.name = name
    
    @abstractmethod
    def update(self) -> None:
        """Called by topic callbacks to update state"""
        pass
    
    @abstractmethod
    def check(self) -> Optional[str]:
        """
        Perform the health check.
        
        Returns:
            None if healthy, error message string if unhealthy
        """
        pass


# ============================================================================
# Rate Monitor Health Check
# ============================================================================

class RateMonitor(HealthCheck):
    """
    Monitors message rate for a topic.
    Reports error if rate deviates from expected by more than tolerance.
    """
    
    def __init__(self, name: str, expected_hz: float, tolerance_hz: float):
        super().__init__(name)
        self.expected_hz = expected_hz
        self.tolerance_hz = tolerance_hz
        self.message_count = 0
        self.last_check_time: Optional[float] = None
        self.last_check_count = 0
        self.current_rate: Optional[float] = None
    
    def update(self) -> None:
        """Called when a message is received"""
        self.message_count += 1
    
    def check(self) -> Optional[str]:
        """Check if rate is within tolerance"""
        current_time = time.time()
        
        # First check - just initialize
        if self.last_check_time is None:
            self.last_check_time = current_time
            self.last_check_count = self.message_count
            return None
        
        # Calculate rate
        elapsed = current_time - self.last_check_time
        if elapsed <= 0:
            return None
        
        messages_received = self.message_count - self.last_check_count
        self.current_rate = messages_received / elapsed
        
        # Update for next check
        self.last_check_time = current_time
        self.last_check_count = self.message_count
        
        # Check if within tolerance
        min_rate = self.expected_hz - self.tolerance_hz
        max_rate = self.expected_hz + self.tolerance_hz
        
        if self.current_rate < min_rate:
            return f'{self.name} rate too low: {self.current_rate:.1f}Hz (expected {self.expected_hz}Hz ± {self.tolerance_hz}Hz)'
        elif self.current_rate > max_rate:
            return f'{self.name} rate too high: {self.current_rate:.1f}Hz (expected {self.expected_hz}Hz ± {self.tolerance_hz}Hz)'
        
        return None


# ============================================================================
# Servo Voltage Monitor Health Check (monitors all servos)
# ============================================================================

class ServoVoltageMonitor(HealthCheck):
    """
    Monitors voltage levels for ALL servos on a GDB.
    Reports error if any servo's average voltage (over ROLLING_WINDOW_SECONDS)
    deviates from expected by more than tolerance.
    """
    
    def __init__(self, name: str, servo_names: list, expected_voltage: float, tolerance_voltage: float):
        super().__init__(name)
        self.servo_names = servo_names
        self.num_servos = len(servo_names)  # Only monitor servos that have names
        self.expected_voltage = expected_voltage
        self.tolerance_voltage = tolerance_voltage
        # Rolling buffer of (timestamp, voltage) tuples for each servo
        self.servo_voltage_history: Dict[int, Deque[Tuple[float, float]]] = {}
    
    def update(self, servo_states: list) -> None:
        """Called when servo states are received"""
        current_time = time.time()
        # Only process the actual number of servos on this board
        for i in range(min(len(servo_states), self.num_servos)):
            servo = servo_states[i]
            # Convert raw voltage (0.1V units) to actual voltage
            voltage = servo.current_voltage / VOLTAGE_SCALE
            
            # Initialize history deque if needed
            if i not in self.servo_voltage_history:
                self.servo_voltage_history[i] = deque()
            
            # Add new reading with timestamp
            self.servo_voltage_history[i].append((current_time, voltage))
            
            # Remove readings older than the window
            cutoff_time = current_time - ROLLING_WINDOW_SECONDS
            while self.servo_voltage_history[i] and self.servo_voltage_history[i][0][0] < cutoff_time:
                self.servo_voltage_history[i].popleft()
    
    def _get_servo_name(self, index: int) -> str:
        """Get human-readable servo name for the given index"""
        if index < len(self.servo_names):
            return self.servo_names[index]
        return f'Servo[{index}]'
    
    def _get_trimmed_mean_voltage(self, servo_idx: int) -> Optional[float]:
        """Calculate the trimmed mean voltage for a servo over the rolling window"""
        if servo_idx not in self.servo_voltage_history:
            return None
        history = self.servo_voltage_history[servo_idx]
        if not history:
            return None
        values = sorted(v for _, v in history)
        n = len(values)
        # Calculate how many values to trim from each end
        trim_count = int(n * TRIM_PERCENT)
        # Keep at least 1 value in the middle
        if n - 2 * trim_count < 1:
            trim_count = max(0, (n - 1) // 2)
        trimmed = values[trim_count:n - trim_count] if trim_count > 0 else values
        if not trimmed:
            return None
        return sum(trimmed) / len(trimmed)
    
    def check(self) -> Optional[str]:
        """Check if all servo average voltages are within tolerance"""
        if not self.servo_voltage_history:
            return None  # No data yet
        
        min_voltage = self.expected_voltage - self.tolerance_voltage
        max_voltage = self.expected_voltage + self.tolerance_voltage
        
        errors = []
        
        for servo_idx in self.servo_voltage_history.keys():
            avg_voltage = self._get_trimmed_mean_voltage(servo_idx)
            if avg_voltage is None:
                continue
            
            servo_name = self._get_servo_name(servo_idx)
            
            if avg_voltage < min_voltage:
                errors.append(f'{servo_name}={avg_voltage:.1f}V (low)')
            elif avg_voltage > max_voltage:
                errors.append(f'{servo_name}={avg_voltage:.1f}V (high)')
        
        if errors:
            return f'{self.name} voltage out of range ({self.expected_voltage}V ± {self.tolerance_voltage}V): {", ".join(errors)}'
        
        return None


# ============================================================================
# Diagnostic Timing Monitor Health Check
# ============================================================================

class DiagnosticTimingMonitor(HealthCheck):
    """
    Monitors GDB diagnostic timing values.
    Reports error if servo timing exceeds the maximum threshold.
    Reports warning if IMU update duration exceeds 10ms (monitored separately).
    """
    
    # Timing fields to monitor from GDBDiagnostics (excluding IMU which is monitored separately)
    SERVO_TIMING_FIELDS = [
        'pollservostatusduration',
        'updateservoidleduration',
        'updateservoactiveduration',
    ]
    
    # IMU timing threshold in ms
    IMU_MAX_MS = 10.0
    
    def __init__(self, name: str, max_total_ms: float):
        super().__init__(name)
        self.max_total_ms = max_total_ms
        self.current_timings: Dict[str, float] = {}
    
    def update(self, diagnostics) -> None:
        """Called when a message is received with GDBDiagnostics"""
        self.current_timings = {
            'pollservostatusduration': diagnostics.pollservostatusduration,
            'updateservoidleduration': diagnostics.updateservoidleduration,
            'updateservoactiveduration': diagnostics.updateservoactiveduration,
            'imuupdateduration': diagnostics.imuupdateduration,
        }
    
    def check(self) -> Optional[str]:
        """Check if servo timing exceeds maximum or IMU timing exceeds 10ms"""
        if not self.current_timings:
            return None  # No data yet
        
        errors = []
        
        # Calculate servo total duration in ms (excluding IMU)
        servo_total_ms = 0.0
        servo_timing_details = []
        for field_name in self.SERVO_TIMING_FIELDS:
            duration_ms = self.current_timings.get(field_name, 0.0)
            servo_total_ms += duration_ms
            servo_timing_details.append(f'{field_name}={duration_ms:.2f}ms')
        
        if servo_total_ms > self.max_total_ms:
            errors.append(f'{self.name} servo timing {servo_total_ms:.2f}ms exceeded {self.max_total_ms}ms: {", ".join(servo_timing_details)}')
        
        # Check IMU timing separately
        # imu_duration_ms = self.current_timings.get('imuupdateduration', 0.0)
        # if imu_duration_ms > self.IMU_MAX_MS:
        #     errors.append(f'{self.name} IMU update duration {imu_duration_ms:.2f}ms exceeded {self.IMU_MAX_MS}ms')
        
        if errors:
            return '; '.join(errors)
        
        return None


# ============================================================================
# Temperature Monitor Health Check
# ============================================================================

class TemperatureMonitor(HealthCheck):
    """
    Monitors temperature values.
    Uses rolling average over ROLLING_WINDOW_SECONDS for stable alerting.
    Reports warning if above warn threshold, error if above critical threshold.
    """
    
    def __init__(self, name: str, warn_temp: float, critical_temp: float):
        super().__init__(name)
        self.warn_temp = warn_temp
        self.critical_temp = critical_temp
        # Rolling buffer of (timestamp, temperature) tuples
        self.temp_history: Deque[Tuple[float, float]] = deque()
    
    def update(self, temp: float) -> None:
        """Called when a new temperature reading is available"""
        current_time = time.time()
        
        # Add new reading with timestamp
        self.temp_history.append((current_time, temp))
        
        # Remove readings older than the window
        cutoff_time = current_time - ROLLING_WINDOW_SECONDS
        while self.temp_history and self.temp_history[0][0] < cutoff_time:
            self.temp_history.popleft()
    
    def _get_trimmed_mean_temp(self) -> Optional[float]:
        """Calculate the trimmed mean temperature over the rolling window"""
        if not self.temp_history:
            return None
        values = sorted(t for _, t in self.temp_history)
        n = len(values)
        # Calculate how many values to trim from each end
        trim_count = int(n * TRIM_PERCENT)
        # Keep at least 1 value in the middle
        if n - 2 * trim_count < 1:
            trim_count = max(0, (n - 1) // 2)
        trimmed = values[trim_count:n - trim_count] if trim_count > 0 else values
        if not trimmed:
            return None
        return sum(trimmed) / len(trimmed)
    
    def check(self) -> Optional[str]:
        """Check if trimmed mean temperature is within safe limits"""
        avg_temp = self._get_trimmed_mean_temp()
        
        if avg_temp is None or avg_temp <= 0:
            return None  # No data yet
        
        if avg_temp >= self.critical_temp:
            return f'{self.name} CRITICAL: {avg_temp:.1f}°C >= {self.critical_temp}°C'
        elif avg_temp >= self.warn_temp:
            return f'{self.name} WARNING: {avg_temp:.1f}°C >= {self.warn_temp}°C'
        
        return None


# ============================================================================
# Threshold Monitor Health Check
# ============================================================================

class ThresholdMonitor(HealthCheck):
    """
    Monitors a percentage value (0-100).
    Reports warning if above warn threshold, error if above critical threshold.
    """
    
    def __init__(self, name: str, warn_percent: float, critical_percent: float):
        super().__init__(name)
        self.warn_percent = warn_percent
        self.critical_percent = critical_percent
        self.current_value: Optional[float] = None
    
    def update(self, value: float) -> None:
        """Called when a new value is available"""
        self.current_value = value
    
    def check(self) -> Optional[str]:
        """Check if value is within safe limits"""
        if self.current_value is None:
            return None  # No data yet
        
        if self.current_value >= self.critical_percent:
            return f'{self.name} CRITICAL: {self.current_value:.1f}% >= {self.critical_percent}%'
        elif self.current_value >= self.warn_percent:
            return f'{self.name} WARNING: {self.current_value:.1f}% >= {self.warn_percent}%'
        
        return None


# ============================================================================
# WiFi Monitor Health Check
# ============================================================================

class WifiMonitor(HealthCheck):
    """
    Monitors WiFi connection status and signal strength.
    Reports error if disconnected or signal too weak.
    """
    
    def __init__(self, name: str, min_signal_dbm: int = -75, require_connected: bool = True):
        super().__init__(name)
        self.min_signal_dbm = min_signal_dbm
        self.require_connected = require_connected
        self.is_connected: Optional[bool] = None
        self.signal_dbm: Optional[int] = None
        self.ssid: str = ""
    
    def update(self, connected: bool, signal_dbm: int, ssid: str = "") -> None:
        """Called when WiFi status is updated"""
        self.is_connected = connected
        self.signal_dbm = signal_dbm
        self.ssid = ssid
    
    def check(self) -> Optional[str]:
        """Check WiFi connection status and signal strength"""
        if self.is_connected is None:
            return None  # No data yet
        
        # Check connection status
        if self.require_connected and not self.is_connected:
            return f'{self.name} CRITICAL: WiFi disconnected!'
        
        # Check signal strength (only if connected)
        if self.is_connected and self.signal_dbm is not None:
            if self.signal_dbm < self.min_signal_dbm:
                return f'{self.name} WARNING: WiFi signal weak: {self.signal_dbm}dBm < {self.min_signal_dbm}dBm (SSID: {self.ssid})'
        
        return None


# ============================================================================
# Servo Monitor Health Check
# ============================================================================

class ServoMonitor(HealthCheck):
    """
    Monitors servo health for a GDB (servo driver board).
    Reports warnings for temperature, status codes, and current draw.
    Uses rolling window for temperature to handle spurious readings.
    """
    
    def __init__(self, name: str, servo_names: list,
                 temp_warn: float = 40.0,
                 current_warn_ma: float = 3000.0):
        super().__init__(name)
        self.servo_names = servo_names
        self.temp_warn = temp_warn
        self.current_warn_ma = current_warn_ma
        
        # Store latest servo states
        self.servo_states: list = []
        # Rolling buffer of (timestamp, temperature) tuples for each servo
        self.servo_temp_history: Dict[int, Deque[Tuple[float, float]]] = {}
    
    def update(self, servo_states: list) -> None:
        """Called when servo states are received"""
        self.servo_states = servo_states
        current_time = time.time()
        
        # Track temperature history for each servo
        for i, servo in enumerate(servo_states):
            if i not in self.servo_temp_history:
                self.servo_temp_history[i] = deque()
            
            self.servo_temp_history[i].append((current_time, float(servo.current_temperature)))
            
            # Remove readings older than the window
            cutoff_time = current_time - ROLLING_WINDOW_SECONDS
            while self.servo_temp_history[i] and self.servo_temp_history[i][0][0] < cutoff_time:
                self.servo_temp_history[i].popleft()
    
    def _get_servo_name(self, index: int) -> str:
        """Get human-readable servo name for the given index"""
        if index < len(self.servo_names):
            return self.servo_names[index]
        return f'Servo[{index}]'
    
    def _get_trimmed_mean_temp(self, servo_idx: int) -> Optional[float]:
        """Calculate the trimmed mean temperature for a servo over the rolling window"""
        if servo_idx not in self.servo_temp_history:
            return None
        history = self.servo_temp_history[servo_idx]
        if not history:
            return None
        values = sorted(t for _, t in history)
        n = len(values)
        trim_count = int(n * TRIM_PERCENT)
        if n - 2 * trim_count < 1:
            trim_count = max(0, (n - 1) // 2)
        trimmed = values[trim_count:n - trim_count] if trim_count > 0 else values
        if not trimmed:
            return None
        return sum(trimmed) / len(trimmed)
    
    def check(self) -> Optional[str]:
        """Check all servos for warning conditions"""
        if not self.servo_states:
            return None  # No data yet
        
        warnings = []
        
        for i, servo in enumerate(self.servo_states):
            servo_warnings = []
            
            # Check temperature using trimmed mean
            avg_temp = self._get_trimmed_mean_temp(i)
            if avg_temp is not None and avg_temp >= self.temp_warn:
                servo_warnings.append(f'temp={avg_temp:.1f}°C')
            
            # Check status code (0 = normal, any other value is an error)
            if servo.servo_status != 0:
                servo_warnings.append(f'status=0x{servo.servo_status:02X}')
            
            # Check current draw (in mA, warn if >= 3000mA = 3A)
            current_ma = servo.current_current  # Already in mA per msg definition
            if current_ma >= self.current_warn_ma:
                servo_warnings.append(f'current={current_ma:.0f}mA')
            
            if servo_warnings:
                servo_name = self._get_servo_name(i)
                warnings.append(f'{servo_name}: {", ".join(servo_warnings)}')
        
        if warnings:
            return f'{self.name} servo warnings: {"; ".join(warnings)}'
        
        return None


# ============================================================================
# Servo Names Configuration
# ============================================================================

# Servo names for GDB0 (driver0) - Right arm and head
# Indices 0-9 map to driver0/servo01-servo10
GDB0_SERVO_NAMES = [
    'R Shoulder Yaw',     # servo01
    'R Shoulder1 Pitch',  # servo02
    'R Shoulder2 Pitch',  # servo03 (derived)
    'R Elbow Pitch',      # servo04
    'R Wrist Pitch',      # servo05
    'R Wrist Roll',       # servo06
    'R Hand',             # servo07
    'Head Yaw',           # servo08
    'Head Pitch',         # servo09
    'Head Roll',          # servo10
]

# Servo names for GDB1 (driver1) - Left arm
# Indices 0-6 map to driver1/servo01-servo07
GDB1_SERVO_NAMES = [
    'L Shoulder Yaw',     # servo01
    'L Shoulder1 Pitch',  # servo02
    'L Shoulder2 Pitch',  # servo03 (derived)
    'L Elbow Pitch',      # servo04
    'L Wrist Pitch',      # servo05
    'L Wrist Roll',       # servo06
    'L Hand',             # servo07
]


# ============================================================================
# Watchdog Configuration Factory
# ============================================================================

def create_health_checks() -> Dict[str, HealthCheck]:
    """
    Create and return all configured health checks.
    
    Returns:
        Dictionary of health check name to HealthCheck instance
    """
    health_checks: Dict[str, HealthCheck] = {}
    
    # ========================================================================
    # GDB Rate Monitors
    # ========================================================================
    
    health_checks['gdb0_rate'] = RateMonitor(
        name='GDB0',
        expected_hz=100.0,
        tolerance_hz=10.0
    )
    
    health_checks['gdb1_rate'] = RateMonitor(
        name='GDB1',
        expected_hz=100.0,
        tolerance_hz=10.0
    )
    
    # ========================================================================
    # GDB Voltage Monitors
    # ========================================================================
    
    health_checks['gdb0_voltage'] = ServoVoltageMonitor(
        name='GDB0',
        servo_names=GDB0_SERVO_NAMES,
        expected_voltage=12.0,
        tolerance_voltage=0.5
    )
    
    health_checks['gdb1_voltage'] = ServoVoltageMonitor(
        name='GDB1',
        servo_names=GDB1_SERVO_NAMES,
        expected_voltage=12.0,
        tolerance_voltage=0.5
    )
    
    # ========================================================================
    # GDB Diagnostic Timing Monitors
    # ========================================================================
    
    health_checks['gdb0_timing'] = DiagnosticTimingMonitor(
        name='GDB0',
        max_total_ms=10.0
    )
    
    health_checks['gdb1_timing'] = DiagnosticTimingMonitor(
        name='GDB1',
        max_total_ms=10.0
    )
    
    # ========================================================================
    # GDB Board Temperature Monitors
    # ========================================================================
    
    health_checks['gdb0_board_temp'] = TemperatureMonitor(
        name='GDB0 Board Temp',
        warn_temp=45.0,
        critical_temp=45.0
    )
    
    health_checks['gdb1_board_temp'] = TemperatureMonitor(
        name='GDB1 Board Temp',
        warn_temp=45.0,
        critical_temp=45.0
    )
    
    # ========================================================================
    # Servo Health Monitors
    # ========================================================================
    
    health_checks['gdb0_servos'] = ServoMonitor(
        name='GDB0',
        servo_names=GDB0_SERVO_NAMES,
        temp_warn=50.0,
        current_warn_ma=3000.0
    )
    
    health_checks['gdb1_servos'] = ServoMonitor(
        name='GDB1',
        servo_names=GDB1_SERVO_NAMES,
        temp_warn=50.0,
        current_warn_ma=3000.0
    )
    
    # ========================================================================
    # Jetson Temperature Monitors
    # ========================================================================
    
    health_checks['jetson_cpu_temp'] = TemperatureMonitor(
        name='Jetson CPU Temp',
        warn_temp=70.0,
        critical_temp=85.0
    )
    
    health_checks['jetson_gpu_temp'] = TemperatureMonitor(
        name='Jetson GPU Temp',
        warn_temp=70.0,
        critical_temp=85.0
    )
    
    health_checks['jetson_thermal'] = TemperatureMonitor(
        name='Jetson Thermal Junction',
        warn_temp=85.0,
        critical_temp=95.0
    )
    
    # ========================================================================
    # Jetson Resource Monitors
    # ========================================================================
    
    health_checks['jetson_ram'] = ThresholdMonitor(
        name='Jetson RAM',
        warn_percent=85.0,
        critical_percent=95.0
    )
    
    health_checks['jetson_swap'] = ThresholdMonitor(
        name='Jetson Swap',
        warn_percent=50.0,
        critical_percent=80.0
    )
    
    health_checks['jetson_disk'] = ThresholdMonitor(
        name='Jetson Disk',
        warn_percent=80.0,
        critical_percent=95.0
    )
    
    health_checks['jetson_cpu_load'] = ThresholdMonitor(
        name='Jetson CPU Load',
        warn_percent=90.0,
        critical_percent=98.0
    )
    
    # ========================================================================
    # WiFi Monitor
    # ========================================================================
    
    health_checks['jetson_wifi'] = WifiMonitor(
        name='Jetson WiFi',
        min_signal_dbm=-75,
        require_connected=True
    )
    
    return health_checks
