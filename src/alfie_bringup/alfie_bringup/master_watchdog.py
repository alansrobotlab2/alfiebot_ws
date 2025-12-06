"""
Master Watchdog Node

Monitors the health of critical subsystems by subscribing to state topics
and tracking message reception timing and rates.
"""

import rclpy
from rclpy.node import Node
from alfie_msgs.msg import GDBState, JetsonState
from rclpy.qos import QoSProfile, ReliabilityPolicy
from typing import Optional, Dict, Callable
from dataclasses import dataclass, field
import time
from abc import ABC, abstractmethod


# ============================================================================
# Constants and Configuration
# ============================================================================

# Watchdog check rate
WATCHDOG_RATE_HZ = 1.0  # Check once per second
WATCHDOG_PERIOD_SEC = 1.0 / WATCHDOG_RATE_HZ


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
# Voltage Monitor Health Check
# ============================================================================

# Voltage is reported in 0.1V units
VOLTAGE_SCALE = 10.0


class VoltageMonitor(HealthCheck):
    """
    Monitors servo voltage levels.
    Reports error if voltage deviates from expected by more than tolerance.
    """
    
    def __init__(self, name: str, expected_voltage: float, tolerance_voltage: float):
        super().__init__(name)
        self.expected_voltage = expected_voltage
        self.tolerance_voltage = tolerance_voltage
        self.current_voltage: Optional[float] = None
    
    def update(self, raw_voltage: int) -> None:
        """Called when a message is received with raw voltage value (0.1V units)"""
        self.current_voltage = raw_voltage / VOLTAGE_SCALE
    
    def check(self) -> Optional[str]:
        """Check if voltage is within tolerance"""
        if self.current_voltage is None:
            return None  # No data yet
        
        min_voltage = self.expected_voltage - self.tolerance_voltage
        max_voltage = self.expected_voltage + self.tolerance_voltage
        
        if self.current_voltage < min_voltage:
            return f'{self.name} voltage too low: {self.current_voltage:.1f}V (expected {self.expected_voltage}V ± {self.tolerance_voltage}V)'
        elif self.current_voltage > max_voltage:
            return f'{self.name} voltage too high: {self.current_voltage:.1f}V (expected {self.expected_voltage}V ± {self.tolerance_voltage}V)'
        
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
        imu_duration_ms = self.current_timings.get('imuupdateduration', 0.0)
        if imu_duration_ms > self.IMU_MAX_MS:
            errors.append(f'{self.name} IMU update duration {imu_duration_ms:.2f}ms exceeded {self.IMU_MAX_MS}ms')
        
        if errors:
            return '; '.join(errors)
        
        return None


# ============================================================================
# Temperature Monitor Health Check
# ============================================================================

class TemperatureMonitor(HealthCheck):
    """
    Monitors temperature values.
    Reports warning if above warn threshold, error if above critical threshold.
    """
    
    def __init__(self, name: str, warn_temp: float, critical_temp: float):
        super().__init__(name)
        self.warn_temp = warn_temp
        self.critical_temp = critical_temp
        self.current_temp: Optional[float] = None
    
    def update(self, temp: float) -> None:
        """Called when a new temperature reading is available"""
        self.current_temp = temp
    
    def check(self) -> Optional[str]:
        """Check if temperature is within safe limits"""
        if self.current_temp is None or self.current_temp <= 0:
            return None  # No data yet
        
        if self.current_temp >= self.critical_temp:
            return f'{self.name} CRITICAL: {self.current_temp:.1f}°C >= {self.critical_temp}°C'
        elif self.current_temp >= self.warn_temp:
            return f'{self.name} WARNING: {self.current_temp:.1f}°C >= {self.warn_temp}°C'
        
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


@dataclass
class ServoAlert:
    """Holds alert thresholds for servo monitoring"""
    servo_index: int
    temp_warn: float = 40.0           # Temperature warning threshold (°C)
    current_warn_ma: float = 3000.0   # Current warning threshold (mA = 3A)


class ServoMonitor(HealthCheck):
    """
    Monitors servo health for a GDB (servo driver board).
    Reports warnings for temperature, status codes, and current draw.
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
    
    def update(self, servo_states: list) -> None:
        """Called when servo states are received"""
        self.servo_states = servo_states
    
    def _get_servo_name(self, index: int) -> str:
        """Get human-readable servo name for the given index"""
        if index < len(self.servo_names):
            return self.servo_names[index]
        return f'Servo[{index}]'
    
    def check(self) -> Optional[str]:
        """Check all servos for warning conditions"""
        if not self.servo_states:
            return None  # No data yet
        
        warnings = []
        
        for i, servo in enumerate(self.servo_states):
            servo_warnings = []
            
            # Check temperature (servo reports in °C as uint8)
            if servo.current_temperature >= self.temp_warn:
                servo_warnings.append(f'temp={servo.current_temperature}°C')
            
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
# MasterWatchdogNode Class
# ============================================================================

class MasterWatchdogNode(Node):
    def __init__(self):
        super().__init__('master_watchdog_node')
        
        # Use BEST_EFFORT QoS for gdb state subscriptions (to match gdb publishers)
        qos_best_effort = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        
        # ====================================================================
        # Health Checks Registry
        # ====================================================================
        self.health_checks: Dict[str, HealthCheck] = {}
        
        # GDB0 rate monitor
        self.health_checks['gdb0_rate'] = RateMonitor(
            name='GDB0',
            expected_hz=100.0,
            tolerance_hz=10.0
        )
        
        # GDB1 rate monitor
        self.health_checks['gdb1_rate'] = RateMonitor(
            name='GDB1',
            expected_hz=100.0,
            tolerance_hz=10.0
        )
        
        # GDB0 voltage monitor (from first servo)
        self.health_checks['gdb0_voltage'] = VoltageMonitor(
            name='GDB0',
            expected_voltage=12.0,
            tolerance_voltage=0.2
        )
        
        # GDB1 voltage monitor (from first servo)
        self.health_checks['gdb1_voltage'] = VoltageMonitor(
            name='GDB1',
            expected_voltage=12.0,
            tolerance_voltage=0.2
        )
        
        # GDB0 diagnostic timing monitor
        self.health_checks['gdb0_timing'] = DiagnosticTimingMonitor(
            name='GDB0',
            max_total_ms=10.0
        )
        
        # GDB1 diagnostic timing monitor
        self.health_checks['gdb1_timing'] = DiagnosticTimingMonitor(
            name='GDB1',
            max_total_ms=10.0
        )
        
        # GDB0 board temperature monitor
        self.health_checks['gdb0_board_temp'] = TemperatureMonitor(
            name='GDB0 Board Temp',
            warn_temp=45.0,
            critical_temp=45.0  # Same as warn - trigger immediately
        )
        
        # GDB1 board temperature monitor
        self.health_checks['gdb1_board_temp'] = TemperatureMonitor(
            name='GDB1 Board Temp',
            warn_temp=45.0,
            critical_temp=45.0  # Same as warn - trigger immediately
        )
        
        # ====================================================================
        # Servo Health Checks
        # ====================================================================
        
        # GDB0 servo monitor - Right arm + Head (temp >= 50°C, status != 0, current >= 3A)
        self.health_checks['gdb0_servos'] = ServoMonitor(
            name='GDB0',
            servo_names=GDB0_SERVO_NAMES,
            temp_warn=50.0,
            current_warn_ma=3000.0
        )
        
        # GDB1 servo monitor - Left arm (temp >= 50°C, status != 0, current >= 3A)
        self.health_checks['gdb1_servos'] = ServoMonitor(
            name='GDB1',
            servo_names=GDB1_SERVO_NAMES,
            temp_warn=50.0,
            current_warn_ma=3000.0
        )
        
        # ====================================================================
        # Jetson Health Checks
        # ====================================================================
        
        # Temperature monitors (warn at 70°C, critical at 85°C)
        self.health_checks['jetson_cpu_temp'] = TemperatureMonitor(
            name='Jetson CPU Temp',
            warn_temp=70.0,
            critical_temp=85.0
        )
        
        self.health_checks['jetson_gpu_temp'] = TemperatureMonitor(
            name='Jetson GPU Temp',
            warn_temp=70.0,
            critical_temp=85.0
        )
        
        self.health_checks['jetson_thermal'] = TemperatureMonitor(
            name='Jetson Thermal Junction',
            warn_temp=85.0,
            critical_temp=95.0
        )
        
        # Memory monitors (warn at 85%, critical at 95%)
        self.health_checks['jetson_ram'] = ThresholdMonitor(
            name='Jetson RAM',
            warn_percent=85.0,
            critical_percent=95.0
        )
        
        self.health_checks['jetson_swap'] = ThresholdMonitor(
            name='Jetson Swap',
            warn_percent=50.0,
            critical_percent=80.0
        )
        
        # Disk monitor (warn at 80%, critical at 95%)
        self.health_checks['jetson_disk'] = ThresholdMonitor(
            name='Jetson Disk',
            warn_percent=80.0,
            critical_percent=95.0
        )
        
        # CPU load monitor (warn at 90%, critical at 98%)
        self.health_checks['jetson_cpu_load'] = ThresholdMonitor(
            name='Jetson CPU Load',
            warn_percent=90.0,
            critical_percent=98.0
        )
        
        # WiFi monitor (require connection, warn below -75dBm)
        self.health_checks['jetson_wifi'] = WifiMonitor(
            name='Jetson WiFi',
            min_signal_dbm=-75,
            require_connected=True
        )
        
        # ====================================================================
        # Subscriptions
        # ====================================================================
        self.gdb0_sub = self.create_subscription(
            GDBState,
            '/alfie/low/gdb0state',
            self.gdb0_callback,
            qos_best_effort
        )
        
        self.gdb1_sub = self.create_subscription(
            GDBState,
            '/alfie/low/gdb1state',
            self.gdb1_callback,
            qos_best_effort
        )
        
        self.jetson_sub = self.create_subscription(
            JetsonState,
            '/alfie/low/jetsonstate',
            self.jetson_callback,
            10  # Use default RELIABLE QoS for Jetson stats
        )
        
        # ====================================================================
        # Watchdog Timer
        # ====================================================================
        self.watchdog_timer = self.create_timer(WATCHDOG_PERIOD_SEC, self.run_health_checks)
        
        self.get_logger().info(f'Master Watchdog Node started - checking at {WATCHDOG_RATE_HZ}Hz')
    
    # ========================================================================
    # Callback Methods
    # ========================================================================
    
    def gdb0_callback(self, msg: GDBState) -> None:
        """Callback for gdb0state"""
        self.health_checks['gdb0_rate'].update()
        # Update voltage from first servo
        if len(msg.servo_state) > 0:
            self.health_checks['gdb0_voltage'].update(msg.servo_state[0].current_voltage)
        # Update diagnostic timings
        self.health_checks['gdb0_timing'].update(msg.driver_diagnostics)
        # Update servo health monitor
        self.health_checks['gdb0_servos'].update(msg.servo_state)
        # Update board temperature
        self.health_checks['gdb0_board_temp'].update(float(msg.board_temp))
    
    def gdb1_callback(self, msg: GDBState) -> None:
        """Callback for gdb1state"""
        self.health_checks['gdb1_rate'].update()
        # Update voltage from first servo
        if len(msg.servo_state) > 0:
            self.health_checks['gdb1_voltage'].update(msg.servo_state[0].current_voltage)
        # Update diagnostic timings
        self.health_checks['gdb1_timing'].update(msg.driver_diagnostics)
        # Update servo health monitor
        self.health_checks['gdb1_servos'].update(msg.servo_state)
        # Update board temperature
        self.health_checks['gdb1_board_temp'].update(float(msg.board_temp))
    
    def jetson_callback(self, msg: JetsonState) -> None:
        """Callback for jetsonstate"""
        # Temperature monitors
        self.health_checks['jetson_cpu_temp'].update(msg.cpu_temp)
        self.health_checks['jetson_gpu_temp'].update(msg.gpu_temp)
        self.health_checks['jetson_thermal'].update(msg.thermal_temp)
        
        # Memory/resource monitors
        self.health_checks['jetson_ram'].update(msg.ram_usage_percent)
        self.health_checks['jetson_swap'].update(msg.swap_usage_percent)
        self.health_checks['jetson_disk'].update(msg.disk_usage_percent)
        self.health_checks['jetson_cpu_load'].update(msg.cpu_usage_percent)
        
        # WiFi monitor
        self.health_checks['jetson_wifi'].update(
            connected=msg.wifi_connected,
            signal_dbm=msg.wifi_signal_dbm,
            ssid=msg.wifi_ssid
        )
    
    # ========================================================================
    # Health Check Runner
    # ========================================================================
    
    def run_health_checks(self) -> None:
        """Run all registered health checks"""
        for check_name, check in self.health_checks.items():
            error_msg = check.check()
            if error_msg:
                self.get_logger().error(error_msg)


# ============================================================================
# Main Entry Point
# ============================================================================

def main(args=None):
    rclpy.init(args=args)
    node = MasterWatchdogNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

