"""
Jetson Stats Node

Publishes Jetson SBC statistics (CPU, GPU, memory, disk, temperatures, fan speed)
to /alfie/low/jetsonstate at 1Hz using the jtop library.

Requires: pip install jetson-stats
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from alfie_msgs.msg import JetsonState
from std_msgs.msg import Header
import os


# ============================================================================
# Constants
# ============================================================================

PUBLISH_RATE_HZ = 1.0
PUBLISH_PERIOD_SEC = 1.0 / PUBLISH_RATE_HZ


# ============================================================================
# JetsonStatsNode Class
# ============================================================================

class JetsonStatsNode(Node):
    def __init__(self):
        super().__init__('jetson_stats_node')
        
        # QoS profile for best effort communication
        qos_best_effort = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        
        # Publisher for Jetson state
        self.publisher = self.create_publisher(
            JetsonState,
            '/alfie/low/jetsonstate',
            qos_best_effort
        )
        
        # Try to import jtop
        self.jtop = None
        self.jtop_available = False
        try:
            from jtop import jtop
            self.jtop = jtop()
            self.jtop.start()
            self.jtop_available = True
            self.get_logger().info('jtop library available - using Jetson hardware stats')
        except ImportError:
            self.get_logger().warn('jtop not available - will publish simulated/fallback stats')
        except Exception as e:
            self.get_logger().warn(f'jtop failed to start: {e} - will publish fallback stats')
        
        # Timer to publish stats
        self.timer = self.create_timer(PUBLISH_PERIOD_SEC, self.publish_stats)
        
        self.get_logger().info(f'Jetson Stats Node started - publishing at {PUBLISH_RATE_HZ}Hz')
    
    def publish_stats(self) -> None:
        """Collect and publish Jetson statistics"""
        msg = JetsonState()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'jetson'
        
        if self.jtop_available and self.jtop is not None:
            self._populate_from_jtop(msg)
        else:
            self._populate_fallback(msg)
        
        # WiFi stats (works with or without jtop)
        self._populate_wifi_stats(msg)
        
        self.publisher.publish(msg)
    
    def _populate_from_jtop(self, msg: JetsonState) -> None:
        """Populate message using jtop library"""
        try:
            # ================================================================
            # Temperature readings
            # ================================================================
            temps = self.jtop.temperature
            if temps:
                # Temperature values are dicts with 'temp' key
                cpu_temp = temps.get('cpu', {})
                msg.cpu_temp = float(cpu_temp.get('temp', 0.0)) if isinstance(cpu_temp, dict) else 0.0
                
                gpu_temp = temps.get('gpu', {})
                msg.gpu_temp = float(gpu_temp.get('temp', 0.0)) if isinstance(gpu_temp, dict) else 0.0
                
                # Auxiliary/SoC temperature
                aux_temp = temps.get('soc0', temps.get('aux', {}))
                msg.aux_temp = float(aux_temp.get('temp', 0.0)) if isinstance(aux_temp, dict) else 0.0
                
                # Thermal junction temperature
                tj_temp = temps.get('tj', temps.get('thermal', {}))
                msg.thermal_temp = float(tj_temp.get('temp', 0.0)) if isinstance(tj_temp, dict) else 0.0
            
            # ================================================================
            # CPU statistics
            # ================================================================
            cpu = self.jtop.cpu
            if cpu:
                # Total CPU usage from 'total' dict
                total = cpu.get('total', {})
                if isinstance(total, dict):
                    user = total.get('user', 0.0)
                    system = total.get('system', 0.0)
                    msg.cpu_usage_percent = float(user + system)
                
                # Per-core usage from 'cpu' list
                cpu_list = cpu.get('cpu', [])
                if isinstance(cpu_list, list):
                    core_usages = []
                    for core in cpu_list:
                        if isinstance(core, dict):
                            user = core.get('user', 0.0)
                            system = core.get('system', 0.0)
                            core_usages.append(float(user + system))
                    msg.cpu_core_usage = core_usages
                    
                    # Get frequency from first core
                    if cpu_list and isinstance(cpu_list[0], dict):
                        freq_info = cpu_list[0].get('freq', {})
                        if isinstance(freq_info, dict):
                            freq_hz = freq_info.get('cur', 0)
                            msg.cpu_frequency_mhz = int(freq_hz / 1000) if freq_hz > 10000 else int(freq_hz)
            
            # ================================================================
            # GPU statistics
            # ================================================================
            gpu = self.jtop.gpu
            if gpu:
                # GPU is nested: {'gpu': {'status': {'load': ...}, 'freq': {...}}}
                gpu_info = gpu.get('gpu', {})
                if isinstance(gpu_info, dict):
                    status = gpu_info.get('status', {})
                    if isinstance(status, dict):
                        msg.gpu_usage_percent = float(status.get('load', 0.0))
                    
                    freq_info = gpu_info.get('freq', {})
                    if isinstance(freq_info, dict):
                        freq_hz = freq_info.get('cur', 0)
                        msg.gpu_frequency_mhz = int(freq_hz / 1000) if freq_hz > 10000 else int(freq_hz)
            
            # ================================================================
            # Memory statistics (values are in KB)
            # ================================================================
            memory = self.jtop.memory
            if memory:
                ram = memory.get('RAM', {})
                if isinstance(ram, dict):
                    # Values are in KB
                    msg.ram_total_mb = float(ram.get('tot', 0)) / 1024.0
                    msg.ram_used_mb = float(ram.get('used', 0)) / 1024.0
                    msg.ram_free_mb = float(ram.get('free', 0)) / 1024.0
                    if msg.ram_total_mb > 0:
                        msg.ram_usage_percent = (msg.ram_used_mb / msg.ram_total_mb) * 100.0
                
                # Swap memory (values are in KB)
                swap = memory.get('SWAP', {})
                if isinstance(swap, dict):
                    msg.swap_total_mb = float(swap.get('tot', 0)) / 1024.0
                    msg.swap_used_mb = float(swap.get('used', 0)) / 1024.0
                    msg.swap_free_mb = msg.swap_total_mb - msg.swap_used_mb
                    if msg.swap_total_mb > 0:
                        msg.swap_usage_percent = (msg.swap_used_mb / msg.swap_total_mb) * 100.0
            
            # ================================================================
            # Disk statistics (values already in GB)
            # ================================================================
            disk = self.jtop.disk
            if disk:
                msg.disk_total_gb = float(disk.get('total', 0))
                msg.disk_used_gb = float(disk.get('used', 0))
                msg.disk_free_gb = float(disk.get('available', 0))
                if msg.disk_total_gb > 0:
                    msg.disk_usage_percent = (msg.disk_used_gb / msg.disk_total_gb) * 100.0
            
            # ================================================================
            # Power statistics (values in mW)
            # ================================================================
            power = self.jtop.power
            if power:
                # Total power from 'tot' dict
                tot = power.get('tot', {})
                if isinstance(tot, dict):
                    msg.power_draw_watts = float(tot.get('power', 0)) / 1000.0
                    msg.power_avg_watts = float(tot.get('avg', 0)) / 1000.0
            
            # Power mode
            nvpmodel = self.jtop.nvpmodel
            if nvpmodel:
                msg.power_mode = str(nvpmodel.name) if hasattr(nvpmodel, 'name') else str(nvpmodel)
            
            # ================================================================
            # Fan speed
            # ================================================================
            fan = self.jtop.fan
            if fan:
                # Fan is: {'pwmfan': {'speed': [40.0], 'rpm': [2963], ...}}
                pwmfan = fan.get('pwmfan', {})
                if isinstance(pwmfan, dict):
                    speed_list = pwmfan.get('speed', [])
                    if isinstance(speed_list, list) and len(speed_list) > 0:
                        msg.fan_speed_percent = float(speed_list[0])
            
            # ================================================================
            # System uptime and load
            # ================================================================
            uptime = self.jtop.uptime
            if uptime:
                msg.uptime_seconds = float(uptime.total_seconds())
            
            # Load averages from /proc/loadavg
            try:
                with open('/proc/loadavg', 'r') as f:
                    load_data = f.read().split()
                    msg.load_avg_1min = float(load_data[0])
                    msg.load_avg_5min = float(load_data[1])
                    msg.load_avg_15min = float(load_data[2])
            except Exception:
                pass
            
            # ================================================================
            # Jetson model information
            # ================================================================
            board = self.jtop.board
            if board:
                hardware = board.get('hardware', {})
                if isinstance(hardware, dict):
                    msg.jetson_model = str(hardware.get('Module', hardware.get('BOARDIDS', 'Unknown')))
                
                platform = board.get('platform', {})
                if isinstance(platform, dict):
                    msg.jetpack_version = str(platform.get('L4T', 'Unknown'))
            
        except Exception as e:
            self.get_logger().warn(f'Error reading jtop stats: {e}')
    
    def _populate_fallback(self, msg: JetsonState) -> None:
        """Populate message using standard Linux tools as fallback"""
        try:
            # ================================================================
            # CPU temperature from thermal zones
            # ================================================================
            for i in range(10):
                temp_path = f'/sys/class/thermal/thermal_zone{i}/temp'
                if os.path.exists(temp_path):
                    with open(temp_path, 'r') as f:
                        temp_mc = int(f.read().strip())
                        temp_c = temp_mc / 1000.0
                        if i == 0:
                            msg.cpu_temp = temp_c
                        elif i == 1:
                            msg.gpu_temp = temp_c
                        break
            
            # ================================================================
            # CPU usage from /proc/stat
            # ================================================================
            try:
                import psutil
                msg.cpu_usage_percent = psutil.cpu_percent()
                msg.cpu_core_usage = [float(x) for x in psutil.cpu_percent(percpu=True)]
                
                # Memory
                mem = psutil.virtual_memory()
                msg.ram_total_mb = mem.total / (1024 * 1024)
                msg.ram_used_mb = mem.used / (1024 * 1024)
                msg.ram_free_mb = mem.available / (1024 * 1024)
                msg.ram_usage_percent = mem.percent
                
                # Swap
                swap = psutil.swap_memory()
                msg.swap_total_mb = swap.total / (1024 * 1024)
                msg.swap_used_mb = swap.used / (1024 * 1024)
                msg.swap_free_mb = swap.free / (1024 * 1024)
                msg.swap_usage_percent = swap.percent
                
                # Disk
                disk = psutil.disk_usage('/')
                msg.disk_total_gb = disk.total / (1024 * 1024 * 1024)
                msg.disk_used_gb = disk.used / (1024 * 1024 * 1024)
                msg.disk_free_gb = disk.free / (1024 * 1024 * 1024)
                msg.disk_usage_percent = disk.percent
                
            except ImportError:
                self.get_logger().warn('psutil not available for fallback stats')
            
            # ================================================================
            # Load averages
            # ================================================================
            try:
                with open('/proc/loadavg', 'r') as f:
                    load_data = f.read().split()
                    msg.load_avg_1min = float(load_data[0])
                    msg.load_avg_5min = float(load_data[1])
                    msg.load_avg_15min = float(load_data[2])
            except Exception:
                pass
            
            # ================================================================
            # Uptime
            # ================================================================
            try:
                with open('/proc/uptime', 'r') as f:
                    uptime_data = f.read().split()
                    msg.uptime_seconds = float(uptime_data[0])
            except Exception:
                pass
            
            msg.jetson_model = 'Unknown (jtop not available)'
            msg.jetpack_version = 'Unknown'
            
        except Exception as e:
            self.get_logger().warn(f'Error reading fallback stats: {e}')
    
    def _populate_wifi_stats(self, msg: JetsonState) -> None:
        """Populate WiFi statistics using iwconfig/iw commands"""
        try:
            import subprocess
            
            # Find WiFi interface
            wifi_interface = None
            for iface in ['wlan0', 'wlan1', 'wlp0s20f3', 'wifi0']:
                if os.path.exists(f'/sys/class/net/{iface}/wireless'):
                    wifi_interface = iface
                    break
            
            if wifi_interface is None:
                # Try to find any wireless interface
                try:
                    result = subprocess.run(
                        ['ls', '/sys/class/net/'],
                        capture_output=True, text=True, timeout=2
                    )
                    for iface in result.stdout.strip().split():
                        if os.path.exists(f'/sys/class/net/{iface}/wireless'):
                            wifi_interface = iface
                            break
                except Exception:
                    pass
            
            if wifi_interface is None:
                msg.wifi_connected = False
                return
            
            msg.wifi_interface = wifi_interface
            
            # Check if interface is up and connected
            try:
                with open(f'/sys/class/net/{wifi_interface}/operstate', 'r') as f:
                    operstate = f.read().strip()
                    msg.wifi_connected = (operstate == 'up')
            except Exception:
                msg.wifi_connected = False
            
            if not msg.wifi_connected:
                return
            
            # Get SSID using iwgetid
            try:
                result = subprocess.run(
                    ['iwgetid', wifi_interface, '-r'],
                    capture_output=True, text=True, timeout=2
                )
                if result.returncode == 0:
                    msg.wifi_ssid = result.stdout.strip()
            except Exception:
                pass
            
            # Get signal strength and link speed using iw
            try:
                result = subprocess.run(
                    ['iw', 'dev', wifi_interface, 'link'],
                    capture_output=True, text=True, timeout=2
                )
                if result.returncode == 0:
                    output = result.stdout
                    
                    # Parse signal strength (e.g., "signal: -45 dBm")
                    for line in output.split('\n'):
                        line = line.strip()
                        if line.startswith('signal:'):
                            try:
                                # Extract dBm value
                                parts = line.split()
                                dbm = int(parts[1])
                                msg.wifi_signal_dbm = dbm
                                # Convert dBm to percentage (rough approximation)
                                # -30 dBm = 100%, -90 dBm = 0%
                                percent = max(0, min(100, int((dbm + 90) * 100 / 60)))
                                msg.wifi_signal_percent = percent
                            except (ValueError, IndexError):
                                pass
                        elif line.startswith('tx bitrate:'):
                            try:
                                # Extract link speed (e.g., "tx bitrate: 866.7 MBit/s")
                                parts = line.split()
                                speed = float(parts[2])
                                msg.wifi_link_speed_mbps = int(speed)
                            except (ValueError, IndexError):
                                pass
            except Exception:
                pass
            
            # Fallback: try iwconfig if iw doesn't work
            if msg.wifi_signal_dbm == 0:
                try:
                    result = subprocess.run(
                        ['iwconfig', wifi_interface],
                        capture_output=True, text=True, timeout=2
                    )
                    if result.returncode == 0:
                        output = result.stdout
                        
                        # Parse "Link Quality=70/70  Signal level=-40 dBm"
                        for line in output.split('\n'):
                            if 'Signal level' in line:
                                try:
                                    # Find signal level
                                    idx = line.find('Signal level=')
                                    if idx >= 0:
                                        signal_str = line[idx+13:].split()[0]
                                        msg.wifi_signal_dbm = int(signal_str)
                                        percent = max(0, min(100, int((msg.wifi_signal_dbm + 90) * 100 / 60)))
                                        msg.wifi_signal_percent = percent
                                except (ValueError, IndexError):
                                    pass
                            if 'Bit Rate' in line or 'Bit Rate:' in line:
                                try:
                                    idx = line.find('Bit Rate')
                                    if idx >= 0:
                                        # Extract number after "Bit Rate="
                                        rate_part = line[idx:].split()[0]
                                        rate_str = rate_part.split('=')[1] if '=' in rate_part else rate_part.split(':')[1]
                                        msg.wifi_link_speed_mbps = int(float(rate_str))
                                except (ValueError, IndexError):
                                    pass
                except Exception:
                    pass
            
        except Exception as e:
            self.get_logger().debug(f'Error reading WiFi stats: {e}')
    
    def destroy_node(self) -> None:
        """Clean up jtop on shutdown"""
        if self.jtop is not None and self.jtop_available:
            try:
                self.jtop.close()
            except Exception:
                pass
        super().destroy_node()


# ============================================================================
# Main Entry Point
# ============================================================================

def main(args=None):
    rclpy.init(args=args)
    node = JetsonStatsNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
