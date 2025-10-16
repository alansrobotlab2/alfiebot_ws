# JetsonState.msg
# Message type for capturing Jetson SBC state information

# Header with timestamp
std_msgs/Header header

# Temperature readings (degrees Celsius)
float32 cpu_temp
float32 gpu_temp
float32 aux_temp          # Auxiliary temperature sensor
float32 thermal_temp      # Thermal zone temperature

# CPU statistics
float32 cpu_usage_percent      # Overall CPU usage (0-100)
float32[] cpu_core_usage       # Per-core CPU usage (0-100)
uint32 cpu_frequency_mhz       # Current CPU frequency in MHz

# GPU statistics
float32 gpu_usage_percent      # GPU usage (0-100)
uint32 gpu_frequency_mhz       # Current GPU frequency in MHz

# Memory statistics (in MB)
float32 ram_total_mb
float32 ram_used_mb
float32 ram_free_mb
float32 ram_usage_percent      # RAM usage percentage (0-100)

# Swap memory (in MB)
float32 swap_total_mb
float32 swap_used_mb
float32 swap_free_mb
float32 swap_usage_percent     # Swap usage percentage (0-100)

# Disk/Storage statistics (in GB)
float32 disk_total_gb
float32 disk_used_gb
float32 disk_free_gb
float32 disk_usage_percent     # Disk usage percentage (0-100)

# Power statistics
float32 power_draw_watts       # Current power consumption in Watts
float32 power_avg_watts        # Average power consumption
string power_mode              # Current power mode (e.g., MAXN, 5W, 10W, 15W)

# Network statistics (optional)
float32 network_rx_mbps        # Network receive rate in Mbps
float32 network_tx_mbps        # Network transmit rate in Mbps

# System uptime and load
float32 uptime_seconds         # System uptime in seconds
float32 load_avg_1min          # 1-minute load average
float32 load_avg_5min          # 5-minute load average
float32 load_avg_15min         # 15-minute load average

# Fan speed (if applicable, 0-100%)
float32 fan_speed_percent

# Jetson model information
string jetson_model            # e.g., "Jetson Nano", "Jetson Orin Nano", etc.
string jetpack_version         # JetPack version string
