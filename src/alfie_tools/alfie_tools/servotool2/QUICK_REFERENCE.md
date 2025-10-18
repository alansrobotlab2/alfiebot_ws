# ServoTool2 Quick Reference Card

## 🚀 Launch Commands

```bash
# Method 1: Direct ROS2
ros2 run alfie_tools servotool2

# Method 2: Launch script
~/alfiebot_ws/src/alfie_tools/alfie_tools/servotool2/launch_servotool2.sh
```

## 🌐 Access URLs

**Local:** http://localhost:7860  
**Network:** http://$(hostname -I | awk '{print $1}'):7860

## 📱 Interface Tabs

| Tab | Purpose | Key Features |
|-----|---------|-------------|
| ⚙️ **EPROM** | Persistent config | Write all, Lock toggle |
| 🎮 **SRAM** | Real-time control | Position slider, Torque enable |
| 📊 **Status** | Live monitoring | Auto-refresh every 1s |
| 🗺️ **Memory Map** | Raw memory view | Complete address space |

## ⌨️ Key Operations

### Reading Servo Data
1. Select servo from dropdown
2. Click "🔄 Refresh Data"
3. View data in current tab

### Writing EPROM
1. Unlock EPROM (uncheck lock box)
2. Modify parameters
3. Click "💾 Write All EPROM Settings"

### Controlling Position
1. Go to SRAM tab
2. Check "⚡ Enable Torque"
3. Move position slider
4. Servo responds in real-time

### Quick Actions
- **0️⃣ Zero Servo**: Move to position 0
- **📍 Set as Min**: Use current position as min angle
- **📍 Set as Max**: Use current position as max angle

## 🔧 Common Parameters

| Parameter | Address | Range | Default | Unit |
|-----------|---------|-------|---------|------|
| Servo ID | 5 | 0-253 | 1 | - |
| Min Angle | 9 | -32766 to 32767 | -4095 | steps |
| Max Angle | 11 | -32766 to 32767 | 4095 | steps |
| Max Temp | 13 | 0-100 | 85 | °C |
| Max Torque | 16 | 0-1000 | 1000 | 1.0% |
| P Gain | 21 | 0-254 | 32 | - |
| D Gain | 22 | 0-254 | 32 | - |
| I Gain | 23 | 0-254 | 32 | - |
| Target Pos | 42 | -30719 to 30719 | 0 | steps |
| Torque Switch | 40 | 0-1 | 0 | bool |

## 🛡️ Safety Notes

⚠️ **EPROM Lock**: Always enable lock when not actively configuring  
⚠️ **Torque Control**: Disable torque before manual manipulation  
⚠️ **Range Limits**: Set appropriate min/max angles to prevent collisions  
⚠️ **Temperature**: Monitor temperature in Status tab  
⚠️ **Current Draw**: Watch current to detect binding/overload

## 🐛 Quick Troubleshooting

| Problem | Solution |
|---------|----------|
| Services not available | `ros2 launch alfie_bringup servos.launch.py` |
| Can't connect | Check port 7860 not in use, check firewall |
| No status updates | Verify driver state topics publishing |
| Write failed | Check servo connection and power |
| Port conflict | Kill existing: `pkill -f servotool2` |

## 📊 Status Icons

- ✅ Success
- ❌ Error/Failed
- ⚠️ Warning
- 🔒 Locked
- 🔓 Unlocked
- ⚡ Torque ON
- 🏃 Moving
- 🛑 Stopped

## 🔢 Useful Conversions

**Steps to Degrees:**  
1 step = 360° / 4096 ≈ 0.088°

**Voltage Reading:**  
Value × 0.1V (e.g., 120 = 12.0V)

**Current Reading:**  
Value × 6.5mA (e.g., 100 = 650mA)

**Speed:**  
50 step/s ≈ 0.732 RPM

## 📞 Support

**Documentation:** `~/alfiebot_ws/src/alfie_tools/alfie_tools/servotool2/README.md`  
**Implementation Details:** `IMPLEMENTATION_SUMMARY.md`  
**ROS2 Logs:** `ros2 node info /servotool2`

---
**ServoTool2 v1.0.0** | Web Interface for ST3215 Servos | Alfie Robot Project
