# GR00T N1.6 Inference Client

This document explains the GR00T N1.6 inference client for Alfiebot, which enables real-time policy execution using a client-server architecture.

## Timing

| Loop | Rate | Purpose |
|------|------|---------|
| Inference | 15 FPS | Get new actions from server |
| Command | **100 Hz** | Publish RobotLowCmd to robot |
| Status | 1 Hz | Publish status information |

The command loop runs at exactly 100 Hz to match robot expectations, holding/republishing the last received action between inference updates.

## Architecture Overview

```
┌─────────────────────────────────────────────────────────────────────────────┐
│                         Jetson Orin NX (Client)                             │
│  ┌─────────────────┐    ┌─────────────────┐    ┌─────────────────┐         │
│  │  Cameras (4x)   │───▶│ ObservationBridge│───▶│                 │         │
│  │  CompressedImage│    │ - Sync & resize │    │                 │         │
│  └─────────────────┘    │ - Extract state │    │   ZMQClient     │         │
│                         └─────────────────┘    │   (15 FPS)      │         │
│  ┌─────────────────┐                           │  - msgpack      │◀───┐    │
│  │  RobotLowState  │───────────────────────────│  - REQ/REP      │    │    │
│  │  (15 servos)    │                           │                 │    │    │
│  └─────────────────┘                           └────────┬────────┘    │    │
│                                                         │             │    │
│                                                    ZeroMQ REQ         │    │
│                                                         │             │    │
└─────────────────────────────────────────────────────────┼─────────────┼────┘
                                                          │             │
                                                          ▼             │
┌─────────────────────────────────────────────────────────┴─────────────┴────┐
│                            PC (Server)                                      │
│  ┌─────────────────────────────────────────────────────────────────────┐   │
│  │                    GR00T Inference Server                            │   │
│  │  - Receives observations (4 images + 21D state + language)          │   │
│  │  - Runs TensorRT inference                                          │   │
│  │  - Returns 16-step action horizon (16 x 21D)                        │   │
│  └─────────────────────────────────────────────────────────────────────┘   │
└─────────────────────────────────────────────────────────────────────────────┘
                                                          │
                                                     ZeroMQ REP
                                                          │
┌─────────────────────────────────────────────────────────┼──────────────────┐
│                         Jetson (continued)              │                  │
│                                                         ▼                  │
│  ┌─────────────────┐    ┌─────────────────┐    ┌─────────────────┐        │
│  │  RobotLowCmd    │◀───│ ActionPublisher │◀───│   ZMQClient     │        │
│  │  @ 100 Hz       │    │ - Denormalize   │    │   (response)    │        │
│  │  (15 servos +   │    │ - Smooth & limit│    │                 │        │
│  │   cmd_vel)      │    │ - 100 Hz publish│    │                 │        │
│  └─────────────────┘    └─────────────────┘    └─────────────────┘        │
└────────────────────────────────────────────────────────────────────────────┘
```

## Components

### 1. ObservationBridge (`core/observation_bridge.py`)

Collects and synchronizes sensor data from ROS2 topics:

| Topic | Message Type | Description |
|-------|-------------|-------------|
| `/alfie/stereo_camera/left_wide/image_raw/compressed` | CompressedImage | Left wide camera |
| `/alfie/stereo_camera/right_wide/image_raw/compressed` | CompressedImage | Right wide camera |
| `/alfie/stereo_camera/left_center/image_raw/compressed` | CompressedImage | Left center camera |
| `/alfie/stereo_camera/right_center/image_raw/compressed` | CompressedImage | Right center camera |
| `/alfie/robotlowstate` | RobotLowState | Robot state (servos, velocity, IMU) |

**Processing:**
- Uses `ApproximateTimeSynchronizer` with 50ms slop
- Decompresses JPEG images, resizes to 320x280
- Extracts 21D state vector from RobotLowState

### 2. State Vector (21 dimensions)

```
Index   Description                     Source
------  ----------------------------    --------------------------------
[0]     Base linear velocity X          robotlowstate.current_cmd_vel.linear.x
[1]     Base linear velocity Y          robotlowstate.current_cmd_vel.linear.y
[2]     Base linear velocity Z          robotlowstate.current_cmd_vel.linear.z
[3]     Base angular velocity X         robotlowstate.current_cmd_vel.angular.x
[4]     Base angular velocity Y         robotlowstate.current_cmd_vel.angular.y
[5]     Base angular velocity Z         robotlowstate.current_cmd_vel.angular.z
[6]     Left shoulder yaw               robotlowstate.servo_state[0].current_location
[7]     Left shoulder pitch             robotlowstate.servo_state[1].current_location
[8]     Left elbow pitch                robotlowstate.servo_state[2].current_location
[9]     Left wrist pitch                robotlowstate.servo_state[3].current_location
[10]    Left wrist roll                 robotlowstate.servo_state[4].current_location
[11]    Left gripper                    robotlowstate.servo_state[5].current_location
[12]    Right shoulder yaw              robotlowstate.servo_state[6].current_location
[13]    Right shoulder pitch            robotlowstate.servo_state[7].current_location
[14]    Right elbow pitch               robotlowstate.servo_state[8].current_location
[15]    Right wrist pitch               robotlowstate.servo_state[9].current_location
[16]    Right wrist roll                robotlowstate.servo_state[10].current_location
[17]    Right gripper                   robotlowstate.servo_state[11].current_location
[18]    Head yaw                        robotlowstate.servo_state[12].current_location
[19]    Head pitch                      robotlowstate.servo_state[13].current_location
[20]    Head roll                       robotlowstate.servo_state[14].current_location
```

### 3. ZMQClient (`core/zmq_client.py`)

Handles communication with the inference server using ZeroMQ REQ/REP pattern.

**Observation Message (Client → Server):**
```python
{
    "timestamp": 1704307200.123,           # Unix timestamp
    "frame_id": 42,                        # Monotonic frame counter
    "images": {
        "left_wide": b"...",               # JPEG bytes, 320x280
        "right_wide": b"...",
        "left_center": b"...",
        "right_center": b"...",
    },
    "state": [0.0, 0.0, ...],              # 21D normalized state
    "language": "find the can and pick it up"
}
```

**Action Response (Server → Client):**
```python
{
    "actions": [[...], [...], ...],        # 16 x 21D action horizon
    "inference_time_ms": 25.3,
    "status": "ok"
}
```

**Serialization:** msgpack with `use_bin_type=True`

### 4. ActionPublisher (`core/action_publisher.py`)

Converts action predictions to RobotLowCmd messages:

| Action Index | RobotLowCmd Field |
|--------------|-------------------|
| [0] | cmd_vel.linear.x |
| [1] | cmd_vel.linear.y |
| [2] | cmd_vel.linear.z |
| [3] | cmd_vel.angular.x |
| [4] | cmd_vel.angular.y |
| [5] | cmd_vel.angular.z |
| [6-11] | servo_cmd[0-5].target_location (left arm) |
| [12-17] | servo_cmd[6-11].target_location (right arm) |
| [18-20] | servo_cmd[12-14].target_location (head) |

**Processing:**
1. Denormalize using training statistics
2. Apply EMA smoothing (alpha=0.7)
3. Apply safety limits (joint + velocity)
4. Publish to `/alfie/robotlowcmd`

### 5. SafetyMonitor (`utils/safety.py`)

Enforces safety limits:

| Limit Type | Value |
|-----------|-------|
| Max linear velocity | 0.15 m/s |
| Max angular velocity | 0.5 rad/s |
| Watchdog timeout | 0.5 seconds |
| Max consecutive failures | 5 |

**Joint Limits (radians):**
```
Servo  Name                  Min      Max
-----  -------------------   ------   ------
0      left_shoulder_yaw     -1.5     1.5
1      left_shoulder_pitch   -1.0     1.5
2      left_elbow_pitch      -1.6     1.6
3      left_wrist_pitch      -1.6     1.6
4      left_wrist_roll       -1.6     1.6
5      left_gripper           0.0     0.8
6      right_shoulder_yaw    -1.5     1.5
7      right_shoulder_pitch  -1.5     1.0
8      right_elbow_pitch     -1.6     1.6
9      right_wrist_pitch     -1.6     2.7
10     right_wrist_roll      -1.6     0.8
11     right_gripper          0.0     0.8
12     head_yaw              -1.0     0.5
13     head_pitch            -1.1     0.2
14     head_roll             -0.2     0.1
```

### 6. Normalizer (`core/normalization.py`)

Handles state/action normalization using training statistics from:
```
/home/alfie/alfiebot_ws/data/alfiebot.CanDoChallenge/meta/stats.jsonl
```

## Running the Client

### Prerequisites

1. **Install Python dependencies:**
```bash
pip install pyzmq msgpack numpy opencv-python
```

2. **Build the package:**
```bash
cd ~/alfiebot_ws
colcon build --packages-select alfie_gr00t --symlink-install
source install/setup.bash
```

3. **Ensure the inference server is running.**

### Transport Modes

| Mode | Address Format | Latency | Use Case |
|------|---------------|---------|----------|
| **IPC** | `ipc:///tmp/groot_inference.sock` | ~1ms | On-device inference (default) |
| **TCP** | `tcp://192.168.1.100:5555` | ~2-5ms | Remote PC server |

IPC (Inter-Process Communication) uses Unix domain sockets and is ~5x faster than TCP localhost. Use IPC when the inference server runs on the same Jetson as the client.

### Launch Methods

**On-device inference (IPC - default, fastest):**
```bash
ros2 launch alfie_gr00t groot_inference.launch.py
```

**Remote inference (TCP):**
```bash
ros2 launch alfie_gr00t groot_inference.launch.py \
    transport:=tcp \
    server_host:=192.168.1.100 \
    server_port:=5555
```

**Custom task description:**
```bash
ros2 launch alfie_gr00t groot_inference.launch.py \
    task_description:="pick up the red can"
```

**Running the node directly:**
```bash
# IPC (on-device)
ros2 run alfie_gr00t groot_client --ros-args \
    -p transport:=ipc \
    -p task_description:="find the can and pick it up"

# TCP (remote)
ros2 run alfie_gr00t groot_client --ros-args \
    -p transport:=tcp \
    -p server_host:=192.168.1.100 \
    -p server_port:=5555
```

### Control Topics

| Topic | Type | Description |
|-------|------|-------------|
| `/alfie/groot_client/activate` | std_msgs/Bool | Activate/deactivate inference |
| `/alfie/groot_client/estop` | std_msgs/Bool | Emergency stop |
| `/alfie/groot_client/task_description` | std_msgs/String | Update task dynamically |
| `/alfie/groot_client/status` | std_msgs/String | Status information |

### Activating the Client

The client starts in IDLE state. To activate:

```bash
# Activate inference
ros2 topic pub --once /alfie/groot_client/activate std_msgs/Bool "data: true"

# Deactivate
ros2 topic pub --once /alfie/groot_client/activate std_msgs/Bool "data: false"

# Emergency stop
ros2 topic pub --once /alfie/groot_client/estop std_msgs/Bool "data: true"

# Update task
ros2 topic pub --once /alfie/groot_client/task_description std_msgs/String "data: 'pick up the blue can'"
```

### Configuration

Edit `config/groot_client.yaml`:

```yaml
groot_client:
  ros__parameters:
    # Transport: "ipc" for on-device, "tcp" for remote
    transport: "ipc"
    server_host: "192.168.1.100"  # Only for TCP
    server_port: 5555              # Only for TCP
    ipc_path: "/tmp/groot_inference.sock"  # Only for IPC

    # Inference
    target_fps: 15
    inference_timeout_ms: 100
    task_description: "find the can and pick it up"
    action_execution_index: 0

    # Safety
    enable_safety_limits: true
    action_smoothing_alpha: 0.7

    # Data
    stats_file: "/home/alfie/alfiebot_ws/data/alfiebot.CanDoChallenge/meta/stats.jsonl"
```

## State Machine

```
         ┌──────────┐
         │   IDLE   │◀──────────────────────────────┐
         └────┬─────┘                               │
              │ /activate (true)                    │
              ▼                                     │
         ┌──────────┐                               │
         │CONNECTING│                               │
         └────┬─────┘                               │
              │                                     │
     ┌────────┴────────┐                            │
     │ success    fail │                            │
     ▼                 ▼                            │
┌──────────┐     ┌──────────┐                       │
│  ACTIVE  │     │  ERROR   │───────────────────────┤
└────┬─────┘     └──────────┘                       │
     │                                              │
     │ /estop (true)                                │
     ▼                                              │
┌──────────┐                                        │
│  E_STOP  │────────────────────────────────────────┘
└──────────┘  /estop (false)
```

## Inference Server

The client expects a ZeroMQ REP server that:

1. Binds to the appropriate address:
   - **IPC (on-device):** `ipc:///tmp/groot_inference.sock`
   - **TCP (remote):** `tcp://*:5555`
2. Receives msgpack-encoded observations
3. Runs GR00T inference
4. Returns msgpack-encoded action response

**Example server skeleton:**
```python
import zmq
import msgpack
import numpy as np

# Choose transport based on deployment
USE_IPC = True  # Set False for remote TCP

context = zmq.Context()
socket = context.socket(zmq.REP)

if USE_IPC:
    # On-device: Use Unix domain socket (faster)
    socket.bind("ipc:///tmp/groot_inference.sock")
else:
    # Remote: Use TCP
    socket.bind("tcp://*:5555")

while True:
    # Receive observation
    data = socket.recv()
    obs = msgpack.unpackb(data, raw=False)

    # Extract data
    images = obs['images']      # dict of JPEG bytes
    state = obs['state']        # 21D normalized state
    language = obs['language']  # task description

    # Run inference (your GR00T model here)
    actions = run_groot_inference(images, state, language)

    # Send response
    response = {
        'actions': actions,  # 16 x 21D
        'inference_time_ms': 25.0,
        'status': 'ok'
    }
    socket.send(msgpack.packb(response, use_bin_type=True))
```

## Troubleshooting

### Connection Issues
```bash
# Test ZMQ connection
python3 -c "import zmq; ctx=zmq.Context(); s=ctx.socket(zmq.REQ); s.connect('tcp://192.168.1.100:5555'); print('Connected')"
```

### No Observations
```bash
# Check camera topics
ros2 topic hz /alfie/stereo_camera/left_wide/image_raw/compressed

# Check robot state
ros2 topic echo /alfie/robotlowstate --once
```

### Check Status
```bash
ros2 topic echo /alfie/groot_client/status
```

## File Structure

```
alfie_gr00t/
├── alfie_gr00t/
│   ├── core/
│   │   ├── __init__.py
│   │   ├── normalization.py      # State/action normalization
│   │   ├── zmq_client.py         # ZeroMQ communication
│   │   ├── observation_bridge.py # ROS2 sensor collection
│   │   └── action_publisher.py   # Action publishing
│   ├── utils/
│   │   ├── __init__.py
│   │   └── safety.py             # Safety limits
│   └── nodes/
│       └── groot_client.py       # Main ROS2 node
├── config/
│   └── groot_client.yaml         # Configuration
├── launch/
│   └── groot_inference.launch.py # Launch file
└── GROOT_CLIENT.md               # This document
```
