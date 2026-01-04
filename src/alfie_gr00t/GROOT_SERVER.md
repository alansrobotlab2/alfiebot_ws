# GR00T N1.6 Inference Server

This document explains the GR00T N1.6 inference server for Alfiebot, which runs the NVIDIA GR00T model on-device using TensorRT for real-time policy execution.

## Architecture

The server uses ZeroMQ REP/REQ pattern to receive observations and return action predictions:

```
┌─────────────────────────────────────────────────────────────────┐
│                    Jetson Orin NX (Server)                      │
│                                                                 │
│  ┌───────────────────────────────────────────────────────────┐ │
│  │              GR00T Inference Server Node                  │ │
│  │                                                           │ │
│  │  1. Receives observation via ZeroMQ REP                   │ │
│  │     - 4 JPEG images (320x280)                            │ │
│  │     - 21D normalized state vector                        │ │
│  │     - Language task description                          │ │
│  │                                                           │ │
│  │  2. Processes observation                                │ │
│  │     - Decodes JPEG images → RGB                          │ │
│  │     - Prepares observation dict                          │ │
│  │                                                           │ │
│  │  3. Runs inference                                       │ │
│  │     - TensorRT engine (15-20 FPS)                        │ │
│  │     - or PyTorch model (slower fallback)                 │ │
│  │                                                           │ │
│  │  4. Returns action prediction                            │ │
│  │     - 16-step action horizon (16 x 21D)                  │ │
│  │     - Inference timing statistics                        │ │
│  │                                                           │ │
│  └───────────────────────────────────────────────────────────┘ │
│                              │                                  │
│                              ▼                                  │
│                  ZeroMQ REP (bind)                             │
│            ipc:///tmp/groot_inference.sock                     │
│                     or tcp://*:5555                            │
└─────────────────────────────────────────────────────────────────┘
```

## Features

### Transport Modes

| Mode | Address | Latency | Use Case |
|------|---------|---------|----------|
| **IPC** | `ipc:///tmp/groot_inference.sock` | ~1ms | On-device inference (default) |
| **TCP** | `tcp://*:5555` | ~2-5ms | Remote PC server |

IPC (Inter-Process Communication) uses Unix domain sockets and is ~5x faster than TCP localhost. Always use IPC for on-device deployment.

### Inference Modes

1. **Production Mode** (default)
   - Loads trained GR00T model checkpoint
   - Runs TensorRT accelerated inference
   - Target: 15-20 FPS on Jetson Orin NX

2. **Mock Mode** (testing)
   - Returns dummy actions without GPU/model
   - Useful for testing communication pipeline
   - Action pattern: exponential decay to zero (hold position)

## Message Protocol

### Request (Client → Server)

The server receives msgpack-encoded observations:

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

### Response (Server → Client)

The server returns msgpack-encoded actions:

```python
{
    "actions": [                           # 16-step action horizon
        [0.1, 0.2, ...],                  # Step 0 (21D)
        [0.1, 0.2, ...],                  # Step 1
        ...
        [0.1, 0.2, ...]                   # Step 15
    ],
    "inference_time_ms": 25.3,
    "status": "ok"                         # or "error"
}
```

### Error Response

On error, the server returns:

```python
{
    "actions": [],
    "inference_time_ms": 0.0,
    "status": "error",
    "error_message": "Model inference failed: ..."
}
```

## Configuration

Edit [config/groot_server.yaml](config/groot_server.yaml):

```yaml
groot_server:
  ros__parameters:
    # Transport: "ipc" for on-device, "tcp" for remote
    transport: "ipc"
    ipc_path: "/tmp/groot_inference.sock"

    # Model configuration
    model_checkpoint: "/home/alfie/alfiebot_ws/models/groot_alfiebot_latest.pth"
    use_tensorrt: true
    mock_mode: false
    action_horizon: 16
    device: "cuda:0"
```

## Running the Server

### Prerequisites

**1. Install GR00T SDK:**
```bash
# Install NVIDIA GR00T SDK (requires NGC access)
pip install nvidia-groot
```

**2. Install dependencies:**
```bash
pip install pyzmq msgpack numpy opencv-python
```

**3. Build the package:**
```bash
cd ~/alfiebot_ws
colcon build --packages-select alfie_gr00t --symlink-install
source install/setup.bash
```

**4. Train or download a model checkpoint:**
```bash
# Place your trained model here:
mkdir -p ~/alfiebot_ws/models
cp /path/to/groot_alfiebot_latest.pth ~/alfiebot_ws/models/
```

### Launch Methods

**On-device inference (server + client together):**
```bash
# Launches both server and client with IPC transport
ros2 launch alfie_gr00t groot_inference.launch.py
```

**Mock mode (testing without GPU/model):**
```bash
ros2 launch alfie_gr00t groot_inference.launch.py mock_mode:=true
```

**Server only (no client):**
```bash
ros2 run alfie_gr00t groot_server --ros-args \
    -p transport:=ipc \
    -p model_checkpoint:=/home/alfie/alfiebot_ws/models/groot_alfiebot_latest.pth
```

**Remote TCP server:**
```bash
# Run on remote PC
ros2 run alfie_gr00t groot_server --ros-args \
    -p transport:=tcp \
    -p bind_host:=* \
    -p bind_port:=5555
```

### Custom Model

```bash
ros2 launch alfie_gr00t groot_inference.launch.py \
    model_checkpoint:=/path/to/custom_model.pth
```

## ROS2 Topics

| Topic | Type | Description |
|-------|------|-------------|
| `/alfie/groot_server/status` | std_msgs/String | Server status and statistics |

### Status Message

Published at 1 Hz:

```python
{
    'total_requests': 1234,
    'average_inference_ms': 24.5,
    'last_inference_ms': 25.3,
    'mock_mode': False,
    'tensorrt': True,
    'bind_address': 'ipc:///tmp/groot_inference.sock'
}
```

## Performance

### Expected Performance on Jetson Orin NX 16GB

| Mode | FPS | Latency | GPU Memory |
|------|-----|---------|------------|
| TensorRT FP16 | 15-20 | 50-65ms | ~4GB |
| PyTorch FP32 | 5-8 | 125-200ms | ~6GB |
| Mock Mode | 1000+ | <1ms | 0 |

### Optimization Tips

1. **Use TensorRT:** Set `use_tensorrt: true` for 3x speedup
2. **Use IPC transport:** 5x faster than TCP for on-device
3. **FP16 inference:** Enabled by default in TensorRT mode
4. **Batch size 1:** Optimized for real-time single-sample inference

## Implementation Details

### Inference Pipeline

1. **Receive observation** via ZeroMQ REP socket
2. **Deserialize** msgpack → Python dict
3. **Decode images** from JPEG → RGB numpy arrays
4. **Prepare observation dict** for GR00T model
5. **Run inference:**
   - TensorRT engine (if enabled) or
   - PyTorch model (fallback)
6. **Serialize response** to msgpack
7. **Send response** via ZeroMQ

### Thread Safety

- Server runs in main ROS2 thread
- ZeroMQ socket is NOT thread-safe (single-threaded access only)
- Timer callback processes one request per cycle (~1ms loop)

### Error Handling

- Invalid observations return error response
- Model failures return error response
- Socket errors logged and connection maintained
- IPC socket cleaned up on shutdown

## File Structure

```
alfie_gr00t/
├── alfie_gr00t/
│   └── nodes/
│       ├── groot_client.py       # Client node
│       └── groot_server.py       # Server node (this file)
├── config/
│   ├── groot_client.yaml         # Client configuration
│   └── groot_server.yaml         # Server configuration
├── launch/
│   └── groot_inference.launch.py # Launches server + client
├── GROOT_CLIENT.md               # Client documentation
└── GROOT_SERVER.md               # This document
```

## Troubleshooting

### Server won't start

```bash
# Check if IPC socket already exists
ls -la /tmp/groot_inference.sock

# Remove stale socket
rm /tmp/groot_inference.sock
```

### Model not loading

```bash
# Check model file exists
ls -la /home/alfie/alfiebot_ws/models/groot_alfiebot_latest.pth

# Run in mock mode to test
ros2 launch alfie_gr00t groot_inference.launch.py mock_mode:=true
```

### Low FPS

```bash
# Enable TensorRT
ros2 launch alfie_gr00t groot_inference.launch.py use_tensorrt:=true

# Check GPU usage
nvidia-smi

# Monitor inference timing
ros2 topic echo /alfie/groot_server/status
```

### Connection Issues

```bash
# Test ZMQ server
python3 -c "import zmq; ctx=zmq.Context(); s=ctx.socket(zmq.REP); s.bind('ipc:///tmp/test.sock'); print('Bound')"

# Check port availability (TCP mode)
netstat -tulpn | grep 5555
```

## Integration with Client

The server is designed to work with [groot_client.py](alfie_gr00t/nodes/groot_client.py). See [GROOT_CLIENT.md](GROOT_CLIENT.md) for client documentation.

**Typical deployment:**

```bash
# Single launch file starts both server and client
ros2 launch alfie_gr00t groot_inference.launch.py

# Activate inference
ros2 topic pub --once /alfie/groot_client/activate std_msgs/Bool "data: true"
```

## Development

### Testing without Robot

Run in mock mode to test the communication pipeline:

```bash
# Start server in mock mode
ros2 launch alfie_gr00t groot_inference.launch.py mock_mode:=true

# In another terminal, activate client
ros2 topic pub --once /alfie/groot_client/activate std_msgs/Bool "data: true"

# Monitor status
ros2 topic echo /alfie/groot_server/status
ros2 topic echo /alfie/groot_client/status
```

### Adding Custom Inference Backend

To use a different inference backend, modify `_run_inference()` in [groot_server.py](alfie_gr00t/nodes/groot_server.py):

```python
def _run_inference(self, images, state, language):
    # Your custom inference code here
    actions = your_model.predict(images, state, language)
    return actions  # Must return (16, 21) numpy array
```

## See Also

- [GROOT_CLIENT.md](GROOT_CLIENT.md) - Client node documentation
- [README.md](README.md) - Package overview
- [alfiebot_config.py](alfie_gr00t/alfiebot_config.py) - GR00T modality configuration
