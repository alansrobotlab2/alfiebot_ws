# alfie_gr00t

NVIDIA GR00T n1.6 integration for Alfie robotics platform - Can-Do Challenge Edition

## Mission: Can-Do Challenge (February 2025)

Train a GR00T n1.6 policy for Alfie to autonomously:
1. **Find** a soda can on the floor
2. **Identify** the soda can among other objects
3. **Pick up** the soda can successfully

## Mission Parameters:
1. Back Height set at 0.100 Meters
2. All Episodes captured with right hand
3. Can always placed radially at any angle within 1 to 3 feet of robot
4. Can is always right side up
5. Can can be facing any direction
6. Alfie will:
  - turn head to find can, right first, then left
  - turn to face the can 
  - then approach to 1.25 foot from right corner of base
  - pick up the can, centered, right side up, 1 foot from the floor (approx)
7. (bonus) Alfie will set the can back down, 1 foot from center of robot

## GR00T n1.6 Overview

GR00T (Generalist Robot 00 Technology) n1.6 is NVIDIA's foundation model for humanoid robotics that enables:
- Vision-language-action (VLA) learning
- Real-world imitation learning from demonstrations
- Multi-modal sensor fusion (RGB, depth, proprioception)
- End-to-end visuomotor policies
- Direct deployment on Jetson platforms

## System Architecture

```
┌───────────────────────────────────────────────────────────────────┐
│              Real-World Data Collection (Alfie)                   │
├───────────────────────────────────────────────────────────────────┤
│  Teleoperation on Hardware                                        │
│  - Collect RGB-D observations                                     │
│  - Record joint states and actions                                │
│  - Human demonstrations of can pickup                             │
│  - 500+ successful trajectories                                   │
└───────────────────────────────────────────────────────────────────┘
                              │
                              ▼ Transfer to Training Workstation
┌───────────────────────────────────────────────────────────────────┐
│                   Training Pipeline (RTX 5090)                    │
├───────────────────────────────────────────────────────────────────┤
│  GR00T SDK Fine-tuning                                            │
│  - Load pre-trained GR00T n1.6 foundation model                   │
│  - Fine-tune on Alfie can-pickup demonstrations                   │
│  - Imitation learning (behavior cloning)                          │
│  - Full precision FP32/FP16 training                              │
└───────────────────────────────────────────────────────────────────┘
                              │
                              ▼ Export & Quantize
┌───────────────────────────────────────────────────────────────────┐
│              Model Optimization Pipeline                          │
├───────────────────────────────────────────────────────────────────┤
│  1. Export to ONNX                                                │
│  2. Quantize to INT8 with calibration                             │
│  3. Build TensorRT engine for Jetson Orin                         │
│  4. Validate accuracy vs. full precision                          │
└───────────────────────────────────────────────────────────────────┘
                              │
                              ▼ Deploy
┌───────────────────────────────────────────────────────────────────┐
│        Alfie Platform (Jetson Orin NX 16GB)                       │
├───────────────────────────────────────────────────────────────────┤
│  ROS2 Topics          │  GR00T Integration (INT8)                 │
│  - Camera feeds       │  - Policy inference node (TensorRT)       │
│  - Joint states       │  - Action publisher (arm + base + gripper)│
│  - Gripper state      │  - Observation preprocessor               │
│  - E-stop             │  - Model management                       │
└───────────────────────────────────────────────────────────────────┘
```

## Hardware Requirements

### Training Workstation
- [ ] **NVIDIA RTX 5090** (32GB VRAM)
  - Full precision fine-tuning (FP32/FP16)
  - GR00T n1.6 SDK training
  - CUDA 12.x support
  - ~500W power requirement

### Deployment Platform (Alfie Robot)
- [ ] **NVIDIA Jetson Orin NX 16GB**
  - 100 TOPS INT8 performance
  - 16GB unified memory
  - **JetPack 6.2.1** (L4T 36.4)
  - TensorRT 10.x for INT8 inference
  - 10-25W power envelope
  - ROS2 Humble native support

### Sensors (Alfie Platform)
- [ ] RGB cameras (stereo or mono)
- [ ] Depth sensor (RealSense, ZED, etc.)
- [ ] Joint encoders for arm/gripper
- [ ] Wheel encoders for base odometry

### Actuators
- [ ] Manipulator arm (6-7 DOF recommended)
- [ ] Parallel jaw gripper or adaptive gripper
- [ ] Mobile base (differential drive or omnidirectional)

## Software Stack

### Core Dependencies
- [ ] Ubuntu 22.04 LTS (training workstation)
- [ ] Ubuntu 22.04 LTS (Jetson Orin NX via JetPack 6.2.1)
- [ ] ROS2 Humble
- [ ] Python 3.10+
- [ ] PyTorch 2.0+
- [ ] CUDA Toolkit 12.x

### GR00T n1.6 Specific

**Training Workstation (RTX 5090):**
- [ ] GR00T SDK n1.6 (access via NVIDIA Developer Program)
- [ ] Pre-trained GR00T n1.6 foundation model
- [ ] PyTorch 2.0+ with CUDA support
- [ ] ONNX export tools
- [ ] Dataset recording and preprocessing tools

**Jetson Orin NX Deployment:**
- [ ] **JetPack 6.2.1** SDK
- [ ] TensorRT 10.x with INT8 support
- [ ] CUDA 12.x for Jetson
- [ ] PyTorch 2.x for Jetson (optional, for debugging)
- [ ] onnx2trt conversion tools
- [ ] NVIDIA DLA (Deep Learning Accelerator) drivers

## Implementation Roadmap

### Phase 1: Environment Setup (Week 1)
**Training Workstation (RTX 5090):**
- [ ] Install GR00T SDK n1.6 and dependencies
- [ ] Download pre-trained GR00T n1.6 foundation model
- [ ] Verify GPU compute capability and drivers
- [ ] Set up Python virtual environment
- [ ] Install PyTorch, ONNX tools, TensorRT

**Alfie Robot (Jetson Orin NX):**
- [ ] Verify JetPack 6.2.1 installation
- [ ] Configure ROS2 Humble workspace with alfie_gr00t package
- [ ] Install TensorRT 10.x runtime
- [ ] Set up camera calibration
- [ ] Verify all sensor topics publishing correctly

### Phase 2: Teleoperation & Data Collection (Week 1-3)
- [X] Set up teleoperation system for Alfie:
  - [X] Implement ROS2 teleoperation node
  - [X] VR controller or gamepad control
  - [X] Joint-level control interface
  - [X] Gripper open/close commands
- [X] Implement data recording pipeline:
  - [X] Record RGB camera streams (4 stereo streams, compressed)
  - [X] Record joint states (positions, velocities)
  - [X] Record gripper states (gdb0, gdb1)
  - [X] Record base commands (backcmd)
  - [X] Timestamp synchronization (automatic via ROS2 bag)
  - [X] ROS2 bag storage format (MCAP + zstd compression)
- [ ] Collect real-world demonstrations:
  - [ ] 500+ successful can pickup trajectories
  - [ ] Varying can positions and orientations
  - [ ] Different lighting conditions
  - [ ] Multiple starting configurations
  - [ ] Multiple environments
- [ ] Label and validate demonstration quality:
  - [ ] Mark successful vs failed attempts
  - [ ] Annotate grasp quality
  - [ ] Filter out corrupted recordings

### Phase 3: GR00T Policy Fine-Tuning (Week 3-5)
- [ ] Prepare training dataset:
  - [ ] Convert ROS2 bags to GR00T format
  - [ ] Preprocess observations (resize, normalize)
  - [ ] Augment data (flips, color jitter, noise)
  - [ ] Split train/val/test sets (80/10/10)
- [ ] Configure GR00T n1.6 fine-tuning:
  - [ ] Load pre-trained foundation model
  - [ ] Define observation space (RGB, depth, joint states)
  - [ ] Define action space (arm joints, gripper, base Twist)
  - [ ] Set learning rate and batch size
  - [ ] Configure behavior cloning loss
- [ ] Train on RTX 5090:
  - [ ] Fine-tune GR00T on Alfie demonstrations
  - [ ] Monitor training loss and validation metrics
  - [ ] Save checkpoints every N iterations
  - [ ] Early stopping on validation plateau
  - [ ] Target <0.01 demonstration loss

### Phase 4: Model Quantization & Optimization (Week 5-6)
- [ ] **Export to ONNX**:
  - [ ] Export trained PyTorch model to ONNX format
  - [ ] Verify ONNX model outputs match PyTorch
  - [ ] Simplify ONNX graph (remove unused nodes)
- [ ] **INT8 Quantization**:
  - [ ] Collect calibration dataset (1000+ diverse observations)
  - [ ] Run PTQ (Post-Training Quantization) to INT8
  - [ ] Measure accuracy degradation (target <2% drop)
  - [ ] Fine-tune quantization-aware if needed
- [ ] **TensorRT Engine Building**:
  - [ ] Build TensorRT engine for Jetson Orin NX
  - [ ] Enable DLA (Deep Learning Accelerator) if applicable
  - [ ] Optimize for INT8 throughput
  - [ ] Target <30ms inference latency
- [ ] **Validation**:
  - [ ] Test quantized model in Isaac Sim
  - [ ] Compare success rates: FP32 vs INT8
  - [ ] Profile memory usage on Jetson
  - [ ] Benchmark inference FPS

### Phase 5: Jetson Deployment (Week 6-7)
- [ ] **Jetson Setup** (already on JetPack 6.2.1):
  - [ ] Verify JetPack 6.2.1 components
  - [ ] Install/verify TensorRT 10.x runtime
  - [ ] Configure power mode (15W or 25W)
  - [ ] Optimize CPU/GPU clocks for inference
  - [ ] Set up NVMe SSD for model storage (if available)
- [ ] **ROS2 Integration**:
  - [ ] Implement observation bridge (ROS2 → GR00T):
    - [ ] Subscribe to 4 camera streams (`/alfie/stereo_camera/*/image_raw/compressed`)
    - [ ] Subscribe to robot state (`/alfie/robotlowstate`)
    - [ ] Decompress JPEG images from compressed topics
    - [ ] Resize camera images (352x256 → model input size, e.g., 224x224)
    - [ ] Normalize pixel values (0-255 → model range)
    - [ ] Parse robotlowstate into GR00T observation format
    - [ ] Synchronize multi-modal inputs (cameras + state) with timestamps
    - [ ] Package observations into GR00T-compatible tensor/dict
  - [ ] Implement TensorRT inference node:
    - [ ] Load INT8 TensorRT engine on Jetson
    - [ ] Create inference pipeline with batching (batch_size=1)
    - [ ] Handle GPU memory management
    - [ ] Profile inference latency (target <30ms)
  - [ ] Implement action publisher (GR00T → ROS2):
    - [ ] Convert GR00T action tensor to `/alfie/robotlowcmd` message
    - [ ] Map action dimensions to robot command fields
    - [ ] Apply action scaling/denormalization
    - [ ] Implement temporal action smoothing filter
    - [ ] Add workspace boundary checks
    - [ ] Add velocity/acceleration limits
  - [ ] Add safety monitors and E-stop handling:
    - [ ] Subscribe to E-stop topic
    - [ ] Implement watchdog timer for inference failures
    - [ ] Add collision detection checks
    - [ ] Graceful degradation on errors
- [ ] **Deploy to Alfie**:
  - [ ] Transfer INT8 TensorRT engine to Jetson
  - [ ] Calibrate camera extrinsics/intrinsics
  - [ ] Test with real soda cans in controlled environment
  - [ ] Monitor CPU/GPU utilization and thermals
  - [ ] Iterate on domain randomization gaps

### Phase 6: Real-World Testing (Week 7-8)
- [ ] Create test scenarios with increasing difficulty:
  - [ ] Level 1: Can in fixed position, good lighting
  - [ ] Level 2: Can in random positions
  - [ ] Level 3: Multiple objects, can among distractors
  - [ ] Level 4: Poor lighting, occlusions
  - [ ] Level 5: Moving can, dynamic environment
- [ ] Measure success metrics:
  - [ ] Detection accuracy
  - [ ] Grasp success rate
  - [ ] Task completion time
  - [ ] Failure modes
- [ ] Collect real-world failure cases
- [ ] Fine-tune policy with real data (optional)

### Phase 7: Competition Preparation (Week 8-10)
- [ ] Optimize policy for competition rules
- [ ] Add fail-safe behaviors and recovery
- [ ] Test in competition-like conditions
- [ ] Prepare backup policies
- [ ] Create demonstration video
- [ ] Document approach and results
- [ ] Pack and ship hardware to venue (if required)

## ROS2 Package Structure

```
alfie_gr00t/
├── alfie_gr00t/
│   ├── __init__.py
│   ├── nodes/
│   │   ├── groot_inference_node.py      # Main policy inference
│   │   ├── observation_bridge.py        # ROS2 to GR00T obs
│   │   ├── action_publisher.py          # GR00T to ROS2 actions
│   │   └── teleop_recorder.py           # Demo collection
│   ├── core/
│   │   ├── policy_manager.py            # Load/manage policies
│   │   ├── preprocessor.py              # Sensor preprocessing
│   │   └── action_postprocessor.py      # Action smoothing/safety
│   ├── training/
│   │   ├── dataset.py                   # Dataset loader
│   │   ├── train_policy.py              # Fine-tuning script
│   │   └── eval_policy.py               # Evaluation utilities
│   └── utils/
│       ├── visualization.py             # Debug visualizations
│       └── metrics.py                   # Performance tracking
├── launch/
│   ├── groot_inference.launch.py        # Deploy trained policy
│   ├── data_collection.launch.py        # Record demonstrations
│   └── teleop.launch.py                 # Teleoperation interface
├── config/
│   ├── groot_config.yaml                # GR00T parameters
│   ├── sensors.yaml                     # Camera/sensor config
│   └── action_space.yaml                # Robot action space
├── models/
│   ├── checkpoints/                     # Trained model weights (FP32/FP16)
│   ├── onnx/                            # Exported ONNX models
│   ├── tensorrt/                        # TensorRT engines (INT8 for Jetson)
│   └── calibration/                     # Calibration data for quantization
├── data/
│   ├── demonstrations/                  # Recorded demos
│   └── datasets/                        # Training datasets
├── scripts/
│   ├── setup_training.sh                # Environment setup (RTX 5090)
│   ├── setup_jetson.sh                  # Jetson Orin setup (JetPack 6.2.1)
│   ├── convert_dataset.py               # Convert ROS bags to GR00T format
│   ├── train.sh                         # Fine-tuning launcher (RTX 5090)
│   ├── export_onnx.sh                   # Export model to ONNX
│   ├── quantize_int8.py                 # INT8 quantization script
│   ├── build_tensorrt.sh                # Build TensorRT engine for Jetson
│   └── deploy_jetson.sh                 # Deploy to Jetson Orin
├── tests/
│   └── test_inference.py                # Unit tests
├── package.xml
├── setup.py
├── setup.cfg
└── README.md
```

## Key Topics and Interfaces

### Subscribed Topics (Observation Space)
```python
# Camera input (4 stereo streams - compressed for efficiency)
/alfie/stereo_camera/left_wide/image_raw/compressed     # sensor_msgs/CompressedImage
/alfie/stereo_camera/left_wide/camera_info              # sensor_msgs/CameraInfo
/alfie/stereo_camera/left_center/image_raw/compressed   # sensor_msgs/CompressedImage
/alfie/stereo_camera/left_center/camera_info            # sensor_msgs/CameraInfo
/alfie/stereo_camera/right_center/image_raw/compressed  # sensor_msgs/CompressedImage
/alfie/stereo_camera/right_center/camera_info           # sensor_msgs/CameraInfo
/alfie/stereo_camera/right_wide/image_raw/compressed    # sensor_msgs/CompressedImage
/alfie/stereo_camera/right_wide/camera_info             # sensor_msgs/CameraInfo

# Robot state (proprioception)
/alfie/robotlowstate            # Custom low-level robot state
```

### Published Topics (Action Space)
```python
# Robot commands (what GR00T policy will output)
/alfie/robotlowcmd              # Custom low-level robot commands
# Debug/visualization (policy inference node)
/groot/policy_output            # Visualization of policy decisions
/groot/attention_maps           # Visual attention from model
/groot/metrics                  # Real-time performance metrics
```

## Training Strategy

### Imitation Learning (Behavior Cloning)
1. Collect diverse real-world demonstrations on Alfie hardware
2. Fine-tune pre-trained GR00T n1.6 foundation model
3. Use behavior cloning loss (MSE on actions)
4. Augment data with:
   - **Visual**: Color jitter, brightness, contrast, Gaussian noise
   - **Temporal**: Random frame skipping, speed variations
   - **Spatial**: Small rotations, translations (within valid workspace)

### Foundation Model Transfer
- Start with GR00T n1.6 pre-trained on large-scale robot data
- Leverage learned visual representations and manipulation priors
- Fine-tune final layers for Alfie-specific can pickup task
- Much faster convergence than training from scratch

## Success Metrics

### Training Metrics (RTX 5090)
- [ ] Demonstration loss (MSE) < 0.01
- [ ] Validation loss plateau detection
- [ ] Action prediction accuracy > 95%
- [ ] Policy inference (FP32) < 20ms @ 500+ FPS

### Quantization Metrics
- [ ] INT8 accuracy drop < 2% vs FP32
- [ ] Model size reduction: ~4x (from ~500MB to ~125MB)
- [ ] Jetson inference latency < 30ms @ 33+ FPS
- [ ] Memory footprint < 2GB on Jetson
- [ ] Power consumption < 15W during inference

### Real-World Metrics (Jetson Orin NX)
- [ ] Can detection accuracy > 90%
- [ ] Grasp success rate > 80%
- [ ] Task completion time < 30 seconds
- [ ] Zero hardware collisions/damage
- [ ] Stable operation for 1+ hour continuous use

## Risk Mitigation

### Technical Risks
- **Sim-to-real gap**: Extensive domain randomization, real-world fine-tuning
- **Grasp failures**: Multiple grasp strategies, recovery behaviors
- **Quantization accuracy loss**: Careful calibration, QAT if needed, validate <2% drop
- **Jetson inference latency**: TensorRT optimization, DLA offloading, input resolution tuning
- **Memory constraints on Jetson**: Model pruning, efficient preprocessing, batch size = 1
- **Thermal throttling**: Power mode tuning (15W vs 25W), active cooling
- **Hardware failures**: Redundant sensors, safety monitors, watchdog timers

### Competition Risks
- **Novel scenarios**: Train on diverse environments
- **Lighting conditions**: Test in various lighting
- **Can variations**: Train on multiple can types
- **Time constraints**: Optimize for speed and reliability

## Resources

### Documentation
- [NVIDIA GR00T Project Page](https://developer.nvidia.com/project-gr00t)
- [GR00T SDK Documentation](https://developer.nvidia.com/gr00t-sdk)
- [JetPack 6.2.1 Documentation](https://developer.nvidia.com/embedded/jetpack)
- [TensorRT 10.x Documentation](https://docs.nvidia.com/deeplearning/tensorrt/)

### Community
- NVIDIA Developer Forums
- ROS2 Manipulation SIG
- Jetson Community Forums

### Reference Implementations
- GR00T n1.6 example fine-tuning scripts (from SDK)
- ROS2 teleoperation examples
- TensorRT INT8 quantization examples

## Timeline Summary

| Week | Phase | Key Deliverables | Hardware |
|------|-------|------------------|----------|
| 1    | Environment Setup | GR00T SDK + JetPack verified | RTX 5090 + Jetson |
| 1-3  | Teleoperation & Data | 500+ real-world demonstrations | Alfie Hardware |
| 3-5  | Fine-Tuning | GR00T policy fine-tuned (FP32) | RTX 5090 |
| 5-6  | Quantization | INT8 model, TensorRT engine | RTX 5090 + Jetson |
| 6-7  | Jetson Deployment | Policy running on Orin NX | Jetson Orin NX |
| 7-8  | Testing | >80% real-world success rate | Jetson Orin NX |
| 8-10 | Competition Prep | Competition-ready system | Jetson Orin NX |

**Competition Date: February 2025**

## License

Apache-2.0

## Maintainers

- Alfie Team <alansrobotlab@gmail.com>

---

**Status**: 🚀 Project Initiated - Ready for Phase 1

**Last Updated**: 2025-12-29
