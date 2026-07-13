# alfie_nanoowl

Open-vocabulary object detection with [NanoOWL](https://github.com/NVIDIA-AI-IOT/nanoowl)
(OWL-ViT accelerated for Jetson). Detection runs **on demand**, not continuously:
the node keeps the model warm and holds the latest camera frame, but runs
inference only when its `nanoowl/detect` service is called — so it never competes
with the LLM / GR00T for the GPU while nobody's asking. Each call can look for
whatever text queries you give it (no fixed class set).

The agent's `look` tool (alfie_agent → tools/see.py) is the usual caller.

## Interface

| Kind | Name | Type | Notes |
|------|------|------|-------|
| service | `nanoowl/detect` | `alfie_msgs/Detect` | run one detection; request carries optional `prompt` + `threshold`, reply carries `Detections` |
| sub | `stereo_camera/left_center/image_raw/compressed` | `sensor_msgs/CompressedImage` | latest frame is held undecoded (`image_topic` param); decoded only on a detect |
| pub | `nanoowl/detections` | `alfie_msgs/Detections` | republished after each detect (observability only — nothing streams) |
| pub | `nanoowl/annotated/compressed` | `sensor_msgs/CompressedImage` | boxes drawn on frame, per detect (if `publish_annotated:=true`) |

`alfie_msgs/Detect` (service): request `string prompt`, `float32 threshold`;
reply `bool ok`, `string message`, `Detections detections`. An empty request
`prompt` keeps the detector's current prompt; `threshold <= 0` uses the node default.

`alfie_msgs/Detections` carries the frame `header`, `source_topic`, `prompt`,
`image_width/height`, and a `Detection[]` where each `Detection` has `label`,
`class_id`, `score`, and a normalized (0..1) corner box `x0,y0,x1,y1`.

## Parameters

| Param | Default | Notes |
|-------|---------|-------|
| `image_topic` | `stereo_camera/left_center/image_raw/compressed` | compressed input stream |
| `detect_service` | `nanoowl/detect` | service name |
| `prompt` | broad default list | comma-separated queries used when a request omits its own (`[a, b]` also accepted) |
| `threshold` | `0.1` | sigmoid confidence cutoff |
| `model_name` | `google/owlvit-base-patch32` | HF OWL-ViT model |
| `image_encoder_engine` | `~/nanoowl_data/...engine` if present | TensorRT engine path; empty = pure PyTorch |
| `publish_annotated` | `false` | also publish a debug image with boxes on each detect |
| `jpeg_quality` | `70` | annotated image JPEG quality |

## Install NanoOWL

Not pulled in by rosdep — install into the same Python env the workspace uses:

```bash
pip install transformers Pillow
pip install git+https://github.com/NVIDIA-AI-IOT/nanoowl
```

torch / torchvision / opencv / cv_bridge are already present on the robot.

### TensorRT image-encoder engine (built)

The pure-PyTorch encoder works out of the box but is slow (~370–590 ms/frame).
The FP16 TensorRT engine cuts that to ~130 ms/frame (encoder itself ~6 ms) and
loads in ~2 s instead of ~18 s. It is already built at:

    ~/nanoowl_data/owl_image_encoder_patch32.engine   # 183 MB, sm_87 / fp16

Launch with `image_encoder_engine:=~/nanoowl_data/owl_image_encoder_patch32.engine`.

To rebuild it (e.g. after a JetPack/TensorRT upgrade):

```bash
mkdir -p ~/nanoowl_data
python3 -m nanoowl.build_image_encoder_engine \
    ~/nanoowl_data/owl_image_encoder_patch32.engine
```

**TensorRT on this Orin — important.** The engine build (and `torch2trt`) needs a
*Tegra* TensorRT, not the pip `tensorrt` wheel. The pip wheel pulls
`tensorrt-cu13` (CUDA 13, dGPU archs, no sm_87) and fails at builder creation
with `CUDA initialization failure with error: 35`. Use the JetPack apt build:

```bash
pip uninstall -y tensorrt tensorrt-cu13 tensorrt_cu13_bindings tensorrt_cu13_libs
sudo apt install tensorrt                       # 10.3.0.30+cuda12.5, ships trtexec
sudo apt install --allow-change-held-packages nvidia-l4t-dla-compiler  # libnvdla_compiler.so
sudo ldconfig
pip install --no-build-isolation git+https://github.com/NVIDIA-AI-IOT/torch2trt
```

## Build & run

```bash
cd ~/alfiebot_ws
colcon build --packages-select alfie_msgs alfie_nanoowl
source install/setup.bash

# Engine is used automatically if it's been built (see below).
ros2 launch alfie_nanoowl nanoowl.launch.py
```

Trigger a detection on demand (the agent's `look` tool does this for you):

```bash
# use the detector's current prompt
ros2 service call /alfie/nanoowl/detect alfie_msgs/srv/Detect "{prompt: '', threshold: 0.0}"

# or look for something specific (open-vocabulary)
ros2 service call /alfie/nanoowl/detect alfie_msgs/srv/Detect \
    "{prompt: 'a dog, a ball', threshold: 0.1}"
```
