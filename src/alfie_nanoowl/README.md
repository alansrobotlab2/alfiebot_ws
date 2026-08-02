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
| `image_topic` | `stereo_camera/left_center/image_raw/compressed` | compressed input stream (the launch file overrides this to `left_wide`) |
| `detect_service` | `nanoowl/detect` | service name |
| `prompt` | broad default list | comma-separated queries used when a request omits its own (`[a, b]` also accepted) |
| `threshold` | `0.1` | sigmoid confidence cutoff |
| `model_name` | `google/owlvit-base-patch32` | HF OWL-ViT model |
| `image_encoder_engine` | `~/nanoowl_data/...engine` if present | TensorRT engine path; empty = pure PyTorch |
| `publish_annotated` | `false` | also publish a debug image with boxes on each detect |
| `jpeg_quality` | `70` | annotated image JPEG quality |

## Install NanoOWL

Not pulled in by rosdep — install into the same Python env the workspace uses
(system python3.12, user site-packages; PEP 668 means `--break-system-packages`):

```bash
PIP="pip install --user --break-system-packages"

$PIP transformers                       # Pillow / opencv / cv_bridge already present

# torchvision must match the installed torch exactly and come from the same
# CUDA index, or pip will drag in a CPU torch and clobber the Jetson build.
# torch 2.11.0+cu130 -> torchvision 0.26.0+cu130
$PIP --no-deps --index-url https://download.pytorch.org/whl/cu130 torchvision==0.26.0+cu130

$PIP --no-deps git+https://github.com/NVIDIA-AI-IOT/nanoowl
```

`--no-deps` on nanoowl is deliberate: its setup.py declares nothing, but pip
still resolves fine — the flag just guarantees it can never touch torch.

### TensorRT image-encoder engine

The pure-PyTorch encoder works out of the box but is slow (~370–590 ms/frame).
The FP16 TensorRT engine cuts that to **~12 ms/frame** (encoder itself ~6 ms) and
loads in ~3 s instead of ~15 s. Build it to:

    ~/nanoowl_data/owl_image_encoder_patch32.engine   # 182 MB, sm_87 / fp16

The launch file picks that path up automatically when it exists, so nothing to
pass; otherwise `image_encoder_engine:=<path>`.

Build (or rebuild after a JetPack/TensorRT upgrade):

```bash
# ONNX export deps — torch >= 2.9 routes torch.onnx.export through onnxscript
pip install --user --break-system-packages onnx onnxscript
pip install --user --break-system-packages --no-build-isolation --no-deps \
    git+https://github.com/NVIDIA-AI-IOT/torch2trt

mkdir -p ~/nanoowl_data
python3 -m nanoowl.build_image_encoder_engine \
    ~/nanoowl_data/owl_image_encoder_patch32.engine
```

Takes ~2 min (ONNX export is silent for most of it, then `trtexec` runs).

**TensorRT must be the apt/JetPack build, not a pip wheel.** The pip `tensorrt`
package pulls dGPU CUDA libs with no sm_87 and dies at builder creation with
`CUDA initialization failure with error: 35`. Since the JetPack 7 / CUDA 13
upgrade this is already correct on the robot — apt `libnvinfer*` +
`python3-libnvinfer` 10.16.2.10 in `/usr/lib/python3.12/dist-packages`, with
`trtexec` at `/usr/bin/trtexec`. Verify before building:

```bash
python3 -c "import tensorrt as t; t.Builder(t.Logger()); print(t.__file__, t.__version__)"
# -> /usr/lib/python3.12/dist-packages/tensorrt/__init__.py 10.16.2.10
```

If it resolves to `~/.local/...` instead, remove the pip copy:
`pip uninstall -y tensorrt tensorrt-cu13 tensorrt_cu13_bindings tensorrt_cu13_libs`.

**Harmless warning at startup:** torch 2.11.0+cu130 ships no sm_87 cubin, so it
logs `GPU0 Orin which is of compute capability 8.7 ... except {8.7}` and JITs
from PTX. Detection is correct and the hot path is the TensorRT engine anyway.

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
