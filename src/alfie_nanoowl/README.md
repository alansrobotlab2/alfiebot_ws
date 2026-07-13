# alfie_nanoowl

Open-vocabulary object detection with [NanoOWL](https://github.com/NVIDIA-AI-IOT/nanoowl)
(OWL-ViT accelerated for Jetson). The node watches a camera stream and publishes
a list of detections for whatever text queries you give it — no fixed class set.

## Topics

| Dir | Topic | Type | Notes |
|-----|-------|------|-------|
| sub | `stereo_camera/left_center/image_raw/compressed` | `sensor_msgs/CompressedImage` | source frames (`image_topic` param) |
| sub | `nanoowl/prompt` | `std_msgs/String` | change queries live, e.g. `"a person, a cup"` |
| pub | `nanoowl/detections` | `alfie_msgs/Detections` | detection list per processed frame |
| pub | `nanoowl/annotated/compressed` | `sensor_msgs/CompressedImage` | boxes drawn on frame (if `publish_annotated:=true`) |

`alfie_msgs/Detections` carries the frame `header`, `source_topic`, `prompt`,
`image_width/height`, and a `Detection[]` where each `Detection` has `label`,
`class_id`, `score`, and a normalized (0..1) corner box `x0,y0,x1,y1`.

## Parameters

| Param | Default | Notes |
|-------|---------|-------|
| `image_topic` | `stereo_camera/left_center/image_raw/compressed` | compressed input stream |
| `prompt` | `a person, a face, a hand` | comma-separated queries (`[a, b]` also accepted) |
| `threshold` | `0.1` | sigmoid confidence cutoff |
| `rate_hz` | `5.0` | inference rate; frames between ticks are dropped |
| `model_name` | `google/owlvit-base-patch32` | HF OWL-ViT model |
| `image_encoder_engine` | `` (empty) | TensorRT engine path; empty = pure PyTorch |
| `publish_annotated` | `false` | also publish a debug image with boxes |
| `jpeg_quality` | `70` | annotated image JPEG quality |

The subscription is depth-1 best-effort and inference is timer-driven, so a slow
model drops frames instead of backing up the camera pipeline.

## Install NanoOWL

Not pulled in by rosdep — install into the same Python env the workspace uses:

```bash
pip install transformers Pillow
pip install git+https://github.com/NVIDIA-AI-IOT/nanoowl
```

torch / torchvision / opencv / cv_bridge are already present on the robot.

### (Recommended) Build the TensorRT image-encoder engine

The pure-PyTorch encoder works out of the box but is slow. Build the engine once
for a large speedup on the Orin:

```bash
mkdir -p ~/nanoowl_data
python3 -m nanoowl.build_image_encoder_engine \
    ~/nanoowl_data/owl_image_encoder_patch32.engine
```

Then launch with `image_encoder_engine:=~/nanoowl_data/owl_image_encoder_patch32.engine`.
(Engine build needs `torch2trt`; see the NanoOWL repo if it's missing.)

## Build & run

```bash
cd ~/alfiebot_ws
colcon build --packages-select alfie_msgs alfie_nanoowl
source install/setup.bash

ros2 launch alfie_nanoowl nanoowl.launch.py \
    prompt:="a person, a coffee mug" \
    image_encoder_engine:=~/nanoowl_data/owl_image_encoder_patch32.engine
```

Watch detections:

```bash
ros2 topic echo /alfie/nanoowl/detections
```

Change what it looks for at runtime:

```bash
ros2 topic pub --once /alfie/nanoowl/prompt std_msgs/String "data: 'a dog, a ball'"
```
