

## Current Environment setup

```bash
cd ~/

docker run \
  -it --rm --runtime=nvidia \
  --network=host \
  -v $(pwd)/.:/workspace/gr00t gr00t-dev /bin/bash
```

```bash
cd Isaac-GR00T

pip install -e .

cd ..
```

```bash
python ./alfiebot_ws/src/alfie_gr00t/alfie_gr00t/scripts/groot_inference_server.py \
  --transport tcp \
  --embodiment NEW_EMBODIMENT \
  --checkpoint ./Isaac-GR00T/alfie-gr00t/checkpoint-2000 \
  --trt-engine-path ./Isaac-GR00T/groot_n1d6_onnx/dit_model_fp16_orin.trt \
  --action-horizon 16 \
  --denoising-steps 4 \
  --enable-viz \
  --verbose
```

## Open-Loop Evaluation

Replays a recorded episode through the ZMQ client→server inference pipeline and
compares predicted actions against ground truth. Produces trajectory plots and
communication diagnostics. No ROS2 required.

```bash
python ./alfiebot_ws/src/alfie_gr00t/alfie_gr00t/scripts/groot_open_loop_eval.py \
    --dataset-path ./alfiebot_ws/data/alfiebot.CanDoChallenge \
    --episode-index 0 \
    --host 192.168.50.201 --port 5555
```