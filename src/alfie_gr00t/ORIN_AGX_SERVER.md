

## Current Environment setup

The inference server and the ROS2 client both run on the Orin; there is no
separate GPU workstation. The client reaches the server over localhost TCP
(REQ/REP on 5555, async PUSH/PULL on 5556/5557). `--host 127.0.0.1` keeps the
server off the robot's network; use `--host 0.0.0.0` only if you need to reach
it from another machine.

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
  --host 127.0.0.1 \
  --embodiment NEW_EMBODIMENT \
  --checkpoint ./Isaac-GR00T/alfie-gr00t/checkpoint-2000 \
  --denoising-steps 4 \
  --rtc \
  --verbose
```

`--rtc` enables Real-Time Chunking: the client sends the actions it has already
committed for the frames after each observation (`rtc_enabled` in
`groot_client.yaml`) and the server seeds the new chunk with them through
N1.7's native inpainting, so consecutive chunks agree where they meet. It
needs at least 4 denoising steps (2-step denoising cannot adapt around the
frozen prefix, see `RTC_ANALYSIS.md`). `--rtc-ramp-rate` shapes the blend from
frozen to free steps; `--rtc-frozen-steps` / `--rtc-overlap-steps` override
what the client sends.

```bash
# previous N1.6 form, for reference
python ./alfiebot_ws/src/alfie_gr00t/alfie_gr00t/scripts/groot_inference_server.py \
  --checkpoint ./Isaac-GR00T/alfie-gr00t/checkpoint-2000 \
  --trt-engine-path ./Isaac-GR00T/groot_n1d6_onnx/dit_model_fp16_orin.trt \
  --denoising-steps 4 \
  --verbose
```

## Open-Loop Evaluation

Replays a recorded episode through the ZMQ client→server inference pipeline and
compares predicted actions against ground truth. Produces trajectory plots and
communication diagnostics. No ROS2 required.

```bash
python ./alfiebot_ws/src/alfie_gr00t/alfie_gr00t/scripts/groot_open_loop_eval.py \
    --dataset-path ./alfiebot_ws/data/can_do_challenge/lerobot \
    --episode-index 0 \
    --host 127.0.0.1 --port 5555
```