# RTC robot A/B runbook (GR00T N1.7, TensorRT)

> **Status, 2026-11-12:** only the first A/B pair was run (RTC cut seam jumps 37% and jerk 33%, but both picks failed), then the session turned into diagnostics. The client defaults have since changed: 8 steps per chunk (window [2:10]), motion matched to VR teleop, back start height 0.34 m. Run trials with `scripts/ab_trial.py`, which sets the start pose, activates and auto-stops; the manual activate commands below skip the start pose. Findings are in CLAUDE.md ("Robot findings, 2026-11-12") and `~/gr00t_ab/2026-11-12/trials.csv`. Redo this A/B once the denoising-step question is settled.

Compares three conditions on the robot with checkpoint-20000 and the
full-pipeline INT8 TensorRT engines:

| | Server | Client |
|---|---|---|
| **A** 4-step, RTC off | 4-step server (below) | `rtc_enabled:=false` |
| **B** 4-step, RTC on | same 4-step server | `rtc_enabled:=true` |
| **C** 2-step, RTC on | 2-step server | `rtc_enabled:=true` |

Each condition uses the same client settings from `config/groot_client.yaml`:
- classic execution mode (`continuous_inference: false`)
- chunk size 16
- each chunk runs actions[2:6], and the next inference fires when they finish
- RTC overlap 6 with 2 frozen steps, ramp 3.0 (server default)
- joint EMA 0.3 and the rate limiter

A `--rtc` server answers requests without a prefix as plain inference. A and B therefore share one server and can alternate without a restart.

## Before you start (once per boot / once ever)

```bash
sudo jetson_clocks                              # every boot; the latency numbers assume pinned clocks
sudo apt install python3-zmq python3-msgpack    # once: the client node runs on system python and needs both
cd ~/alfiebot_ws && colcon build --packages-select alfie_gr00t && source install/setup.zsh
```

Then:
1. Bring the robot stack up as usual. Check whether the LLM server is running (`pgrep -af mlc_llm`). The Qwen model holds about 24 GB and shares the GPU, and any LLM request stalls GR00T inference. Stop it for the session, or leave it idle and send it no requests during trials. Do the same for every condition. If you stop it, stop agent_node with it: restarting agent_node without mlc_llm_node wedges the engine.
2. Clear a 2 m radius around Alfie.
3. Mark 6 can positions with tape, 1–3 ft from the robot, at different bearings, like the training data. Use the same positions in the same order for every condition.
4. Keep the e-stop in hand.

Make the log folder:
```bash
export AB=~/gr00t_ab/$(date +%F); mkdir -p $AB
```

## Server (terminal 1)

```bash
cd ~/Isaac-GR00T && source .venv/bin/activate && source scripts/activate_orin.sh
python ~/alfiebot_ws/src/alfie_gr00t/alfie_gr00t/scripts/groot_inference_server.py \
  --host 127.0.0.1 --embodiment NEW_EMBODIMENT \
  --checkpoint checkpoints/alfie/checkpoint-20000 \
  --trt-engine-dir /home/alfie/Isaac-GR00T/gr00t_trt_deployment/alfie_int8_rtc/engines \
  --denoising-steps 4 --rtc 2>&1 | tee $AB/server_4step.log
```

- GPU image preprocessing is on by default.
- Wait for `Server ready`.
- For block 2 (condition C), stop the server with Ctrl-C and rerun it with `--denoising-steps 2`, teeing to `$AB/server_2step.log`.

## Client, one launch per trial (terminal 2)

Name every trial `<cond>_t<n>` so each one gets its own logs:

```bash
cd ~/alfiebot_ws && source install/setup.zsh
# A
ros2 launch alfie_gr00t groot_inference.launch.py rtc_enabled:=false csv_log_path:=$AB/A_t1.csv
# B
ros2 launch alfie_gr00t groot_inference.launch.py rtc_enabled:=true  csv_log_path:=$AB/B_t1.csv
# C (2-step server running)
ros2 launch alfie_gr00t groot_inference.launch.py rtc_enabled:=true  csv_log_path:=$AB/C_t1.csv
```

Start and stop each trial from terminal 3:
```bash
ros2 topic pub --once /alfie/groot_client/activate std_msgs/Bool "data: true"    # start
ros2 topic pub --once /alfie/groot_client/activate std_msgs/Bool "data: false"   # stop
ros2 topic pub --once /alfie/groot_client/estop std_msgs/Bool "data: true"       # software e-stop
```

Stop the client with Ctrl-C after each trial so its logs are closed.

## Trials

- **Task:** the yaml default, "find the can and pick it up".
- **Block 1 (4-step server):** A_t1, B_t1, A_t2, B_t2 … A_t6, B_t6. Alternate the two conditions, and use can position n for trial n.
- **Block 2 (2-step server):** C_t1 … C_t6 on the same positions.
- **Trial end:** stop when the can is lifted clear, or after 45 s.

Record each trial in `$AB/trials.csv` with columns `trial,position,success,notes`, e.g. `B_t3,3,1,small head jitter at grasp`.

**What to watch:**
- Arm twitch or "pogo" at about 2.5 Hz, which means visible chunk boundaries. RTC (B and C) should reduce it compared with A.
- Hesitation or switching approach partway through a reach.
- Head jitter, base creep or overshoot, and the gripper closing at the right time.
- Hot or stalling servos. If anything looks unsafe, use the e-stop.

**Latency budget:** the new chunk's first executed action (index 2 = `latency_skip`) is planned for 133 ms after the observation. The bench round trip is about 96 ms at 4 steps and 82 ms at 2 steps, but the ROS stack has cost about 20 ms more on this Orin before. So 4-step may run 110–120 ms live, close to that 133 ms window. Round trips over 133 ms make each new chunk start slightly behind its plan. That shows up as bigger seam jumps in every condition, not as hold-and-wait, which needs a round trip of about 667 ms. Watch `late%` in the analysis. If it is well above 0 in block 1, part of any A/B difference is latency, not RTC. Record it; condition C (2-step) is the fix.

**Quick checks after the first trial of each condition:**
- B and C: the client log shows `[rtc] prefix #1: overlap=6 frozen=2`, and the server log shows `RTC #n` lines.
- A: neither appears.
- All conditions: the client banner shows `Action Chunk Size: 16`, `exec window: [2:6]` and `RTC: on/off` as expected.

## Analysis

```bash
cd ~/alfiebot_ws/src/alfie_gr00t/alfie_gr00t/scripts
~/Isaac-GR00T/.venv/bin/python rtc_ab_analysis.py \
  A=$(ls $AB/A_t*.csv | grep -v _actions | paste -sd,) \
  B=$(ls $AB/B_t*.csv | grep -v _actions | paste -sd,) \
  C=$(ls $AB/C_t*.csv | grep -v _actions | paste -sd,) \
  --dataset ~/alfiebot_ws/data/CleanTheRoom/lerobot --out $AB/summary.json
```

The script reports, per condition:
- **seam jump, step size and seam ratio:** the size of the jump where a new chunk takes over, compared with a normal step.
- **jerk RMS, and jerk relative to the demonstrations' own jerk.**
- **the same seam and jerk numbers for position joints only.**
- **RTT median / P95**, from `<run>_chunks.jsonl`.
- **late %:** the share of chunks whose round trip exceeded the 133 ms latency-skip window. Compare conditions only where late % is similar.
- **hold-and-wait %:** the share of time spent holding the last action while waiting for a chunk. It should be 0.
- **the fraction of chunks sent with an RTC prefix:** 0 for A, about 1 for B and C.
- **right-arm tracking error.**

All metrics use the raw policy output each step executed, before the EMA and rate limiter. They are defined as in Isaac-GR00T `scripts/deployment/rtc_smoothness_eval.py`, so they line up with the offline numbers:

| 4-step offline | seam jump | jerk RMS |
|---|---|---|
| RTC off | 0.0148 | 0.0391 |
| RTC on | 0.0074 | 0.0154 |

The offline run was open loop. On the robot the state follows what the policy did, so expect different absolute values, but the same direction.

Files per trial:

| File | Contents |
|---|---|
| `<run>.csv` | 100 Hz: action, smoothed and state per tick |
| `<run>_actions.csv` | One row per action step, raw chunk action |
| `<run>_chunks.jsonl` | Every server chunk, with RTT and whether a prefix was sent |
