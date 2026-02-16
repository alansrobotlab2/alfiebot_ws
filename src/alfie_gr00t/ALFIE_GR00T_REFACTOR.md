# GR00T Client-Server Refactoring Status

## Summary

This document tracks the refactoring of the alfiebot GR00T client-server. The initial refactor (Phases 1-4) is **complete** — the server uses `PolicyServer` + `JpegPolicyWrapper`, the client supports `n_action_steps`, and `ReplayPolicy` replaces custom replay logic. Remaining cleanup is documented below.

**Next major work:** Phase 5 adapts the client-server for **action_horizon=4** (AH=4) to match inference rate and achieve smooth closed-loop execution. Phase 6 moves inference **on-device** to eliminate the client-server split entirely and enable GPU pipeline optimizations.

---

## Completed Work

### Phase 1: Server Rewrite (DONE)

**File:** `scripts/groot_inference_server.py` — **1082 lines -> 694 lines**

The server now uses NVIDIA's `PolicyServer` for ZMQ/protocol and `Gr00tPolicy` for inference, with a thin `JpegPolicyWrapper` translation layer.

**Architecture:**
```python
policy = Gr00tPolicy(embodiment_tag=..., model_path=..., device=...)
wrapped = JpegPolicyWrapper(policy, language_key=...)
server = PolicyServer(wrapped, host=host, port=port)
# Custom run loop (PolicyServer.run() blocks with no timeout for clean shutdown)
```

**What was eliminated:**
- Custom ZMQ socket setup/teardown
- Custom msgpack serialization
- Custom request handling loop
- Custom action reassembly (body-parts -> 22D) — now in `JpegPolicyWrapper._reassemble_actions()`
- Custom mock mode
- Custom replay mode (replaced by `ReplayPolicy`)
- Custom stats tracking

**What was kept:**
- `TensorRTDiTWrapper` (~220 lines) — TRT support retained as option, though PyTorch backend is recommended (TRT bf16 causes numerical divergence)
- `JpegPolicyWrapper` (~130 lines) — translates JPEG wire format to native Gr00tPolicy format, reassembles body-part action dicts back to flat 22D for the client
- Periodic input/output logging (every 10th request)

**Deviation from plan:** The original plan estimated ~150 lines. Actual is 694 because TRT wrapper adds ~220 lines and the custom run loop with signal handling adds ~50 lines (needed because `PolicyServer.run()` blocks on `socket.recv()` with no timeout, preventing clean Ctrl-C shutdown).

**Server is now a standalone CLI script** (not a ROS2 node). `groot_server.yaml` is vestigial — the server uses command-line arguments instead:
```bash
python groot_inference_server.py --checkpoint /path/to/model --host 0.0.0.0
python groot_inference_server.py --dataset-path /path/to/dataset --episode-index 3
python groot_inference_server.py --checkpoint /path/to/model --torch-compile
```

### Phase 2: Client `n_action_steps` (DONE)

**File:** `nodes/groot_client.py` — **918 lines -> 785 lines**

- `n_action_steps` parameter added (default 16, configurable)
- `action_chunk_enabled` toggle removed (always true)
- Chunk blending logic removed from `_command_callback` (proven counterproductive)
- Inference loop waits for full `n_action_steps` before re-querying
- Post-exhaust observation capture preserved (model sees RESULT of its plan)
- Base velocity zeroing on chunk exhaust preserved
- Inter-action interpolation preserved

### Phase 3: ReplayPolicy (DONE)

The server uses NVIDIA's `ReplayPolicy` when `--dataset-path` is provided:
```python
policy = ReplayPolicy(
    dataset_path=args.dataset_path,
    modality_configs=modality_configs,
    execution_horizon=args.execution_horizon,
    strict=False,
)
```

This correctly advances by `execution_horizon` per call (the old custom replay advanced by +1, which was wrong).

### Phase 4: torch.compile (DONE)

Two compile options available via CLI flags:

```bash
# DiT compile (20-40% speedup on DiT forward pass)
python groot_inference_server.py --checkpoint /path --torch-compile

# Backbone compile (~10% additional speedup)
python groot_inference_server.py --checkpoint /path --compile-backbone
```

First inference after `--torch-compile` is slow (compilation), subsequent ones benefit from kernel fusion and CUDA graph optimizations.

### ZMQ Client Protocol Update (DONE)

**File:** `core/zmq_client.py`

The client now speaks the NVIDIA `MsgSerializer` protocol natively:
- Custom `_encode_custom`/`_decode_custom` functions are MsgSerializer-compatible (numpy arrays as `.npy` bytes)
- No `gr00t` package dependency needed on Jetson — wire format reimplemented in ~25 lines
- Supports both `send_observation()` (JPEG) and `send_raw_observation()` (raw RGB arrays)
- `_call_endpoint()` wraps requests in `{"endpoint": ..., "data": ...}` format matching PolicyServer

---

## Remaining Cleanup

### Dead Parameters in Client

Several parameters are declared, read, logged on startup, and present in config/launch files, but **never actually used** in the command callback or anywhere else:

| Parameter | Declared | Used? | Notes |
|-----------|----------|-------|-------|
| `latency_skip_base` | `groot_client.py:98` | **NO** | Read and logged but never referenced in `_command_callback`. The per-body-part latency skip logic was removed during refactoring. |
| `base_velocity_decay` | `groot_client.py:97` | **NO** | Read and logged but never applied. Was `v *= (1 - decay)` in the old code. |
| `max_joint_delta` | `groot_client.py:99` | **NO** | Read and logged but never passed to `publish_action()`. `ActionPublisher.publish_action()` has the parameter with default 0.3 but also doesn't call `safety.compute_delta_limits()`. |
| `_blend_from` | `groot_client.py:142` | **Partial** | SET when promoting a chunk (line 668) but never READ in the command callback. Dead write. |
| `_blend_steps` | `groot_client.py:141` | **NO** | Set to 3 but never read anywhere. |

**Files affected:**
- `nodes/groot_client.py` — remove dead parameter declarations, reads, and logging
- `config/groot_client.yaml` — remove `latency_skip_base` and `max_joint_delta` entries (keep `base_velocity_decay` if we plan to re-enable it)
- `launch/groot_inference.launch.py` — remove `base_velocity_decay` launch arg (already the only dead one exposed)

### Dead Safety Methods

`SafetyMonitor` has methods that are defined but never called by the client:
- `apply_joint_limits()` — never called (hardware enforces limits)
- `apply_velocity_limits()` — never called
- `apply_limits()` — never called
- `compute_delta_limits()` — never called

The client only calls `is_safe()` (e-stop + watchdog check), `update_inference_time()`, `record_failure()`, and `reset_failures()`. The limit methods exist for potential future use but could be removed to simplify.

### Vestigial `groot_server.yaml`

`config/groot_server.yaml` contains parameters (`model_checkpoint`, `use_tensorrt`, `mock_mode`, `action_horizon`, `device`, `dataset_path`, `episode_index`, `stats_path`) for the old ROS2 server node. The server is now a standalone CLI script and ignores this file entirely. The ROS2 server node (`nodes/groot_server.py`) may still reference it — check if that node is still used.

---

## n_action_steps: Current Status

The refactor plan's main thesis was that `n_action_steps < 16` would make the system more responsive. Testing revealed:

**n_action_steps=16 (current default):** Best for manipulation. The model's full 16-action trajectory plays out, giving coherent arm movements for grasping.

**n_action_steps < 16:** Causes arm trajectory repetition ("praying mantis" effect). The model sees a nearly identical scene each time (arm hasn't moved far enough in 0.53s) and outputs overlapping plans that fight each other.

**Root cause:** Unlike NVIDIA's sim environments where the scene changes significantly between observations (objects move, contacts happen), Alfie's real-world manipulation has slow dynamics. The arm hasn't completed its trajectory in 8 actions (0.53s), so the model re-plans the same motion from a similar starting state.

**Conclusion:** `n_action_steps=16` is correct for Alfie. The parameter exists and works, but the default should remain 16. The per-body-part latency skip (base reads ahead 3 actions) was the right approach for responsive base movement without disrupting arm trajectories — but it was removed during cleanup and should be reconsidered if base responsiveness needs tuning.

---

## Current Architecture (Post-Refactor)

### File Map

```
alfie_gr00t/
  core/
    action_publisher.py     361 lines  Publishes 22D actions to /alfie/robotlowcmd
    observation_bridge.py   409 lines  Syncs 4 cameras + state, resizes to 320x240
    zmq_client.py           441 lines  ZMQ REQ/REP client, MsgSerializer protocol
    normalization.py                   Stats-based normalization (offline analysis)
  nodes/
    groot_client.py         785 lines  Main ROS2 node: inference loop + 100Hz command
    groot_server.py                    Local server node (may be vestigial)
    data_recorder.py                   Data collection node
  scripts/
    groot_inference_server.py  694 lines  Standalone GPU inference server
    groot_open_loop_eval.py            Open-loop eval tool
    hybrid_image_test.py               Diagnostic tool for image domain gap
    rosbag_to_groot.py                 ROS2 bag -> LeRobot format converter
    ...
  utils/
    safety.py               241 lines  E-stop, watchdog, joint/velocity limits
config/
  groot_client.yaml                    Client ROS2 parameters
  groot_server.yaml                    (vestigial — server uses CLI args)
launch/
  groot_inference.launch.py            Launches client node only
```

### Data Flow

```
Cameras (320x240 JPEG) + Robot State
        |
  ObservationBridge
  - Decompress, resize to 320x240, BGR->RGB
  - Optional H.264 conditioning
  - Re-compress to JPEG Q95
  - Extract 22D state vector
        |
  ZMQClient (MsgSerializer protocol)
  - Send: {left_wide: jpeg, ..., state: [22D], language: str}
  - Endpoint: get_action
        |  WiFi (~130ms round trip)
        v
  PolicyServer (NVIDIA)
  - Routes to JpegPolicyWrapper._get_action()
        |
  JpegPolicyWrapper
  - Decode JPEG -> RGB numpy (1,1,H,W,3)
  - Split 22D state -> body-part dicts (1,1,D)
  - Format language -> {key: [[str]]}
        |
  Gr00tPolicy.get_action(native_obs)
  - Processor: eval transforms (SmallestMaxSize->CenterCrop->224x224)
  - Backbone: vision encoding
  - DiT: flow matching, 4 denoising steps
  - Action decoder -> body-part action dicts
        |
  JpegPolicyWrapper._reassemble_actions()
  - body-part dicts -> flat 22D x 16 actions
        |
  PolicyServer -> ZMQ -> ZMQClient
  - Response: (action_dict, info) tuple
  - Client extracts actions[0:16] as (16, 22) array
        |
  GrootClientNode._command_callback (100 Hz)
  - Steps through chunk at 67ms/action (15 FPS)
  - Inter-action interpolation for smooth output
  - Zero base velocity when chunk exhausted
        |
  ActionPublisher.publish_action()
  - Per-body-part EMA smoothing (base alpha=0.95, joints configurable)
  - Safety check (e-stop + watchdog)
  - Build RobotLowCmd message
  - Publish to /alfie/robotlowcmd
```

### Key Timing

```
Action step period:     67ms (1/15 FPS, matches training)
Chunk duration:         16 x 67ms = 1.07s (n_action_steps=16)
Inference latency:      ~130ms (PyTorch), ~80ms (torch.compile)
Inter-chunk gap:        ~130ms (observation capture + inference)
Command publish rate:   100 Hz (10ms period)
Full cycle:             ~1.2s (chunk + inference)
Effective inference Hz: ~0.83 Hz
```

### Tuning Parameters (Active)

| Parameter | Default | Where | Effect |
|-----------|---------|-------|--------|
| `n_action_steps` | 16 | client yaml | Actions executed before re-query. Keep at 16. |
| `action_smoothing_alpha` | 0.95 | client yaml | Joint EMA smoothing (base hardcoded to 0.95) |
| `interpolate_actions` | true | client yaml | Lerp between actions for smooth 100Hz |
| `back_init_height` | 0.1 | client yaml | Back position on startup (-1.0 to disable) |
| `task_description` | "find the can and pick it up" | client yaml | Language conditioning |
| `--torch-compile` | off | server CLI | torch.compile on DiT |
| `--compile-backbone` | off | server CLI | torch.compile on backbone |
| `--denoising-steps` | model default (4) | server CLI | Override denoising steps |

### Tuning Parameters (Declared but Unused — Dead Code)

| Parameter | Default | Where | Was Used For |
|-----------|---------|-------|-------------|
| `latency_skip_base` | 3 | client yaml | Base velocity read-ahead for latency compensation |
| `base_velocity_decay` | 0.0 (yaml) / 0.15 (launch) | client yaml/launch | Scaling down base velocity for approach tuning |
| `max_joint_delta` | 2.0 | client yaml | Per-step joint position change limit |

---

## Phase 5: Action Horizon = 4 — Matching Inference Rate

### Why AH=4

The AH=16 model predicts 1.07s into the future but only re-queries at ~0.83 Hz. Between queries, the robot is running blind on stale actions. Other GR00T implementations achieve smoother, more accurate execution by matching the action horizon to the inference cadence — the model plans exactly as far as it needs to before the next observation arrives.

With AH=4:
- **Chunk duration:** 4 × 67ms = **267ms** (vs 1.07s for AH=16)
- **Re-query rate:** ~3-4 Hz (vs 0.83 Hz) — 4x fresher observations
- **Less DiT compute:** sa_embs shrinks from ~17 tokens to ~5 tokens. DiT attention is O(n²) on tokens, so the action head runs significantly faster.
- **The model LEARNS shorter plans:** Unlike using `n_action_steps<16` with an AH=16 model (which caused "praying mantis" trajectory repetition), AH=4 training teaches the model to make coherent 267ms plans. Each plan is a complete thought, not a truncated 16-step trajectory.
- **Observation freshness:** Max observation staleness drops from ~1.2s to ~400ms.

Training config is already updated (`alfiebot_config.py`): `delta_indices=list(range(4))`.

### Timing Analysis

**The critical constraint:** inference latency must be less than chunk duration to avoid dead time between chunks. With overlapped inference (see below), the constraint relaxes to: inference must complete before the *next* chunk finishes.

| Config | Inference (est.) | Chunk | Fits? | Dead time |
|--------|-----------------|-------|-------|-----------|
| Remote 3090, PyTorch | ~120ms | 267ms | Yes | 0ms (overlapped) |
| Remote 3090, torch.compile | ~80ms | 267ms | Yes | 0ms (overlapped) |
| On-device Orin, baseline | ~600-800ms | 267ms | **No** | Needs Phase 6 optimizations |
| On-device Orin, optimized | ~200-300ms | 267ms | Tight | See Phase 6 |

**Note:** AH=4 reduces DiT compute (fewer action tokens to denoise), so inference should be somewhat faster than current AH=16 measurements. Exact speedup depends on how much of total inference is DiT vs backbone (backbone time is unchanged).

### The Overlapped Inference Architecture

**Current (AH=16, sequential):**
```
|===chunk N (1.07s)===|gap|===chunk N+1 (1.07s)===|
                       ^
                  capture obs
                  send inference (130ms)
                  install chunk
                  (130ms dead time — base holds, joints hold)
```

**New (AH=4, overlapped — double-buffered):**
```
Time:    0ms      120ms     267ms     387ms     534ms
         |         |         |         |         |
Chunks:  |--chunk A (267ms)--|--chunk B (267ms)--|--chunk C...
Infer:   |=obs+inf(120ms)=|  |=obs+inf(120ms)=|
         ^  result→pending   ^  result→pending
         obs for B           obs for C
         captured at         captured at
         chunk A start       chunk B start
```

**How it works:**
1. When chunk A starts playing, the inference thread immediately captures an observation and sends it to the server
2. ~120ms later, the result arrives and is stored as `_pending_chunk`
3. When chunk A finishes (t=267ms), `_command_callback` promotes pending → active (chunk B starts instantly)
4. The inference thread sees the pending slot is empty → immediately captures a new observation and sends again
5. Seamless: no dead time between chunks as long as inference < chunk_duration

**Key insight:** The observation for chunk B is captured at the START of chunk A (t=0), not at the END. With AH=16, we needed "post-exhaust observation" because 1.07s of motion means significant state change. With AH=4, only 267ms passes — the state barely changes, so a slightly stale observation is fine. The model plans only 267ms ahead from that state anyway.

**Observation staleness budget:**
- Observation captured at chunk N start
- Inference takes ~120ms
- Result sits in pending for ~147ms
- Chunk N+1 starts executing from observation that is now 267ms old
- Chunk N+1 finishes at 534ms after observation capture
- But the actions only target 267ms from the observed state — well within tolerance

### Client Changes Required

**1. Inference loop: wait on pending slot, not chunk exhaustion**

The single most critical change. Current code (`groot_client.py:560-570`):
```python
# Current: wait for chunk to finish, THEN start inference
if has_chunk:
    elapsed = time.monotonic() - chunk_ts
    remaining = chunk_duration - elapsed
    if remaining > 0:
        time.sleep(min(remaining, 0.05))
        continue
```

New behavior:
```python
# New: start inference as soon as pending slot is empty
with self._action_lock:
    has_pending = self._pending_chunk is not None
if has_pending:
    time.sleep(0.01)
    continue
# Capture obs immediately and send to server
```

This keeps the inference thread always one chunk ahead. As soon as the command callback promotes pending → active, the inference thread fires again.

**2. Chunk transition blending (re-enable `_blend_from`)**

With AH=16, transitions occur every 1.07s — infrequent enough that discontinuities are tolerable. With AH=4, transitions occur every 267ms (3.75 Hz). Any mismatch between chunk N's last action and chunk N+1's first action creates visible jitter at 3.75 Hz.

The `_blend_from` field is already SET when promoting chunks (line 668) but never READ. Re-enable it:
```python
# In _command_callback, after promoting pending → active:
if self._blend_from is not None and idx < self._blend_steps:
    blend_alpha = (idx + 1) / (self._blend_steps + 1)
    # Blend joints only (6:22), not base velocity (0:6)
    action[6:] = (1 - blend_alpha) * self._blend_from[6:] + blend_alpha * action[6:]
```

Start with `_blend_steps=2` (~134ms blend window). Test with 0, 1, 2, 3 and compare CSV logs for joint discontinuities at chunk boundaries.

**Why blending may work now:** With AH=16, blending fought the model's course corrections (model planned a new direction, blending pulled it back toward the old trajectory). With AH=4, the model re-plans every 267ms with a fresh observation — course corrections are smaller and more incremental, so blending smooths without fighting.

**3. Re-enable latency skip for base velocity**

With 120ms inference latency and 67ms action period, the observation is ~2 actions stale by the time actions execute. For base velocity (which directly controls position via integration), this lag causes overshoot.

Re-enable the latency skip that was removed during refactoring:
```python
# In _command_callback action selection:
base_idx = min(idx + self.latency_skip_base, len(chunk) - 1)
action[0:6] = chunk[base_idx, 0:6]  # base velocity reads ahead
# Joints use action[idx] as before (no skip)
```

With AH=4 and `latency_skip_base=1`: base reads 1 action ahead (67ms compensation). Start conservative — with 4 actions total, skip=2 leaves only 2 usable base actions before the chunk ends. Skip=1 is likely the right value.

**4. Re-enable base velocity decay**

With shorter chunks, the base moves less per chunk. But cumulative overshoot across many rapid chunks can still cause approach distance errors. Re-enable:
```python
action[0:6] *= (1.0 - self.base_velocity_decay)
```

Start at 0.0 (disabled) and tune empirically. The effect may be less necessary with AH=4 since the model gets fresh observations every 267ms and can self-correct.

**5. Parameter updates**

| Parameter | AH=16 value | AH=4 value | Reason |
|-----------|-------------|------------|--------|
| `action_chunk_size` | 16 | 4 | Match training horizon |
| `n_action_steps` | 16 | 4 | Execute all (full chunk) |
| `latency_skip_base` | 3 (dead) | 1 | 120ms / 67ms ≈ 2, but conservative |
| `base_velocity_decay` | 0.0 (dead) | 0.0 (tune) | Start disabled, tune empirically |
| `_blend_steps` | 3 (dead) | 2 | ~134ms blend at chunk transitions |
| Watchdog timeout | 2.07s | 0.7s | chunk_duration (0.267s) + 0.4s margin |

**6. First-chunk bootstrapping**

When inference activates, there's no pending chunk yet. The inference thread sends the first request, waits ~120ms, then installs the result as the active chunk (not pending). This is the same as the current code — no change needed, just verify it works with the shorter timing.

### Server Changes

**1. Action horizon auto-correction**

The server's `standalone_inference_script.py` already has logic to fix `action_horizon` mismatches:
```python
model_ah = policy.model.action_head.action_horizon
decode_ah = len(modality["action"].delta_indices)
if model_ah != decode_ah:
    policy.model.action_head.config.action_horizon = decode_ah
    policy.model.action_head.action_horizon = decode_ah
```

The inference server (`groot_inference_server.py`) should do the same after loading the AH=4 checkpoint. Without this, the model may generate 16 or 50 actions (base model's horizon) instead of 4.

**2. JpegPolicyWrapper update**

`_reassemble_actions()` currently builds `np.zeros((temporal_dim, 22))` and fills from action_dict. With AH=4, `temporal_dim` will be 4 instead of 16. No code change needed — it already reads the dimension dynamically. Just verify with the new checkpoint.

**3. ReplayPolicy execution_horizon**

When using `--dataset-path` for replay validation, pass `--execution-horizon 4` to match.

### Validation Plan

**Step 1: Open-loop eval**
- Run `groot_open_loop_eval.py` with the AH=4 checkpoint
- Verify: server returns 4 actions per request (not 16)
- Verify: trajectory plots match ground truth within tolerance
- Compare MSE/MAE to AH=16 checkpoint

**Step 2: Overlapped inference timing (no robot)**
- Add timing instrumentation to the inference loop
- Measure: time from chunk exhaustion to next chunk ready
- Target: <10ms gap (pending chunk pre-computed)
- Measure: observation staleness at chunk start

**Step 3: CSV log analysis (on robot)**
- Record CSV with `csv_log_path` enabled
- Check for: joint discontinuities at chunk boundaries (every 267ms)
- Check for: base velocity overshoot vs AH=16
- Check for: action trajectory coherence within each 4-step chunk

**Step 4: Blend tuning**
- Test blend_steps = 0, 1, 2, 3
- Plot joint positions across chunk boundaries in CSV
- Pick the value with smoothest transitions without sluggish response

**Step 5: Full CanDo challenge run**
- Compare: approach accuracy, grasp success, execution time
- Compare: smoothness of motion (video review)
- Compare: against AH=16 baseline

---

## Phase 6: On-Device Inference

### Goal

Run GR00T inference directly on-device, eliminating the client-server split. This removes WiFi latency, ZMQ serialization overhead, and JPEG encode/decode cycles — and enables GPU pipeline optimizations that are impossible across a network boundary.

**Hardware:** Gen2 Alfie hardware replaces the Orin NX 16GB with an **Orin AGX** — significantly more CUDA cores, more memory, and higher memory bandwidth. This makes on-device GR00T inference feasible where it would be marginal on the NX.

### Why This Matters

| Overhead | Current (remote) | On-device |
|----------|------------------|-----------|
| WiFi round-trip | 2-10ms (variable, jitter) | 0ms |
| JPEG encode (client) | ~2ms (4 cameras) | 0ms (pass RGB arrays directly) |
| JPEG decode (server) | ~2ms (4 cameras) | 0ms |
| Msgpack serialize | ~1ms | 0ms |
| Msgpack deserialize | ~1ms | 0ms |
| ZMQ socket overhead | ~1ms | 0ms |
| **Total saved** | **~10-20ms** | |

But the real win is enabling **GPU pipeline optimizations** that require both backbone and DiT to run on the same device:

### Proven Optimizations (benchmarked on 3090)

**Pipeline parallelism (backbone + DiT overlap) — COMPLETED:**

Double-buffered: backbone(N+1) runs on a separate CUDA stream while DiT(N) processes on the default stream. When DiT(N) finishes, backbone(N+1)'s features are already computed. Next DiT call starts immediately with zero wait.

| Config | Avg E2E | P90 E2E | Min E2E | Notes |
|--------|---------|---------|---------|-------|
| Baseline (flash + TRT DiT) | 274.6ms | 277.4ms | 260.6ms | Sequential |
| Pipeline only | 242.3ms | 260.8ms | 85.0ms | **11.8% faster** |
| torch.compile + pipeline | **213.7ms** | **229.2ms** | 83.7ms | **22.2% faster** |

Min of 85ms confirms overlap is working — that's roughly just DiT time when backbone was already running from the previous frame.

**Trade-off:** Pipeline adds 1-frame latency (frame N's actions use frame N-1's backbone features for the first DiT step). With AH=4 at 3.75 Hz, that's 267ms of extra visual latency. Acceptable for slow-changing scenes (can doesn't move between frames), but worth measuring impact.

### Additional Optimizations Available On-Device

| Optimization | Mechanism | Expected Impact |
|---|---|---|
| Async CPU prefetch | `ThreadPoolExecutor` runs observation preprocessing while GPU does inference | Hides ~15-33ms CPU work |
| AH=4 DiT reduction | sa_embs shrinks from ~17 to ~5 tokens | DiT ~40-60% faster (attention is O(n²)) |
| Reduced denoising (4→2) | Two fewer DiT passes | Saves ~36ms (2 × ~18ms/step) |
| cuDNN benchmark | Auto-selects fastest conv algorithms for fixed 224x224 input | ~5-10% backbone speedup |
| TRT backbone engine | Eagle backbone as TRT engine | Significant backbone speedup (backbone is the bottleneck at ~235ms) |
| torch.compile action encoder/decoder | Fuse `torch.bmm()` kernels in denoising loop | Small per-step savings, 4x amplified by denoising steps |
| Model distillation/pruning | Smaller backbone = less DRAM bandwidth | High effort, requires retraining. 50% smaller → ~2x faster |

### Architecture Change

**Current (client-server):**
```
ObservationBridge → JPEG encode → ZMQ → JPEG decode → JpegPolicyWrapper
  → Gr00tPolicy → JpegPolicyWrapper → ZMQ → ZMQ Client → ActionPublisher
```

**On-device:**
```
ObservationBridge → DirectPolicyBridge → Gr00tPolicy → ActionPublisher
```

`DirectPolicyBridge` replaces `ZMQClient` + `JpegPolicyWrapper`:
- Takes RGB arrays directly from `ObservationBridge.images_array` (no JPEG cycle)
- Splits 22D state → body-part dicts
- Formats observation in Gr00tPolicy native format
- Calls `policy.get_action()` directly (no ZMQ)
- Returns flat 22D actions (same interface as current ZMQ response)

The `GrootClientNode` switches between `ZMQClient` (remote) and `DirectPolicyBridge` (local) based on a config flag. `ActionPublisher` and `ObservationBridge` are unchanged.

### Memory Budget

**Orin AGX (Gen2 hardware):** 32GB or 64GB unified memory — ample headroom. GR00T model fits comfortably with room for TRT engines, PyTorch cache, and OS/ROS2 overhead. Memory management that was critical on the NX (60% cap, aggressive GC, skip_dit) becomes optional.

**Orin NX 16GB (current Gen1):** Tight but technically feasible. Rough budget:

| Component | Estimated Size |
|-----------|---------------|
| OS + ROS2 + other nodes | ~2-3 GB |
| GR00T backbone (Eagle3) | ~4-6 GB |
| GR00T DiT + action head | ~1-2 GB |
| TRT engines (if used) | ~1-2 GB |
| PyTorch CUDA cache | ~1-2 GB |
| Observation buffers | ~0.1 GB |
| **Total** | **~10-15 GB** |

NX memory management requirements:
- `torch.cuda.set_per_process_memory_fraction(0.6)` — cap PyTorch at 60%
- `skip_dit=True` when loading PyTorch model with TRT DiT replacement
- `garbage_collection_threshold:0.6` — aggressive GC
- **Note:** `expandable_segments:True` is BROKEN on Jetson (PyTorch 2.8 + CUDA 12.6 + r36.5)

### Inference Time Target

For seamless AH=4 execution on-device, inference must complete within one chunk duration: **267ms**.

**Estimated Orin AGX timeline (speculative — needs benchmarking):**

The Orin AGX has ~2-3x the CUDA compute of the NX (2048 vs 1024 CUDA cores on AGX 64GB, plus higher clocks and memory bandwidth). The 3090 benchmarks provide a rough upper-performance reference.

| Optimization Stack | Backbone | DiT (4 steps) | Total | Fits 267ms? |
|---|---|---|---|---|
| Baseline PyTorch | ~350ms | ~100ms | ~450ms | No |
| + torch.compile (both) | ~280ms | ~70ms | ~350ms | No |
| + pipeline parallelism | ~280ms | ~70ms | ~280ms* | Barely |
| + TRT backbone | ~150ms | ~70ms | ~170ms* | Yes |
| + AH=4 DiT reduction | ~150ms | ~40ms | ~160ms* | Yes |
| + 2 denoising steps | ~150ms | ~20ms | ~155ms* | Yes |

*Pipeline overlap means total ≈ max(backbone, DiT) + overhead, not sum.

**For comparison — Orin NX (Gen1, much tighter):**

| Optimization Stack | Backbone | DiT (4 steps) | Total | Fits 267ms? |
|---|---|---|---|---|
| Baseline PyTorch | ~500ms | ~150ms | ~650ms | No |
| + all optimizations | ~200ms | ~30ms | ~200ms* | Marginal |

**The path to 267ms:** On Orin AGX, pipeline parallelism + torch.compile should get close. Adding TRT backbone provides comfortable margin. On Orin NX, every optimization is required and the margin is thin — this is why Gen2 targets the AGX.

### Implementation Steps

1. **Benchmark on Orin AGX** — Run `standalone_inference_script.py` with AH=4 checkpoint on Gen2 hardware to establish baseline timings for backbone and DiT separately. This tells us exactly how far we need to optimize and which optimizations are worth the complexity.

2. **Build TRT engines for Orin AGX (SM87)** — The backbone (Eagle3) is the bottleneck. A TRT engine for SM87 could cut backbone time significantly. DiT TRT needs careful validation (bf16 divergence issue from Gen1 may or may not apply to AGX).

3. **Create `DirectPolicyBridge`** — Implement the direct inference path that replaces ZMQ. Should be ~50 lines: format observation, call `policy.get_action()`, extract flat 22D.

4. **Wire up pipeline parallelism** — Port `PipelinedInference` from `standalone_inference_script.py` into the client node. Backbone runs on a background CUDA stream, DiT on the default stream.

5. **Integrate into `GrootClientNode`** — Add `inference_mode` parameter: `"remote"` (ZMQ, current) or `"local"` (DirectPolicyBridge). The inference loop and command callback don't change — only the transport layer swaps.

6. **Benchmark and tune** — Measure E2E on Orin AGX with each optimization stacked. Find the minimum config that hits 267ms.

---

## Reference Files (NVIDIA)

| File | What it provides |
|------|-----------------|
| `gr00t/eval/run_gr00t_server.py` | Canonical server — PolicyServer + Gr00tPolicy |
| `gr00t/eval/rollout_policy.py` | Canonical client loop — PolicyClient + MultiStepWrapper |
| `gr00t/policy/server_client.py` | PolicyServer/PolicyClient ZMQ protocol, MsgSerializer |
| `gr00t/policy/gr00t_policy.py` | Gr00tPolicy with prepare_inputs/run_inference split |
| `gr00t/policy/replay_policy.py` | ReplayPolicy with correct execution_horizon stepping |
| `gr00t/eval/sim/wrapper/multistep_wrapper.py` | MultiStepWrapper — n_action_steps concept |
| `scripts/deployment/standalone_inference_script.py` | torch.compile, async prefetch, TRT wrappers |
