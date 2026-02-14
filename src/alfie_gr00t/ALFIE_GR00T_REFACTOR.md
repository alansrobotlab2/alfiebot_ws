# GR00T Client-Server Refactoring Plan

## Context

The current alfiebot GR00T client-server is a custom implementation that diverges significantly from NVIDIA's reference architecture. After reading the **canonical reference** (`run_gr00t_server.py`, `rollout_policy.py`, `server_client.py`, `replay_policy.py`, `MultiStepWrapper`), the differences are stark:

**NVIDIA's reference is 111 lines for the server. Ours is 1082.**

The reference server is just:
```python
policy = Gr00tPolicy(embodiment_tag=..., model_path=..., device=...)
server = PolicyServer(policy=policy, host=..., port=...)
server.run()
```

Everything else — JPEG decode, state splitting, observation formatting, action reassembly, TRT wrappers, replay mode, mock mode, visualization — is custom code we wrote that either duplicates `Gr00tPolicy` internals or adds complexity for marginal benefit.

### Critical Insight: `n_action_steps` (execution horizon) vs `action_horizon` (prediction horizon)

The `rollout_policy.py` and `MultiStepWrapper` reveal a crucial concept we've been reimplementing incorrectly:

- **`action_horizon`** = 16 (model predicts 16 future steps, set by `delta_indices`)
- **`n_action_steps`** = number of actions **actually executed** before re-querying the policy (default 8 in rollout, configurable)
- `ReplayPolicy` explicitly separates these: `assert execution_horizon <= action_horizon`
- `MultiStepWrapper.step()` loops `for step in range(n_action_steps)` — executing only the first N actions of the 16-step chunk, then re-observing and re-querying

**Our implementation always executes all 16 actions before re-querying.** This means we run the full 1.07s trajectory before getting new observations. The reference supports executing fewer steps (e.g., 8 = 0.53s at 15 FPS) and re-querying more frequently with fresher observations. This is a fundamental architectural difference.

### What the reference architecture does

1. **Server** (`run_gr00t_server.py` → `PolicyServer`):
   - Wraps `Gr00tPolicy` (or `ReplayPolicy`) with `PolicyServer`
   - `PolicyServer` handles ZMQ REP, msgpack serialization via `MsgSerializer`, endpoint routing
   - `get_action` endpoint just calls `policy.get_action(observation)` directly
   - Observation arrives pre-formatted (nested dict with video/state/language keys, numpy arrays)
   - Returns action dict + info dict directly via `MsgSerializer`

2. **Client** (`rollout_policy.py` → `PolicyClient` + `MultiStepWrapper`):
   - `PolicyClient` implements `BasePolicy` interface — drop-in for `Gr00tPolicy`
   - `MultiStepWrapper` handles action chunking: takes N-step action from policy, executes `n_action_steps` in the env, returns observation
   - The client **does not** manage action chunks, interpolation, or timing — the `MultiStepWrapper` gym env does it

3. **Data format**:
   - `MsgSerializer` sends numpy arrays as `.npy` (not JPEG). Works for local/sim, not WiFi
   - Observation format: `{video: {key: (B,T,H,W,C) uint8}, state: {key: (B,T,D) float32}, language: {key: [[str]]}}`
   - Action format: `{key: (B,T,D) float32}` — split by body part, NOT a flat 22D vector

---

## FPS Mismatch Analysis: 15 FPS Training vs 2.25-4 Hz Inference

### The Concern
The model was fine-tuned on a 15 FPS dataset (`delta_indices=list(range(16))` → 16 contiguous frames → 1.07s trajectory per inference). Hardware can only run inference at 2.25 Hz (current) to ~4 Hz (theoretical max after optimization). Does this mismatch cause problems? Should we retrain at a lower FPS?

### How `delta_indices` Works
In `sharded_single_step_dataset.py:26`:
```python
indices_to_load = [step_index + delta_index for delta_index in config.delta_indices]
```
These are pure frame indices. With a 15 FPS dataset, each index = 67ms of real time. The 16-action prediction spans 1.07s. There is **no FPS parameter in training** — temporal resolution is entirely determined by the dataset's recording rate.

### Answer: No Retraining Needed — Action Chunking Handles This By Design

**Training FPS and inference frequency are intentionally decoupled by action chunking.** This is the entire purpose of predicting 16-step trajectories:

1. **The model predicts a temporally dense trajectory** (16 actions at 67ms each = 1.07s)
2. **The client plays the trajectory at the training rate** (100Hz timer steps through actions at 67ms intervals, with inter-action interpolation for smooth output)
3. **Inference happens once per trajectory** — the inference frequency (~0.8-0.9 Hz currently) is independent of the action execution rate

**Current effective operation:**
- Chunk plays out: 16 actions × 67ms = 1.07s (or ~0.87s with latency_skip=3)
- Inference latency: ~130-444ms depending on optimization
- Total cycle: ~1.2-1.5s per inference → ~0.7-0.8 Hz inference rate
- This is normal and by design

**NVIDIA's own deployments confirm this pattern:**
- SO100 real robot eval: `action_horizon=8` at 30 FPS → 267ms execution, inference far slower than 30 Hz
- Sim evals: `n_action_steps` ranges from 1 (6.25% of horizon) to 20 (67% of 30-step horizon)
- `MultiStepWrapper` exists specifically to execute a subset of the predicted horizon

**Why lower FPS retraining would be counterproductive:**
- At 4 FPS, each action = 250ms, 16 actions = 4 seconds. Manipulation (grasping) needs finer temporal resolution than 250ms waypoints
- Non-contiguous `delta_indices` (e.g., `[0, 2, 4, ...]`) is untested in the codebase. The pretrained backbone was trained with contiguous sequences — changing this could degrade representations
- The current 15 FPS × 16 actions = 1.07s trajectory is a good balance: dense enough for manipulation, long enough that even slow inference doesn't starve the action pipeline

### Where the Real Optimization Opportunity Lies

The issue isn't FPS mismatch — it's that **executing all 16 actions before re-querying makes the system less responsive**. With `n_action_steps < 16`, we can:
- Execute 8 actions (0.53s) then re-query with a fresh observation
- React faster to environment changes (can moved, approach error accumulation)
- Still benefit from the full 16-step prediction (the model "plans" further than we execute)

This is addressed in Phase 2 below.

---

## Plan

### Phase 1: Server — Replace with `PolicyServer` + thin JPEG preprocessing layer

**Goal:** Reduce server from 1082 lines to ~150 lines by using NVIDIA's `PolicyServer` for ZMQ/protocol and `Gr00tPolicy` for inference.

**File:** `scripts/groot_inference_server.py` (rewrite)

The fundamental insight: `PolicyServer` expects observations in Gr00tPolicy's native format (nested dicts with numpy arrays). Our client sends JPEG bytes + flat 22D state over WiFi. We need a thin **translation layer** between the JPEG-over-msgpack transport and PolicyServer's native format.

**Approach:** Create a `JpegPolicyWrapper` that wraps `Gr00tPolicy`, accepts JPEG bytes + flat state, and translates to the native format before calling the underlying policy:

```python
class JpegPolicyWrapper(BasePolicy):
    """Wraps Gr00tPolicy to accept JPEG-encoded observations over network.

    Translates from wire format:
      {images: {key: jpeg_bytes}, state: [22D flat], language: str}
    To Gr00tPolicy format:
      {video: {key: (1,1,H,W,3) uint8}, state: {key: (1,1,D) float32}, language: {key: [[str]]}}
    """
    def __init__(self, policy: Gr00tPolicy):
        super().__init__(strict=False)
        self.policy = policy

    def _get_action(self, observation, options=None):
        # Decode JPEGs → numpy RGB
        # Split flat 22D state → body part dicts
        # Format language
        # Call self.policy.get_action(formatted_obs)
        # Return action dict (already in body-part format)
```

Then the server becomes:
```python
policy = Gr00tPolicy(embodiment_tag=..., model_path=..., device=...)
wrapped = JpegPolicyWrapper(policy)
server = PolicyServer(wrapped, host=host, port=port)
server.run()
```

**What this eliminates:**
- Custom ZMQ socket setup/teardown (~50 lines)
- Custom msgpack serialization (~30 lines)
- Custom request handling loop with error recovery (~140 lines)
- Custom action reassembly from body parts → 22D (~50 lines) — PolicyServer returns body-part dicts directly
- Custom TRT wrapper (200+ lines) — use `standalone_inference_script.py`'s version or skip TRT
- Custom mock mode (~20 lines)
- Custom replay mode (~80 lines) — use `ReplayPolicy` from NVIDIA
- Custom stats tracking (~30 lines)
- Custom visualizer integration (~20 lines)

**What we keep (in JpegPolicyWrapper):**
- JPEG decode + BGR→RGB (~10 lines)
- Flat 22D state → body part split (~15 lines)
- Language string → nested list format (~3 lines)

**Additional server improvements:**
- Add `torch.compile` on model load (from `standalone_inference_script.py:1111`)
- The client will need to adapt to receiving body-part action dicts instead of flat 22D arrays (or `JpegPolicyWrapper` can reassemble to 22D in a custom response — see Phase 2)

### Phase 2: Client — Adapt to `n_action_steps` concept + simplify

**File:** `nodes/groot_client.py`

**Key change:** Instead of always executing all 16 actions, make `n_action_steps` configurable (default to current behavior of 16, but allow e.g., 8 for more responsive closed-loop).

The `MultiStepWrapper` pattern from NVIDIA shows the correct abstraction:
1. Get 16-action chunk from policy
2. Execute first `n_action_steps` of them at the training rate
3. Re-observe and re-query

This is essentially what our inference loop already does with `effective_actions = action_chunk_size - latency_skip_actions`. The latency skip was our ad-hoc version of `n_action_steps < action_horizon`. With proper `n_action_steps`, we can:
- Execute 8 actions (0.53s at 15 FPS) then re-query with fresh observation
- No need for latency skip hack — the model gets a fresh observation every 0.53s instead of every 1.07s
- More responsive to environment changes (can closer, object moved, etc.)

**Specific changes:**

1. **Add `n_action_steps` parameter** (rename/replace `latency_skip_actions`):
   - `n_action_steps = 16` preserves current behavior (execute full chunk)
   - `n_action_steps = 8` executes half, then re-queries (more responsive)
   - Inference loop waits until `n_action_steps` are consumed, then re-queries

2. **Remove chunk blending code** (currently disabled, proven counterproductive):
   - Remove `_prev_chunk_tail` field
   - Remove `chunk_blend_actions` parameter
   - Remove blend logic from `_command_callback` (lines 809-819)

3. **Remove `action_chunk_enabled` toggle** (always True, dead code path)

4. **Simplify latency skip**:
   - With shorter execution horizons (n_action_steps=8), latency skip becomes less critical
   - Keep it as an option but default to 0 when n_action_steps < action_horizon

5. **Adapt to body-part action dicts** (if server returns native format):
   - Either: `JpegPolicyWrapper` reassembles to flat 22D on server side (simpler client change)
   - Or: Client unpacks body-part dicts → flat 22D (cleaner protocol but more client changes)
   - **Recommendation:** Keep flat 22D over the wire for now. The `JpegPolicyWrapper._get_action()` returns body-part dicts from `Gr00tPolicy`, but we add a post-processing step in the wrapper to concatenate back to flat 22D before `PolicyServer` serializes the response. This minimizes client changes.

### Phase 3: Use NVIDIA's `ReplayPolicy` for replay mode

**Current:** Custom `_replay_inference()` in server (80 lines) with incorrect step advancement (+1 instead of +action_horizon).

**Fix:** Use `ReplayPolicy` from `gr00t/policy/replay_policy.py`:
```python
if dataset_path:
    policy = ReplayPolicy(
        dataset_path=dataset_path,
        modality_configs=modality_configs,
        execution_horizon=n_action_steps,  # advances correctly
    )
else:
    policy = Gr00tPolicy(...)

wrapped = JpegPolicyWrapper(policy)  # JpegPolicyWrapper works with any BasePolicy
server = PolicyServer(wrapped, host=host, port=port)
```

`ReplayPolicy` already:
- Loads from LeRobot parquet
- Advances by `execution_horizon` per call (correct!)
- Handles end-of-episode padding
- Returns body-part action dicts in correct format
- Supports episode switching via `reset(options={'episode_index': N})`

### Phase 4: `torch.compile` for inference speedup

**File:** `scripts/groot_inference_server.py` (new version)

After loading `Gr00tPolicy`:
```python
policy.model.action_head.model.forward = torch.compile(
    policy.model.action_head.model.forward, mode="max-autotune"
)
```

From `standalone_inference_script.py:1111-1113`. First inference is slow (compilation), subsequent ones benefit from kernel fusion and CUDA graph optimizations.

**Expected impact:** The DiT forward pass is ~115ms of ~235ms total on 3090. `torch.compile` with `max-autotune` can reduce this by 20-40%.

---

## Files to Modify

| File | Action | Lines Before → After (est.) |
|------|--------|----------------------------|
| `scripts/groot_inference_server.py` | Rewrite | 1082 → ~150 |
| `nodes/groot_client.py` | Simplify | 918 → ~750 |
| `config/groot_client.yaml` | Update params | Minor |
| `launch/groot_inference.launch.py` | Update launch args | Minor |

## Files NOT Modified (preserved as-is)

| File | Reason |
|------|--------|
| `core/action_publisher.py` | Sound, handles 22D → RobotLowCmd + EMA + safety |
| `core/observation_bridge.py` | Sound, handles camera sync + resize + JPEG encode |
| `core/zmq_client.py` | Still needed for JPEG transport (PolicyClient uses MsgSerializer which sends raw numpy — too much bandwidth over WiFi) |

## Preserved Tuning Parameters

These were validated through extensive live robot testing and remain in the client:
- `action_chunk_size=16` — must match training horizon
- `base_velocity_decay=0.15` — fine-tunes approach distance
- `action_smoothing_alpha=0.95` — near pass-through EMA
- `interpolate_actions=true` — smooth 100Hz output within chunks
- Post-exhaust observation capture — model sees RESULT of its plan
- Base velocity zeroing on chunk exhaust — prevents coasting overshoot
- Per-body-part latency skip (base only) — kept as option, may become unnecessary with shorter n_action_steps

## New Tuning Parameter

- `n_action_steps` (default 16, can try 8) — how many of the 16 predicted actions to execute before re-querying. Lower = more responsive but higher inference load.

**Timing at different `n_action_steps` values (with ~130ms inference latency after torch.compile):**

| n_action_steps | Execution time | + Inference | Cycle time | Inference Hz | Notes |
|---|---|---|---|---|---|
| 16 | 1.07s | 130ms | ~1.2s | ~0.83 Hz | Current behavior, least responsive |
| 12 | 0.80s | 130ms | ~0.93s | ~1.08 Hz | |
| 8 | 0.53s | 130ms | ~0.66s | ~1.52 Hz | NVIDIA's default in sim evals |
| 4 | 0.27s | 130ms | ~0.40s | ~2.5 Hz | Very responsive, high inference load |

**Recommendation:** Start with `n_action_steps=8` (matches NVIDIA's sim eval default). If manipulation precision suffers from stale observations, try 4. If inference load is too high, increase to 12.

---

## Verification

### 1. Server rewrite
- Start new server with same checkpoint
- Run `hybrid_image_test.py` Test 7 (consistency check) — must match baseline
- Run `groot_open_loop_eval.py` — trajectory plots must match pre-refactoring

### 2. torch.compile
- Measure inference time before/after over 100 requests
- Expect 20-40% reduction in DiT forward pass time

### 3. Client n_action_steps
- Test with n_action_steps=16 first (should match current behavior exactly)
- Then test n_action_steps=8 for comparison (more responsive, higher inference load)
- CSV log comparison: timing, action values, chunk boundaries

### 4. Replay mode
- Replay episode via new server (using ReplayPolicy) vs old server
- Verify chunks advance by execution_horizon, not by 1
- Compare robot trajectory

### 5. End-to-end
- Full CanDo challenge run with refactored code
- Compare with previous successful runs

---

## Reference Files (NVIDIA)

| File | What it provides |
|------|-----------------|
| `gr00t/eval/run_gr00t_server.py` | **Canonical server** — 111 lines, PolicyServer + Gr00tPolicy |
| `gr00t/eval/rollout_policy.py` | **Canonical client loop** — PolicyClient + MultiStepWrapper |
| `gr00t/policy/server_client.py` | PolicyServer/PolicyClient ZMQ protocol |
| `gr00t/policy/gr00t_policy.py` | Gr00tPolicy with prepare_inputs/run_inference split |
| `gr00t/policy/replay_policy.py` | ReplayPolicy with correct execution_horizon stepping |
| `gr00t/eval/sim/wrapper/multistep_wrapper.py` | MultiStepWrapper — n_action_steps concept |
| `scripts/deployment/standalone_inference_script.py` | torch.compile, async prefetch, TRT wrappers |
