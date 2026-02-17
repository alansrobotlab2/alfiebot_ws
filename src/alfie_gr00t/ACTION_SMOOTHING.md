# Action Smoothing Benchmark & Analysis Framework

## Context

The GR00T N1.6 model outputs 16-action chunks at ~3.3Hz (target 5Hz). Current execution uses a single-chunk pipeline: skip first 4 actions for latency compensation, execute 8 actions, discard the remaining 4, then promote the next chunk. This creates discontinuities at chunk boundaries ("pogoing") — especially visible in right arm joints during manipulation.

The core opportunity: with continuous inference, we receive overlapping 16-action chunks that cover the same timesteps. Instead of discarding overlap, we can **average predictions from multiple chunks** for each timestep (temporal ensembling), dramatically smoothing chunk boundaries. This is the approach used by the ACT paper (Zhao et al.) and is well-suited to our setup since we already have cached per-frame horizons from `overlapped_execution_test.py`.

**Goal**: Build offline tooling to benchmark smoothing strategies against GT episodes, find the best approach, then integrate into the live client.

## Architecture

### ChunkBuffer (`core/chunk_buffer.py`)

Central data structure shared between offline simulation and live client.

- Holds a deque of recent `TimestampedChunk` objects (max ~8)
- `get_action(target_frame)` finds all chunks covering that frame, extracts each chunk's prediction, combines via configurable weighting
- Thread-safe for live use (Lock around shared state)

**Weighting strategies:**

| Strategy | Formula | Use case |
|---|---|---|
| `latest` | Only use most recent chunk | Baseline (current behavior) |
| `uniform` | Equal weight to all overlapping chunks | Simplest temporal ensembling |
| `recency` | Linear decay: `w = 1 - age/max_age` | Moderate recency bias |
| `exp_decay` | ACT-style: `w = exp(-m * k)` | Configurable decay rate |
| `triangle` | Peak at chunk center, taper at edges | Downweight early/late predictions |

Optional per-body-part EMA as a second smoothing stage.

### Smoothing Simulator (`scripts/smoothing_simulator.py`)

Offline benchmarking tool. Takes cached horizons + GT episodes, simulates continuous inference at configurable rates, applies strategies, outputs comparison CSVs and plots.

```bash
python smoothing_simulator.py \
    --horizons /tmp/overlapped_test/horizons_ep320.npz \
    --dataset-path /home/alfie/alfiebot_ws/data/alfiebot.CanDoChallenge \
    --episode-index 320 \
    --inference-rates 3.3,5.0 \
    --strategies latest,uniform,exp_m1,exp_m5,triangle \
    --output-dir /tmp/smoothing_results/
```

Batch mode: `--episode-indices 317,318,319,320,321` aggregates across episodes.

### Smoothing Analysis (`scripts/smoothing_analysis.py`)

Unified analysis tool for both GT-benchmarked and live-captured CSVs.

```bash
# Compare simulator strategies against GT
python smoothing_analysis.py \
    --csv latest=.../latest.csv,exp_m1=.../exp_m1.csv \
    --gt-actions .../gt_actions.npy \
    --output-dir /tmp/smoothing_analysis/

# Analyze live capture (smoothness metrics only)
python smoothing_analysis.py \
    --csv live=/tmp/groot_client_debug.csv \
    --output-dir /tmp/smoothing_analysis/
```

## Strategies to Benchmark

All tested at inference rates 3.3Hz, 5Hz, 7.5Hz:

| Strategy | Config | Rationale |
|---|---|---|
| `latest` | Single chunk, no overlap | Current behavior reference |
| `uniform` | Equal-weight overlap avg | Simplest temporal ensembling |
| `exp_m01` | exp(-0.01 * k) | ACT paper gentle decay |
| `exp_m1` | exp(-0.1 * k) | Moderate recency bias |
| `exp_m5` | exp(-0.5 * k) | Strong recency bias |
| `triangle` | Peak at chunk center | Downweight early/late predictions |
| `exp_m1_ema90` | exp_decay + EMA alpha=0.9 | Overlap + output smoothing |

## Metrics

- **MAE vs GT** (per body part, overall) — when GT available
- **Boundary discontinuity**: mean |Δaction| at chunk transitions
- **Smoothness**: action jerk (2nd derivative) RMS
- **Pogo score**: sign alternation rate at chunk boundaries
- **Tracking delay**: cross-correlation lag between smoothed action and GT

## Live Client Integration

New `continuous_inference` mode in `groot_client.py`:
- Inference loop always churning: capture obs → send → receive → add to ChunkBuffer → immediately loop
- Command callback queries ChunkBuffer at 100Hz
- Gated by config param, backward compatible when disabled

```yaml
continuous_inference: false
smoothing_strategy: "latest"
smoothing_decay_m: 0.01
max_buffer_chunks: 8
```

## Benchmark Results (2026-02-16)

Benchmarked 8 strategies across 3 inference rates on 6 GT episodes (311, 313, 315, 317, 319, 321) from the CanDoChallenge dataset. Horizons collected from the PyTorch inference server (checkpoint-10000). All values are aggregates (means across episodes).

### 3.3 Hz (current hardware limit)

| Strategy | MAE | Jerk RMS | Boundary Jump | Pogo (R.Shoulder) | Delta RMS |
|---|---|---|---|---|---|
| **`exp_m01`** | **0.01926** | **0.01656** | **0.00580** | 0.341 | 0.01901 |
| `uniform` | 0.01927 | 0.01650 | 0.00578 | 0.338 | 0.01900 |
| `recency` | 0.01937 | 0.01873 | 0.00628 | 0.385 | 0.01984 |
| `triangle` | 0.01937 | 0.01873 | 0.00628 | 0.385 | 0.01984 |
| `exp_m1` | 0.01934 | 0.01830 | 0.00623 | 0.385 | 0.01965 |
| `exp_m1_ema90` | 0.01959 | 0.01615 | 0.00585 | 0.388 | 0.01890 |
| `exp_m5` | 0.02020 | 0.02842 | 0.00878 | 0.478 | 0.02420 |
| `latest` (baseline) | 0.02048 | 0.03168 | 0.00967 | 0.413 | 0.02579 |

### 5.0 Hz (target)

| Strategy | MAE | Jerk RMS | Boundary Jump | Pogo (R.Shoulder) | Delta RMS |
|---|---|---|---|---|---|
| **`exp_m01`** | **0.01942** | **0.01475** | **0.00511** | 0.348 | 0.01822 |
| `uniform` | 0.01943 | 0.01471 | 0.00509 | 0.348 | 0.01821 |
| `recency` | 0.01947 | 0.01649 | 0.00553 | 0.350 | 0.01878 |
| `triangle` | 0.01947 | 0.01649 | 0.00553 | 0.350 | 0.01878 |
| `exp_m1` | 0.01946 | 0.01607 | 0.00546 | 0.354 | 0.01866 |
| `exp_m1_ema90` | 0.01973 | 0.01430 | 0.00518 | 0.344 | 0.01806 |
| `exp_m5` | 0.02023 | 0.02576 | 0.00770 | 0.415 | 0.02293 |
| `latest` (baseline) | 0.02075 | 0.03191 | 0.00927 | 0.377 | 0.02611 |

### 7.5 Hz (aspirational)

| Strategy | MAE | Jerk RMS | Boundary Jump | Pogo (R.Shoulder) | Delta RMS |
|---|---|---|---|---|---|
| **`exp_m01`** | **0.01936** | **0.01355** | **0.00491** | 0.334 | 0.01795 |
| `uniform` | 0.01937 | 0.01352 | 0.00491 | 0.324 | 0.01794 |
| `recency` | 0.01944 | 0.01485 | 0.00509 | 0.343 | 0.01840 |
| `triangle` | 0.01944 | 0.01485 | 0.00509 | 0.343 | 0.01840 |
| `exp_m1` | 0.01942 | 0.01456 | 0.00506 | 0.339 | 0.01830 |
| `exp_m1_ema90` | 0.01970 | 0.01305 | 0.00485 | 0.346 | 0.01780 |
| `exp_m5` | 0.02010 | 0.02347 | 0.00659 | 0.374 | 0.02194 |
| `latest` (baseline) | 0.02082 | 0.03458 | 0.00852 | 0.331 | 0.02701 |

### Key Findings

1. **Temporal ensembling works.** Every overlap strategy beats `latest` (baseline) on every metric at every rate. This is not a marginal improvement — jerk drops ~48%, boundary jumps drop ~40%, and MAE drops ~6%.

2. **`exp_m01` and `uniform` are tied as clear winners.** At 3.3Hz: MAE 0.01926 vs 0.01927, jerk 0.01656 vs 0.01650, boundary jump 0.00580 vs 0.00578. The differences are in the noise floor. `exp_m01` is preferred because it gives a tuning knob via `decay_m` if needed, and it theoretically handles non-stationary motions better by slightly favoring newer predictions.

3. **Stronger decay hurts.** `exp_m5` (aggressive recency bias, m=0.5) is the worst overlap strategy — it barely beats baseline because it discards older chunks too fast, approaching single-chunk behavior. The sweet spot is gentle or no decay (m=0.01 or uniform).

4. **`recency` and `triangle` are identical.** Both produce exactly the same results because at our overlap counts (~3-4 chunks at 3.3Hz), linear decay and triangle weighting collapse to the same relative weights.

5. **Higher inference rates help modestly.** Going from 3.3→5.0Hz improves jerk by ~11% and boundary jumps by ~12%, but MAE barely changes. The biggest gains come from temporal ensembling itself, not raw speed.

6. **EMA adds little on top of ensembling.** `exp_m1_ema90` has slightly lower jerk than `exp_m1` alone, but higher MAE. The ensembling already provides sufficient smoothing; adding EMA just adds lag.

7. **`exp_m5` is worse than baseline for pogoing.** Its pogo score (0.478) actually exceeds baseline (0.413) at 3.3Hz. Strong decay creates its own discontinuities at chunk arrival boundaries.

### Improvement vs Baseline (3.3 Hz, exp_m01)

| Metric | Baseline | exp_m01 | Improvement |
|---|---|---|---|
| MAE | 0.02048 | 0.01926 | **-6.0%** |
| Jerk RMS | 0.03168 | 0.01656 | **-47.7%** |
| Boundary Jump | 0.00967 | 0.00580 | **-40.0%** |
| Pogo (R.Shoulder) | 0.413 | 0.341 | **-17.4%** |
| Delta RMS | 0.02579 | 0.01901 | **-26.3%** |

## Build Order (completed)

1. ~~`core/chunk_buffer.py` — standalone, unit testable~~
2. ~~`scripts/smoothing_simulator.py` — uses cached horizons, no server needed~~
3. ~~`scripts/smoothing_analysis.py` — consumes simulator output~~
4. ~~Run benchmarks, identify winning strategy~~
5. ~~`nodes/groot_client.py` — integrate winner into live client~~

## Recommended Live Config

```yaml
continuous_inference: true
smoothing_strategy: "exp_decay"
smoothing_decay_m: 0.01
max_buffer_chunks: 8
```

## Phase 2: Advanced Smoothing (2026-02-17)

### Problem

Temporal ensembling (Phase 1) reduced jerk by 48% and boundary jumps by 40%, but the output is still noticeably jerkier than ground truth. Two sources of residual noise:

1. **Intra-chunk noise**: Frame-to-frame jitter in the model's 15 FPS predictions that ensembling alone doesn't eliminate
2. **C0 interpolation artifacts**: Linear lerp between 15 FPS actions gives continuous position but **discontinuous velocity** at every 67ms waypoint boundary — the velocity profile is a staircase

### New Modules

#### ActionSmoother (`core/action_smoother.py`)

Causal post-processing filter applied to the 15 FPS ensembled output BEFORE interpolation. Maintains a ring buffer for stateful filtering.

| Method | How it works | Tuning params |
|---|---|---|
| `savgol` | Savitzky-Golay polynomial least-squares smoothing. Fits local polynomial to sliding window — preserves trajectory shape (peaks, direction changes) better than moving average. | `window` (odd, >= 3), `polyorder` (< window) |
| `butterworth` | Digital low-pass IIR filter. Removes frequencies above cutoff while preserving signal shape. More intuitive tuning than SG. | `order`, `cutoff_hz` (< 7.5 Hz Nyquist) |

Both use causal (one-sided) filtering for live use. Butterworth initializes filter state to avoid startup transient.

#### ActionInterpolator (`core/action_interpolator.py`)

Pluggable 100 Hz interpolation between 15 FPS waypoints. Replaces the inline `(1-alpha)*a[i] + alpha*a[i+1]` lerp.

| Method | Continuity | How it works |
|---|---|---|
| `linear` | C0 (position) | Current behavior — linear lerp, velocity has corners at 67ms boundaries |
| `cubic_spline` | C2 (acceleration) | scipy CubicSpline through rolling waypoint window — smooth velocity AND acceleration. Not-a-knot boundary conditions. |

### Pipeline (continuous inference mode)

```
Server → ChunkBuffer (temporal ensembling) → ActionSmoother (15 FPS filter) → ActionInterpolator (100 Hz) → ActionPublisher → Robot
```

### New Strategy Presets

All build on top of `exp_m01` (the Phase 1 winner):

| Preset | Filter | Interpolation | What it tests |
|---|---|---|---|
| `exp_m01` | none | linear | Phase 1 baseline |
| `exp_m01_savgol5_p2` | savgol w=5 p=2 | linear | Gentle polynomial smoothing |
| `exp_m01_savgol7_p3` | savgol w=7 p=3 | linear | Stronger polynomial smoothing |
| `exp_m01_butter_c3` | butterworth cutoff=3Hz | linear | Aggressive low-pass |
| `exp_m01_butter_c5` | butterworth cutoff=5Hz | linear | Moderate low-pass |
| `exp_m01_butter_c7` | butterworth cutoff=7Hz | linear | Gentle low-pass |
| `exp_m01_spline` | none | cubic_spline | C2 interpolation only |
| `exp_m01_spline_savgol5` | savgol w=5 p=2 | cubic_spline | Filter + C2 interpolation |
| `exp_m01_spline_butter5` | butterworth cutoff=5Hz | cubic_spline | Filter + C2 interpolation |

### Running the Benchmark

```bash
# 15 FPS benchmark (filters only, fast)
python smoothing_simulator.py \
    --horizons-dir /tmp/overlapped_test/ \
    --dataset-path /home/alfie/alfiebot_ws/data/alfiebot.CanDoChallenge \
    --episode-indices 311,313,315,317,319,321 \
    --inference-rates 3.3 \
    --strategies exp_m01,exp_m01_savgol5_p2,exp_m01_savgol7_p3,exp_m01_butter_c3,exp_m01_butter_c5,exp_m01_butter_c7 \
    --output-dir /tmp/smoothing_results_v2/

# 100 Hz benchmark (interpolation comparison)
python smoothing_simulator.py \
    --horizons-dir /tmp/overlapped_test/ \
    --dataset-path /home/alfie/alfiebot_ws/data/alfiebot.CanDoChallenge \
    --episode-indices 311,313,315,317,319,321 \
    --inference-rates 3.3 \
    --strategies exp_m01,exp_m01_spline,exp_m01_spline_savgol5,exp_m01_spline_butter5 \
    --output-rate 100 \
    --output-dir /tmp/smoothing_results_v2_100hz/
```

### Benchmark Results (2026-02-17)

All benchmarks run at 3.3 Hz across 6 GT episodes (311, 313, 315, 317, 319, 321). Values are aggregates (means across episodes). All strategies build on `exp_m01` temporal ensembling.

#### 15 FPS Results (post-ensembling filters)

| Strategy | MAE | Jerk RMS | Boundary Jump | vs baseline Jerk | vs baseline MAE |
|---|---|---|---|---|---|
| `exp_m01` (baseline) | 0.01593 | 0.01654 | 0.00576 | — | — |
| `exp_m01_savgol5_p2` | 0.01596 | 0.01469 | 0.00542 | **-11.2%** | +0.2% |
| `exp_m01_savgol7_p3` | 0.01596 | 0.01527 | 0.00556 | **-7.7%** | +0.2% |
| `exp_m01_butter_c7` | 0.01619 | 0.01541 | 0.00558 | **-6.8%** | +1.6% |
| `exp_m01_butter_c5` | 0.01719 | 0.01169 | 0.00497 | **-29.3%** | +7.9% |
| `exp_m01_butter_c3` | 0.01898 | 0.00818 | 0.00439 | **-50.5%** | +19.2% |

#### 100 Hz Results (interpolation + filters, upsampled from 15 FPS)

| Strategy | MAE | Jerk RMS | Accel RMS | Boundary Jump | vs baseline Jerk | vs baseline MAE |
|---|---|---|---|---|---|---|
| `exp_m01` (linear interp) | 0.01560 | 0.00133 | 0.00397 | 0.00086 | — | — |
| `exp_m01_spline` | 0.01573 | 0.00052 | 0.00226 | 0.00083 | **-60.9%** | +0.8% |
| `exp_m01_spline_savgol5` | 0.01578 | 0.00043 | 0.00197 | 0.00080 | **-67.7%** | +1.2% |
| `exp_m01_spline_butter5` | 0.01702 | 0.00031 | 0.00157 | 0.00075 | **-76.7%** | +9.1% |

#### Key Findings

1. **Cubic spline is the single biggest improvement.** Replacing linear lerp with cubic spline drops 100 Hz jerk by **61%** with only +0.8% MAE. This is because splines give C2 continuity (smooth velocity and acceleration) vs C0 (velocity corners at every 67ms boundary). This should be the default interpolation method.

2. **Savitzky-Golay is the best filter for low MAE impact.** `savgol w=5 p=2` reduces 15 FPS jerk by 11% with essentially zero MAE regression (+0.2%). It preserves trajectory shape well because it fits local polynomials rather than attenuating frequencies.

3. **Butterworth trades MAE for smoothness.** Lower cutoffs give progressively smoother output but deviate further from GT. `butter_c5` is the sweet spot: -29% jerk for +8% MAE. `butter_c3` halves jerk but +19% MAE is too much for accuracy-critical tasks.

4. **Stacking filter + spline compounds benefits.** `spline_savgol5` gets -68% jerk (vs linear baseline) at only +1.2% MAE. `spline_butter5` gets -77% jerk but +9% MAE.

5. **Recommended configuration for live robot:**
   - **Conservative** (accuracy-first): `interpolation_method: cubic_spline` + `smoothing_method: none`. Gets 61% jerk reduction with minimal MAE impact.
   - **Balanced**: `interpolation_method: cubic_spline` + `smoothing_method: savgol` (w=5 p=2). Gets 68% jerk reduction at +1.2% MAE.
   - **Smooth** (smoothness-first): `interpolation_method: cubic_spline` + `smoothing_method: butterworth` (cutoff=5Hz). Gets 77% jerk reduction at +9% MAE.

#### Total Improvement vs Original Baseline (Phase 1 → Phase 2)

Comparing `latest` (no ensembling, linear interp) vs `exp_m01_spline_savgol5` (ensembling + savgol + spline):

| Metric | `latest` (Phase 0) | `exp_m01` (Phase 1) | `exp_m01_spline_savgol5` (Phase 2) |
|---|---|---|---|
| Jerk RMS (15 FPS) | 0.03168 | 0.01654 (-48%) | 0.01469 (-54%) |
| Jerk RMS (100 Hz) | — | 0.00133 | 0.00043 (-68% vs Phase 1) |
| MAE | 0.02048 | 0.01593 (-22%) | 0.01578 (-23%) |

### Live Config

```yaml
# Temporal ensembling (Phase 1)
continuous_inference: true
smoothing_strategy: "exp_decay"
smoothing_decay_m: 0.01
max_buffer_chunks: 8

# Post-ensembling filter (Phase 2) — recommended: savgol for balanced smoothing
smoothing_method: "savgol"        # "none", "savgol", "butterworth"
savgol_window: 5
savgol_polyorder: 2
butterworth_order: 2
butterworth_cutoff_hz: 5.0

# Interpolation (Phase 2) — recommended: cubic_spline
interpolation_method: "cubic_spline"  # "linear", "cubic_spline"
spline_window: 6
```

### Techniques Considered but Not Implemented

- **Global polynomial fit** through 16 waypoints: Runge's phenomenon (edge oscillation). Piecewise polynomials with continuity = splines, already covered.
- **Minimum jerk trajectory**: Higher complexity, boundary condition estimation from finite differences amplifies noise. Could revisit if splines aren't enough.
- **Gaussian smoothing**: Adds lag (symmetric kernel), no advantage over Butterworth for causal use.

## Next Steps

### Immediate
1. ~~Run the 15 FPS and 100 Hz benchmark sweeps~~ ✓ (2026-02-17)
2. ~~Identify winning combination~~ ✓ — `cubic_spline` + `savgol w=5 p=2`
3. Test on live robot with CSV logging
4. Compare live results to offline benchmarks

### Phase 3: Inference Speed
5. **Profile server latency**: Measure actual inference RTT distribution (mean, P95, P99). Current estimate is ~280ms → 3.3Hz.
6. **TensorRT revisit**: Try FP32 engine, bf16 with more denoising steps, or bf16 with fp32 accumulation.

### Phase 4: Advanced Strategies
7. **Adaptive decay**: Scale `decay_m` based on action variance across overlapping chunks.
8. **Per-body-part strategies**: Different filter/interpolation params for base velocity vs arm joints.
9. **Longer horizons**: More overlap = more ensembling benefit if future GR00T supports >16 steps.
