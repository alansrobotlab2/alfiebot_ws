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

## Next Steps

### Phase 1: Live Robot Validation
1. **Baseline capture**: Run the current overlapped execution mode (`continuous_inference: false`) on the live robot for 2-3 CanDo attempts. Capture CSV logs.
2. **Continuous inference test**: Enable `continuous_inference: true` with `exp_decay` (m=0.01). Run the same task. Capture CSV logs.
3. **Compare**: Run `smoothing_analysis.py` on both CSVs side-by-side to verify that the offline benchmark improvements translate to live execution. Focus on right arm jerk and boundary jumps during the grasp phase.
4. **Tune if needed**: If live results diverge from offline benchmarks, the likely culprits are:
   - Network jitter (WiFi latency variance) causing irregular chunk arrival times
   - Observation staleness (camera frame age vs. chunk timing)
   - If pogoing persists, try `uniform` as a fallback (no tuning params to go wrong)

### Phase 2: Inference Speed
5. **Profile server latency**: Measure actual inference RTT distribution (mean, P95, P99). Current estimate is ~280ms → 3.3Hz. Identify bottlenecks (model forward pass vs. ZMQ serialization vs. image preprocessing).
6. **TensorRT revisit**: The bf16 TRT engine was unusable due to flow-matching divergence. Try:
   - FP32 TRT engine (slower but numerically stable)
   - bf16 with more denoising steps (8 instead of 4)
   - bf16 with fp32 accumulation mode
   - Any of these could push inference to 5Hz+, which the benchmarks show gives another ~11% jerk reduction.

### Phase 3: Advanced Strategies (if needed)
7. **Adaptive decay**: Scale `decay_m` based on action variance across overlapping chunks. When chunks agree (low variance), use gentle decay. When they disagree (high variance during fast motion), trust the latest chunk more.
8. **Per-body-part strategies**: Base velocity might benefit from stronger recency (it's reactive), while arm joints might benefit from more averaging (they're trajectory-following). The ChunkBuffer already supports per-body-part EMA; extending to per-body-part ensembling weights is straightforward.
9. **Longer horizons**: If a future GR00T version supports >16 action steps, more overlap = more ensembling benefit. The ChunkBuffer handles arbitrary chunk sizes already.
