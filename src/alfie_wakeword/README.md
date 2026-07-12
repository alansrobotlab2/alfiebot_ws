# alfie_wakeword

Wake-word detection for Alfie, built on [openWakeWord](https://github.com/dscripka/openWakeWord).
It gates the conversation (Alfie only listens when addressed), drives LED feedback,
and provides barge-in (interrupting Alfie mid-reply). Runs **fully offline**.

---

## What it does

- Runs custom wake-word models over the reSpeaker's AEC-cleaned mic stream.
- **`hey alfie` / `alfie`** → open a conversation window so the agent processes the
  next transcript(s). A follow-up window keeps it open for hands-free back-and-forth.
- **`stop`** → a *cancel* word: barges in / aborts the reply, without opening a window.
- **Energy-gate barge-in** → speak up over Alfie and he stops (see [Barge-in](#barge-in)).
- Every detection is echoed for LED feedback: **green flash** for a wake word,
  **red flash** for a cancel / barge-in.

## Pipeline / topics

```
audio_publisher ──audio_frames──▶ wakeword_node ──wake (String phrase)──▶ agent_node (opens window)
                                       │        ──wakeword/detection (String key)──▶ led_behavior (flash)
                                       └────────── barge_in (Empty) ──▶ alfie_tts (abort) + agent_node (cancel)
```

- `audio_frames` — `alfie_msgs/AudioFrame`, int16[512] @ 16 kHz (~32 ms), BEST_EFFORT.
  Accumulated into openWakeWord's 1280-sample (80 ms) hops.
- `wake` — `std_msgs/String` (spoken phrase), RELIABLE. Opens the agent window.
- `wakeword/detection` — `std_msgs/String` (raw model key), RELIABLE. LED feedback only.
- `barge_in` — `std_msgs/Empty`, BEST_EFFORT. Aborts TTS + cancels the in-flight turn.
- `speaking` — `alfie_msgs/Speaking` in; tells the node when TTS is playing.

## Models

Bundled in [`models/`](models/) and loaded **by path** (never downloaded at runtime):

| File | Role | Default threshold |
|---|---|---|
| `hey_alfie.onnx` | wake word | 0.45 |
| `alfie.onnx` | wake word (short → noisier) | 0.45 |
| `stop.onnx` | cancel word | 0.25 |
| `melspectrogram.onnx`, `embedding_model.onnx` | oww base models | — |

Custom models are the openWakeWord classifier format: input `x [1,16,96]` → `sigmoid [1,1]`.
Single-file `.onnx` preferred; if a model ships as `.onnx` + external `.onnx.data`, both
are bundled (the `setup.py` `glob('models/*')` handles it).

**Swapping in a retrained model:** drop the new `.onnx` in `models/`, rebuild
`alfie_wakeword`, restart the node. Keep the same filename for a zero-code swap.

## Offline operation

The whole voice stack boots with no network:
- **Wake models** — bundled here, loaded by path.
- **ASR (Parakeet)** — `HF_HUB_OFFLINE=1` in `parakeet_asr_node` resolves the model from
  the local HF cache.
- **VAD (Silero)** — the `.onnx` ships inside the `silero-vad` pip wheel.
- **TTS (Piper)** — bundled voice in `alfie_tts`.

## Wake-gated conversation (in `alfie_agent/agent_node.py`)

- Transcripts are dropped unless a wake window is open.
- A wake opens a window; it extends on each accepted user command (not on Alfie's own
  replies, so background can't hold it open). ~8 s; then the wake word is required again.
- The leading wake phrase is stripped from the command; a bare wake ("Hey Alfie") or a
  very short mis-heard wake right after a detection is acknowledged, not answered.

## LED feedback (in `alfie_mic/led_behavior.py`)

| Event / state | Ring |
|---|---|
| Wake detected (`hey_alfie`/`alfie`) | brief **green** flash |
| Cancel / barge-in (`stop`, `bargein`) | brief **red** flash |
| Listening (window open) | DOA — points at the speaker |
| Thinking (LLM generating) | cyan↔purple crossfade |
| Speaking (TTS) | cyan pulse tracking voice amplitude |
| Idle | dim solid |

---

## Barge-in

Interrupting Alfie while he's talking. This was explored in depth; findings below.

### Why wake-word barge-in doesn't work (yet)
Saying "Alfie"/"stop" *over* TTS does **not** reliably detect. Investigation:

- The reSpeaker's **AEC works well** — during TTS, Alfie's own voice is cancelled to
  ~250 RMS residual (below ambient). The mic is *not* ducked.
- Sweeping `PP_ECHOONOFF`, `PP_NLATTENONOFF`, `PP_AGCONOFF` in every combination did **not**
  restore wake detection during playback.
- Cause: during double-talk the wake word is **spectrally smeared** with Alfie's residual
  voice, and the **beamformer steers toward Alfie's own speaker**, attenuating the user's
  off-axis voice. So oww (trained on clean audio) can't match it.

### Energy-gate barge-in (current solution)
Because the AEC keeps the mic quiet unless the user speaks, a **sustained loud mic during
TTS = the user interrupting** — no wake-word match needed. Implemented in `wakeword_node`:
while `speaking`, if mic RMS stays above `barge_energy_threshold` for `barge_hold_hops`
hops (80 ms each) → publish `barge_in`.

Measured levels (this workshop, with a background TV):

| Signal (during TTS) | mic RMS |
|---|---|
| AEC residual (Alfie only) | ~250 |
| Background TV | up to ~2400 |
| User interrupting (normal → speak-up) | ~800–3100, peaks 4000–11000 |

**Tuning is environment-dependent** because the user's voice and background overlap:
- **Quiet room:** `barge_energy_threshold ≈ 1200` → natural normal-voice barge-in.
- **This room (TV):** `≈ 2800`, `barge_hold_hops = 2` → a deliberate speak-up clears the
  background. (Baked as the default.)

### The robust upgrade (future)
Retrain the `alfie` / `stop` models with **Alfie's TTS mixed into the positive samples**,
so oww's *pattern* matching (not just energy) can catch the wake word during double-talk.
Desktop training work.

---

## Live tuning (no rebuild)

`wakeword_node` (node `/alfie/wakeword_node`):

```bash
# per-model detection thresholds
ros2 param set /alfie/wakeword_node thr_hey_alfie 0.45
ros2 param set /alfie/wakeword_node thr_alfie 0.45
ros2 param set /alfie/wakeword_node thr_stop 0.25
# energy-gate barge-in (threshold MUST be a float, e.g. 1200.0)
ros2 param set /alfie/wakeword_node barge_energy_threshold 2800.0
ros2 param set /alfie/wakeword_node barge_hold_hops 2
ros2 param set /alfie/wakeword_node barge_enabled true
```

`respeaker_control` (node `/alfie/respeaker_control_node`) — reSpeaker AEC / DSP:

```bash
ros2 param set /alfie/respeaker_control_node pp_echo 1        # linear AEC on/off
ros2 param set /alfie/respeaker_control_node pp_nlatten 1     # non-linear residual suppressor
ros2 param set /alfie/respeaker_control_node pp_agc_onoff 1   # auto gain control
ros2 param set /alfie/respeaker_control_node ref_gain 0.5     # AEC reference gain (float; -1 = leave default)
ros2 param set /alfie/respeaker_control_node mic_gain 0.5     # (float; -1 = leave default)
ros2 param set /alfie/respeaker_control_node pp_min_ns 0.5    # noise-suppression floor (float; -1 = default)
```

`debug_scores` (default true, tuning aid): logs wake near-miss scores ≥ 0.10 and barge
mic-RMS readings ≥ 250, throttled. Set false to quiet the logs in production.

---

## Known limitations / TODO

- **`stop` model is too short** — it only scores ~0.10–0.18 on the target word (vs 0.45–0.72
  for `hey alfie`/`alfie`) and ~0 during TTS. Retrain as **"alfie stop"** (a 2-word phrase
  is far more separable); save as `stop.onnx` for a drop-in swap.
- **`alfie` is noisier** than `hey alfie` (short word) — kept at a slightly looser threshold.
- **Barge-in is energy-based**, so it's environment-sensitive (see above). Retrain with
  TTS-mixed positives for robust wake-word barge-in.
- Wake-word barge-in currently relies on the energy gate; the `stop`/`alfie` models firing
  *during* TTS is not reliable and shouldn't be depended on until retrained.

## Dependencies

`openwakeword` (pip), `onnxruntime`, `numpy`. Installed via pip, not rosdep.
Run oww with `inference_framework='onnx'` (no `tflite_runtime` needed).
