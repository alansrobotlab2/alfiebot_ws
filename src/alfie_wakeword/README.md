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
  Armed only while Alfie is speaking or thinking (`speaking` / `generating`), so a
  "stop" in idle room chatter does nothing.
- **Energy-gate barge-in** → speak up over Alfie and he *pauses*; the agent decides
  from the transcript whether to stop or carry on (see [Barge-in](#barge-in)).
- Every detection is echoed for LED feedback: **green flash** for a wake word,
  **red flash** for a cancel / barge-in.

## Pipeline / topics

```
audio_publisher ──audio_frames──▶ wakeword_node ──wake (String phrase)──▶ agent_node (opens window)
                                       │        ──wakeword/detection (String key)──▶ led_behavior (flash)
                                       ├────────── barge_in (Empty) ──▶ alfie_tts (stop) + agent_node (cancel)
                                       └────────── barge_in/soft (Empty) ──▶ agent_node (pause, then decide)
```

- `audio_frames` — `alfie_msgs/AudioFrame`, int16[512] @ 16 kHz (~32 ms), BEST_EFFORT.
  Accumulated into openWakeWord's 1280-sample (80 ms) hops.
- `wake` — `std_msgs/String` (spoken phrase), RELIABLE. Opens the agent window.
- `wakeword/detection` — `std_msgs/String` (raw model key), RELIABLE. LED feedback only.
- `barge_in` — `std_msgs/Empty`, BEST_EFFORT. Hard stop: the `stop` word, or a wake
  word said over speech. Stops TTS + cancels the in-flight turn.
- `barge_in/soft` — `std_msgs/Empty`, BEST_EFFORT. Energy-gate candidate; the agent
  pauses TTS and lets the transcript decide (`barge_mode: hard` publishes `barge_in`
  instead, the old behaviour).
- `speaking` — `alfie_msgs/Speaking` in; tells the node when TTS is playing.
- `generating` — `std_msgs/Bool` in; the agent is thinking (arms the `stop` word).

## Models

Bundled in [`models/`](models/) and loaded **by path** (never downloaded at runtime):

| File | Role | Default threshold |
|---|---|---|
| `hey_alfie.onnx` | wake word | 0.10 |
| `alfie.onnx` | wake word (short → noisier) | 0.20 |
| `stop.onnx` | cancel word | 0.10 |
| `melspectrogram.onnx`, `embedding_model.onnx` | oww base models | — |

Custom models are the openWakeWord classifier format: input `x [1,16,96]` → `sigmoid [1,1]`.
Single-file `.onnx` preferred; if a model ships as `.onnx` + external `.onnx.data`, both
are bundled (the `setup.py` `glob('models/*')` handles it).

**Swapping in a retrained model:** drop the new `.onnx` in `models/`, rebuild
`alfie_wakeword`, restart the node. Keep the same filename for a zero-code swap.

## Offline operation

The whole voice stack boots with no network:
- **Wake models** — bundled here, loaded by path.
- **ASR (Parakeet)** — loaded from `~/.alfie_asr/models`; HuggingFace is touched only for
  the one-time download when that directory is empty.
- **VAD (Silero)** — the `.onnx` ships inside the `silero-vad` pip wheel and is copied to
  `~/.alfie_asr/models` on first run (run through onnxruntime directly; no torch).
- **End of turn (turn-1-mini)** — `~/.alfie_asr/models/turn-1-mini/`, downloaded once if
  missing; without it the ASR node falls back to VAD silence alone.
- **TTS (Piper)** — bundled voice in `alfie_tts`.

## Wake-gated conversation (in `alfie_agent/agent_node.py`)

- Transcripts are dropped unless a wake window is open — **or the transcript opens with
  a wake phrase** ("Hey Alfie, …", one leading "um" allowed), which opens one. This
  second path catches wake words openWakeWord misses (in lloyd it caught 40/48 against
  17–23 acoustically).
- The window stays open while a turn runs or Alfie speaks, then reopens for
  `followup_window_s` (6 s) after each reply — `question_window_s` (20 s) when the reply
  ended with a question.
- The leading wake phrase is stripped from the command; a bare wake ("Hey Alfie") or a
  very short mis-heard wake right after a detection is acknowledged, not answered.
- An acoustic wake on a *mention* ("did Alfie finish?") is dropped: the transcript names
  him mid-sentence rather than addressing him.
- Backchannels ("yeah", "okay", "great") are not turns unless Alfie just asked something;
  closers ("that's all", "thanks Alfie") end the conversation.

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

### Pause, then decide (current solution)
Two candidate signals, one decision (ported from lloyd's full-duplex barge-in):

- **Energy gate** (here): while `speaking`, mic RMS above `barge_energy_threshold` for
  `barge_hold_hops` hops (80 ms each) → publish `barge_in/soft`.
- **Sustained speech** (agent): `vad/speech` from the ASR node held for
  `barge_min_speech_s` (0.5 s), after `barge_warmup_s` (2 s) of the reply.

Either one **pauses** TTS (`tts/pause`). The transcript that follows decides: real words →
stop and answer them (the next turn is told exactly what the listener heard); empty, a
backchannel ("yeah", "mm"), or Alfie's own echo → **resume** from the same sample; no words
within `false_interruption_s` (2 s) of the speaker going quiet → resume. A cough or the TV
costs a short pause instead of the whole reply.

The original energy-gate rationale still holds: because the AEC keeps the mic quiet unless
the user speaks, a sustained loud mic during TTS is usually the user — no wake-word match
needed.

Measured levels (this workshop, with a background TV):

| Signal (during TTS) | mic RMS |
|---|---|
| AEC residual (Alfie only) | ~250 |
| Background TV | up to ~2400 |
| User interrupting (normal → speak-up) | ~800–3100, peaks 4000–11000 |

**Tuning is environment-dependent** because the user's voice and background overlap:
- **Quiet room:** `barge_energy_threshold ≈ 1200` → natural normal-voice barge-in.
- **This room (TV):** `≈ 2800`, `barge_hold_hops = 2` → a deliberate speak-up clears the
  background. (Baked as the default.) With the soft gate a false trigger only pauses, so a
  lower threshold is now worth trying.
- **Background talk (TV) can also trip the sustained-speech trigger.** Its transcript then
  decides like any other; if the TV keeps interrupting replies, set the agent's
  `barge_mode` to `off` (hard `stop`/wake barge-in still works).

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
ros2 param set /alfie/wakeword_node barge_mode soft      # soft | hard
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
- **The energy gate is environment-sensitive** (see above); the pause-then-decide
  agent logic limits what a false trigger costs. Retrain with TTS-mixed positives for
  robust wake-word barge-in.
- Wake-word barge-in currently relies on the energy gate; the `stop`/`alfie` models firing
  *during* TTS is not reliable and shouldn't be depended on until retrained.

## Dependencies

`openwakeword` (pip), `onnxruntime`, `numpy`. Installed via pip, not rosdep.
Run oww with `inference_framework='onnx'` (no `tflite_runtime` needed).
