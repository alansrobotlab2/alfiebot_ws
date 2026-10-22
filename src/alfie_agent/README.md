# alfie_agent

The conversation **brain** for Alfie. It bridges ASR and TTS: it takes a
transcript (`asrresult`), runs an agentic turn against the local MLC-LLM server
(Qwen3.6 35B-A3B) — which can call **tools** such as reading, writing, and
searching notes in a shared Obsidian vault — and publishes the spoken reply as
`speechrequest`s, clause by clause while the model is still writing. Ported
(minimal slice) from the [lloyd](https://github.com/alansrobotlab2/lloyd) agent;
the voice behaviour follows lloyd's `architecture/voice.md` (see
[Voice behaviour](#voice-behaviour)).

## Architecture

```
asrresult ─▶ agent_node ─▶ prefetch.py ─(<context> on the user msg)─▶ harness.run_turn ──▶ MLC-LLM (localhost:8000)
                │  ▲          └─ facts / qmd lex / earlier episodes (150 ms budget)     │
                │  │(MEMORY.md + recent memory)                                     └─▶ tools/ ── vault_read / vault_write
                │  │                                                                          ├── vault_search ──▶ QMD fork (localhost:8181)
                │  │                                                                          ├── memory_add / _replace / _remove
                │  │                                                                          └── fact_get
                │  └─ memory.py ◀── idle: summarize ▶ memory/episodes/*.md + recent.jsonl + memory/facts/
                └─▶ speechrequest (TTS),  generating (thinking LED)
```

- **`agent_node.py`** — ROS node: topic I/O, wake gating, conversation rules,
  pause-then-decide barge-in, turn-id cancellation, fillers, latency log.
  Delegates thinking to the harness.
- **`speakable.py`** — `ClauseStream`: streamed text → speakable clauses (lloyd's).
- **`conversation.py`** — backchannel / closer / stopper vocabularies, transcript
  wake, mention and echo detection, tool fillers.
- **`timeline.py`** — the per-turn `[latency]` line.
- **`harness.py`** — the tool-calling loop. MLC-LLM can't do native OpenAI
  tool-calling, so tools are described in the system prompt and Qwen emits
  `<tool_call>{...}</tool_call>` blocks that the harness parses and dispatches.
- **`llm_client.py`** — streaming client for the MLC-LLM OpenAI-compatible server.
- **`prompt_builder.py`** — assembles the system prompt: anti-compliance directive
  + SOUL personality + tool protocol. Loads `SOUL.md` from the vault
  (`<vault_root>/alfie/SOUL.md`) if present, else the bundled
  `prompts/SOUL.default.md`.
- **`memory.py`** — the memory subsystem (context management; see below).
- **`prefetch.py`** — per-turn automatic context injection (lloyd's
  "subliminal", slimmed down; see below).
- **`tools/`** — in-process tool modules behind lloyd's `list_tools()` /
  `call_tool()` interface (so they can be lifted behind an MCP server later):
  - `vault.py` — `vault_read`, `vault_write` (path-escape-guarded, audited).
  - `qmd_search.py` — `vault_search` via the QMD daemon (below).
  - `memory_tools.py` — `memory_add`, `memory_replace`, `memory_remove` over the
    curated `alfie/MEMORY.md` (byte-capped, injection-screened).
  - `facts.py` — lloyd-format fact files and the read-only `fact_get`.
  - `clock.py`, `calc.py`, `room.py`, `see.py`, `estop.py` — `current_datetime`,
    `calculate`, `identify_room`/`learn_room`, `look`, `clear_estop`.

### ROS interface

| Direction | Topic | Type |
|---|---|---|
| in | `asrresult` | `alfie_msgs/ASRResult` |
| in | `wake` | `std_msgs/String` (RELIABLE) |
| in | `barge_in` | `std_msgs/Empty` — hard stop (stop word / wake over speech) |
| in | `barge_in/soft` | `std_msgs/Empty` — energy-gate candidate: pause, then decide |
| in | `vad/speech` | `std_msgs/Bool` — ASR node's VAD edges (sustained-speech barge-in) |
| in | `speaking` | `alfie_msgs/Speaking` |
| in | `tts/spoken` | `std_msgs/String` — clauses TTS finished playing (what was heard) |
| in | `llm/ready` | `std_msgs/Bool` (latched) |
| out | `speechrequest` | `alfie_msgs/SpeechRequest` (RELIABLE) — one clause, `turn_id`, `more_follows` |
| out | `tts/pause` | `std_msgs/Bool` — true pauses, false resumes |
| out | `tts/stop` | `std_msgs/Empty` — stop and drop queued clauses |
| out | `generating` | `std_msgs/Bool` |

### ROS parameters (declared with defaults)

| Param | Default | Notes |
|---|---|---|
| `llm_base_url` | `http://localhost:8000/v1` | MLC-LLM OpenAI API |
| `model_id` | `dist/qwen3_6-35B-A3B-q4f16_1` | fallback if `/v1/models` fails |
| `vault_root` | `~/obsidian` | shared-with-lloyd Obsidian vault |
| `qmd_url` | `http://localhost:8181/query` | QMD search daemon |
| `qmd_skip_rerank` | `true` | skip the cross-encoder reranker (unneeded in agent use; much faster on CPU) |
| `max_tool_iters` | `4` | max tool round-trips per turn |
| `max_history_turns` | `6` | **deprecated** — declared for launch compat; superseded by `active_token_budget` |
| `active_token_budget` | `2000` | approx-token cap on the live episode's raw turns |
| `recent_window_min` | `30` | how far back the injected recent-memory window reaches (minutes) |
| `recent_max_items` | `5` | max recent-interaction summaries injected at episode start |
| `compaction_enabled` | `true` | write episode summaries into memory on idle |
| `memory_max_bytes` | `1500` | byte cap on `alfie/MEMORY.md` (injected every episode); adds past it are refused |
| `prefetch_enabled` | `true` | per-turn `<context>` prefetch (`prefetch.py`) |
| `prefetch_budget_ms` | `150` | hard wall for the prefetch legs; late legs are abandoned (a late vault hit rides the next turn) |
| `prefetch_max_chars` | `1200` | cap on the injected `<context>` block (prefill latency) |
| `primary_user` | `''` | entity whose facts a first-person question ("what's *my*...") loads; Alfie can't tell who is speaking. e.g. `Alan` |
| `max_tokens` / `temperature` | `200` / `0.7` | generation |
| `followup_window_s` | `6.0` | listening window after a wake / after each reply |
| `question_window_s` | `20.0` | window after a reply that ended with a question |
| `text_wake` | `true` | a transcript that opens with a wake phrase wakes Alfie |
| `barge_mode` | `pause` | `pause` (pause, then decide on the transcript) or `off` |
| `barge_min_speech_s` | `0.5` | sustained speech needed to pause |
| `barge_warmup_s` | `2.0` | no speech-triggered pause in the first seconds of a reply |
| `false_interruption_s` | `2.0` | resume if no words this long after the speaker goes quiet |
| `max_pause_s` | `10.0` | resume a pause no transcript ever resolved |
| `echo_overlap` | `0.6` | word overlap (plus a 3-word run) that marks a transcript as own echo |
| `filler_after_s` | `2.5` | say "One moment." if nothing has been said by then |
| `progress_every_s` | `8.0` | progress line during a long tool phase |

## Voice behaviour

- **Streaming reply.** The harness hands out the spoken part of each generation
  as it streams (`SpokenFilter` drops `<think>`/`<tool_call>`); `ClauseStream`
  cuts it into clauses — the first may be cut at a comma once 20 chars long — and
  each is published immediately with `more_follows=true`. An empty request with
  `more_follows=false` ends the reply. Text written before a tool call ("Let me
  look.") is spoken before the tool runs.
- **Fillers.** Nothing said 2.5 s in → "One moment."; a tool about to run with
  nothing said → a contextual line ("Let me take a look.", "Let me check my
  notes."); long tool phases → "Still working on it." every 8 s.
- **Barge-in: pause, then decide.** See `alfie_wakeword/README.md#barge-in`. A new
  request — including one made during a silent tool phase — supersedes the running
  turn, and the next turn is told what the listener actually heard and not to
  resume or repeat the interrupted reply.
- **Latency.** Each turn logs one line, e.g.
  `[latency] eos->audio 1.30s | speech_end -> asr_done +451ms -> turn_start +1ms -> first_delta +479ms -> first_clause +303ms -> first_audio +56ms -> done +550ms | asr 120ms/1.5s (complete, p=0.50)`.
  `eos->audio` is end of speech to the first sound Alfie makes.

## Context management & memory

Alfie has no "new chat" button, so context is managed automatically as
**stateless episodes over an actively-managed memory subsystem** (`memory.py`):

- **Episode = one wake-initiated conversation.** Raw turns accumulate only within
  the live episode, bounded by `active_token_budget` (approx-token trim, oldest
  pairs dropped first — `harness.budget_trim`). There is no cross-episode raw
  history.
- **Idle compaction (hard reset + memory).** When the follow-up window lapses with
  un-compacted turns, an idle timer hands the episode to a background thread that
  summarizes it (one LLM call, run while the GPU is otherwise free) and then
  clears active context. The summary and the extracted facts
  (`{entity, category, fact}`) are written to:
  - `<vault>/memory/episodes/<ts>.md` — a permanent, **QMD-indexed** note;
  - `<vault>/memory/recent.jsonl` — the rolling working-memory log;
  - `<vault>/memory/facts/<Entity>/<Entity>-<category>.md` — one file per entity
    and category in **lloyd's fact-file format** (same frontmatter, category
    vocabulary, `<prefix>-NNN` ids and bitemporal `expired_at`/`invalid_at`
    fields; `provenance: ALFIE_VOICE`). Exact duplicates are refused. Entity
    resolution, contradiction handling and graph edges are lloyd's nightly job
    and are not done on the Orin.
- **Curated memory — `alfie/MEMORY.md`.** A short list the model edits itself with
  `memory_add` / `memory_replace` / `memory_remove` (lloyd's memory tools). Capped
  at `memory_max_bytes`; writes matching lloyd's prompt-injection shapes ("ignore
  previous instructions", "you are now a", ...) are refused — anyone in the room
  can talk to Alfie.
- **Episode-start injection.** A fresh wake folds `MEMORY.md` (`# Memory`) and a
  time+size-windowed view of `recent.jsonl` (`# Recent context`,
  `recent_window_min` / `recent_max_items`) into the system prompt as trailing
  sections, after the pinned base prompt (MLC only accepts a system message at
  position 0), so the pinned prefix is never invalidated.
- **Per-turn prefetch (`prefetch.py`).** Before each turn, three CPU-only legs run
  in parallel under `prefetch_budget_ms`, and what finishes is rendered into a
  `<context>` block (≤ `prefetch_max_chars`) prepended to the *model's copy* of
  the user message (history keeps what was said):
  - **facts** — entities named in the message (`Alfie` for "you/your";
    `primary_user` + `User` for "I/my") → up to 2 entities × 3 facts;
  - **vault** — one OR-mode BM25 qmd query over the message's content words plus
    the episode's decaying focus keywords; a hit is kept only if it shares ≥ 2 of
    the message's own (stemmed) keywords — qmd's fused score is rank-relative and
    the vector leg matches *something* on a small vault, so it cannot gate;
  - **earlier** — only on explicit past-conversation cues ("yesterday", "what did
    we", "last time"): episode summaries older than the recent window.

  Messages under 10 characters skip it. Measured on the Orin: 10–20 ms per turn.
  No LLM call — lloyd's every-5-turns topic extraction would compete for the one
  MLC slot.
- **Deeper recall stays on-demand** via `vault_search` and `fact_get`.

**Eval** (`eval/memory_eval.py`, 15 recall questions over a seeded throwaway
vault, real MLC server, 2026-10-22):

| arm | accuracy | mean turn | tool calls / q |
|---|---|---|---|
| tools only | 6/15 | 0.86 s | 0.60 |
| + MEMORY.md & recent window | 11/15 | 1.14 s | 0.67 |
| + per-turn prefetch | **15/15** | **0.70 s** | 0.20 |

The prefetch arm is the fastest as well as the most accurate: an injected fact
replaces a `fact_get` / `vault_search` round trip (~0.5–1 s each).

## Prefix caching — fast first turns (`_pin_base_prefix`)

Every request carries the same ~800-token system prompt. Without help, the
**first turn of each conversation re-prefills all of it** — measured **~2.3 s**
TTFT cold on the Orin. The fix (validated on-device 2026-07-13) pins that prompt
once so first turns **fork** from it and prefill only the user delta: **~0.31 s**
(~8× faster). Resolved the engine-side problem posed in
[`docs/mlc_prefix_cache_problem_statement.md`](../../docs/mlc_prefix_cache_problem_statement.md).

**Why it's not automatic (the hybrid wall).** Qwen3.6-35B-A3B is a *hybrid*
model — attention KV **plus** a GatedDeltaNet **recurrent** state. Pure-attention
engines (vLLM/SGLang) reuse any prefix for free because KV pages are
position-independent. The GDN recurrent state only exists at a sequence's *latest*
position, so reusing an interior prefix normally means rolling that state back —
bounded and memory-expensive. The escape hatch: the system prompt is byte-identical
every request, so its terminal GDN state is a **constant** — capture it once and
every request forks with ~zero rollback.

**How the agent does it.** On `llm/ready`, `_pin_base_prefix` sends one
`max_tokens=1` completion of the **base** system prompt with
`debug_config.pinned_system_prompt=True`. MLC keeps that sequence resident (never
recycled, never evicted); real turns fork from it at the system-prompt boundary.
No per-turn re-warm, survives episode/`summarize()` prompt changes.

**Configuration intuition** — three load-bearing settings on the serve node
([`alfie_llm/mlc_llm_serve_node.py`](../alfie_llm/alfie_llm/mlc_llm_serve_node.py)),
each of which silently breaks the feature if wrong:

| Setting | Value | Why |
|---|---|---|
| `--enable-debug` | **on** | Without it the server *strips* `debug_config` from requests (a `server_context` gate), so the pin is silently dropped and every first turn re-prefills cold. This is why the pin "never worked" before and the earlier warm-parent hack existed. |
| `max_num_sequence` | **2** | rnn_state slots = `max_num_sequence + recycling_seqs`; the pinned parent counts in **neither**, so it permanently steals one. Reserve a slot (1 live stream + 1 pin) or the engine's background loop dies with `rnn_state GetFreeSlot: sequence slot is full` and requests hang. |
| `prefix_cache_max_num_recycling_seqs` | **0** | Do **not** hold finished turns. A held prior turn *preempts* the clean pin-fork and sends distinct first-turns down a slow reuse/rollback path — measured **~1.87 s vs ~0.31 s**. With 0, every turn forks the pin. (Follow-ups then fork the pin too and prefill the small growing history delta: ~0.3→0.5 s across several turns — fine for short voice chats.) Never `-1`: that sizes 0 KV slots and the engine reload deadlocks. |

`max_history_size=16` sizes the rnn_state history ring that bounds the fork's
rollback (the chat-template tail); 8 also works. **Pin the *base* prompt, not the
per-episode one** — there is no unpin API, so pinning a changing prompt each
episode would leak rnn_state slots until the engine runs out. Episodes that fold
in a recent-memory preamble still fork the base and prefill only the (small)
preamble.

**Measured (Orin, 35B-A3B q4f16_1, exact production serve command):**

| Case | TTFT |
|---|---|
| Cold first turn (no pin) | ~2.3 s |
| Pinned fork — first turn | **~0.31 s** |
| Pinned fork — flat across 15 distinct turns | ~0.31 s (no drift, no crash) |
| Growing conversation (fork pin + history delta) | 0.30 → 0.48 s over 4 turns |
| `recycling=1` regression (why it's 0) | ~1.87 s |

## Requirements

### Python / ROS (the node itself)
- ROS 2 Jazzy, `rclpy`, `std_msgs`, `alfie_msgs`
- `requests` (LLM stream and QMD client) and `yaml` (fact-file frontmatter)
- The **MLC-LLM server** must be up (`alfie_llm/mlc_llm_serve_node`); the node
  gates on its latched `llm/ready`.

### QMD — vault search daemon (external service)

`vault_search` and the prefetch's vault leg talk to **QMD**, a local hybrid
**BM25 (SQLite FTS5) + vector (embeddinggemma-300M)** search engine over the
vault. We run **lloyd's fork** —
[alansrobotlab2/qmd](https://github.com/alansrobotlab2/qmd), branch `lloyd`
(upstream tobi/qmd 2.8.3 plus lloyd's patches) — built from source in `~/qmd`
under Node 24 with node-llama-cpp's prebuilt linux-arm64 **CPU** binary. It is a
separate process; if it's down, `vault_search` returns a clean error, the
prefetch's vault leg is skipped, and the rest of the agent still works.

What the fork buys on the Orin (CPU-only, because MLC owns the GPU):
- **In-memory vector index** (`src/vecindex.ts`) instead of a sqlite-vec scan per
  query: warm hybrid query **~35 ms**, lex-only **~5 ms** (upstream 2.5.3 here
  measured ~121 ms).
- **`lexMode: "or"`** — OR the BM25 terms; upstream ANDs them, which drops most
  loose spoken queries. `qmd_search.py` always sends it.
- **`rerank: false` honoured**, and a `meta` block with per-phase timings
  (`fts`/`embed`/`vec`) on every response.
- The rerank cache survives a re-index; global fusion / `collectionFloor` for
  when the vault is split into several collections.

Contract: `POST http://127.0.0.1:8181/query`
```json
{"searches":[{"type":"lex","query":"..."},{"type":"vec","query":"..."}],
 "limit":5,"collections":["obsidian"],"rerank":false,"lexMode":"or"}
```
→ `{"results":[{"file","title","snippet","score"}, ...], "meta":{...}}`
(`file` is `qmd://obsidian/<path>`; `qmd_search.py` strips the scheme and
collection so the path lines up with `vault_read`, and strips the snippet's hunk
header and line numbers.)

- **Keep the reranker off.** The 0.6B cross-encoder takes ~18 s per query on CPU.
  `qmd_skip_rerank` (default `true`) controls it.
- **Separate indexes need `INDEX_PATH`.** `QMD_CONFIG_DIR` moves only the config:
  a `qmd update` with another config still rewrites `~/.cache/qmd/index.sqlite`.
  `eval/memory_eval.py` sets both.

#### Services (boot-persistent)
Unit + env files live in [`qmd/`](qmd/):
- `alfie-qmd-daemon.service` → `node ~/qmd/bin/qmd mcp --http --port 8181` (CPU, loopback)
- `alfie-qmd-watcher.service` → `qmd/qmd-watcher.sh` (`qmd update && qmd embed` on
  vault changes: inotify when `inotify-tools` is installed, else a 10 s poll)
- `qmd/qmd.env` — shared `EnvironmentFile`; `qmd/index.yml` — tracked copy of
  `~/.config/qmd/index.yml`

#### Install from scratch
```bash
# Node 24 (arm64) into ~/.local/node
V=v24.22.1; curl -sO https://nodejs.org/dist/$V/node-$V-linux-arm64.tar.xz
tar xf node-$V-linux-arm64.tar.xz -C ~/.local && mv ~/.local/node-$V-linux-arm64 ~/.local/node
export PATH=$HOME/.local/node/bin:$PATH
# lloyd's qmd fork
git clone -b lloyd https://github.com/alansrobotlab2/qmd ~/qmd && cd ~/qmd
npm install                                   # builds dist/
npm install-scripts approve node-llama-cpp esbuild && npm rebuild node-llama-cpp esbuild
# index the vault (CPU; first embed downloads embeddinggemma to ~/.cache/qmd/models)
mkdir -p ~/.config/qmd && cp ~/alfiebot_ws/src/alfie_agent/qmd/index.yml ~/.config/qmd/
QMD_FORCE_CPU=1 node ~/qmd/bin/qmd update && QMD_FORCE_CPU=1 node ~/qmd/bin/qmd embed
# services (needs sudo)
sudo apt-get install -y inotify-tools        # optional; the watcher polls without it
sudo cp ~/alfiebot_ws/src/alfie_agent/qmd/alfie-qmd-*.service /etc/systemd/system/
sudo systemctl daemon-reload && sudo systemctl enable --now alfie-qmd-daemon alfie-qmd-watcher
```

### obsidian-cli (installed, unconfigured)

The Python [`obsidian-cli`](https://github.com/Bip901/obsidian-cli) (1.0.2) is
installed via `uv` (auto-provisions Python 3.11+, isolated from the ROS env):
```bash
uv tool install obsidian-cli      # executable: ~/.local/bin/obsidian
```
It is intentionally **left unconfigured** (no vault registered) — set up in
anticipation of future vault tooling.

## Build & test
```bash
colcon build --packages-select alfie_agent
colcon test --packages-select alfie_agent          # parser, vault, qmd_search, memory, facts, prefetch units
# memory A/B against the live MLC server (sequential requests; not mid-conversation)
PYTHONPATH=src/alfie_agent python3 src/alfie_agent/eval/memory_eval.py
```
The unit tests are pure Python (no ROS/LLM/QMD needed); the harness/LLM/QMD are
verified live against the running servers.

## Deferred (not in this iteration)
Still deferred: recent-buffer consolidation and daily digests; lloyd's heavier
fact pipeline (entity resolution, contradiction handling, typed graph edges) and
read access to lloyd's `kg.sqlite` — that needs the vault shared between the two
machines (it is not today: `~/obsidian` here is Alfie's own), and a WAL-mode
SQLite file should arrive as a backup-API snapshot, not a file-sync copy; plus
skills, autonomy/workers, discord/email, and inner voice. The tool-module
interface is kept MCP-shaped so these can be added incrementally.
