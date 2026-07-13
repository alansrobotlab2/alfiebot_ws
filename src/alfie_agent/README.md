# alfie_agent

The conversation **brain** for Alfie. It bridges ASR and TTS: it takes a
transcript (`asrresult`), runs an agentic turn against the local MLC-LLM server
(Qwen3.6 35B-A3B) — which can call **tools** such as reading, writing, and
searching notes in a shared Obsidian vault — and publishes the spoken reply as a
`speechrequest`. Ported (minimal slice) from the [lloyd](https://github.com/alansrobotlab2/lloyd)
agent.

## Architecture

```
asrresult ─▶ agent_node ─▶ harness.run_turn ──▶ MLC-LLM (localhost:8000)  ← streams <tool_call> blocks
                │                   │
                │                   └─▶ tools/  ── vault_read / vault_write (file I/O)
                │                              └── vault_search ──▶ QMD daemon (localhost:8181/query)
                └─▶ speechrequest (TTS),  generating (thinking LED)
```

- **`agent_node.py`** — ROS node: topic I/O, wake-window gating, turn-id / barge-in
  cancellation. Delegates thinking to the harness.
- **`harness.py`** — the tool-calling loop. MLC-LLM can't do native OpenAI
  tool-calling, so tools are described in the system prompt and Qwen emits
  `<tool_call>{...}</tool_call>` blocks that the harness parses and dispatches.
- **`llm_client.py`** — streaming client for the MLC-LLM OpenAI-compatible server.
- **`prompt_builder.py`** — assembles the system prompt: anti-compliance directive
  + SOUL personality + tool protocol. Loads `SOUL.md` from the vault
  (`<vault_root>/alfie/SOUL.md`) if present, else the bundled
  `prompts/SOUL.default.md`.
- **`tools/`** — in-process tool modules behind lloyd's `list_tools()` /
  `call_tool()` interface (so they can be lifted behind an MCP server later):
  - `vault.py` — `vault_read`, `vault_write` (path-escape-guarded, audited).
  - `qmd_search.py` — `vault_search` via the QMD daemon (below).

### ROS interface

| Direction | Topic | Type |
|---|---|---|
| in | `asrresult` | `alfie_msgs/ASRResult` |
| in | `wake` | `std_msgs/String` (RELIABLE) |
| in | `barge_in` | `std_msgs/Empty` |
| in | `speaking` | `alfie_msgs/Speaking` |
| in | `llm/ready` | `std_msgs/Bool` (latched) |
| out | `speechrequest` | `alfie_msgs/SpeechRequest` |
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
| `max_history_turns` | `6` | rolling conversation history |
| `max_tokens` / `temperature` | `200` / `0.7` | generation |

## Requirements

### Python / ROS (the node itself)
- ROS 2 Humble, `rclpy`, `std_msgs`, `alfie_msgs`
- `requests` (only extra Python dep; used for the LLM stream and QMD client)
- The **MLC-LLM server** must be up (`alfie_llm/mlc_llm_serve_node`); the node
  gates on its latched `llm/ready`.

### QMD — vault search daemon (external service)

`vault_search` talks to **QMD** ([tobi/qmd](https://github.com/tobi/qmd), npm
`@tobilu/qmd`) — a local hybrid **BM25 (SQLite FTS5) + vector (embeddinggemma)**
search engine over the vault, with an optional Qwen3 reranker. It runs under
**Bun + Node 24** with **node-llama-cpp** (llama.cpp GGUF models). It is a
separate process; if it's down, `vault_search` returns a clean error and the rest
of the agent still works.

Contract the agent speaks: `POST http://localhost:8181/query`
```json
{"searches":[{"type":"lex","query":"..."},{"type":"vec","query":"..."}],
 "limit":5,"collections":["obsidian"]}
```
→ `{"results":[{"file","title","snippet","score"}, ...]}`
(qmd namespaces `file` as `obsidian/<path>`; `qmd_search.py` strips the collection
prefix so the path lines up with `vault_read`.)

#### Installed toolchain (on the Orin AGX, `alfiebrain`)
| Component | Version / location | Notes |
|---|---|---|
| Bun | 1.3.14 — `~/.bun/bin/bun` | JS runtime / installer |
| Node.js | 24.18.0 — `~/.local/node/bin` | **must be Node 24** — `better-sqlite3`'s prebuilt is ABI 137 (Node 24); Node 20 fails to load it |
| qmd | `@tobilu/qmd` 2.5.3 — `~/.bun/bin/qmd` | `bun install -g @tobilu/qmd` |
| node-llama-cpp | 3.18.1 (a CUDA build of llama.cpp b8390 was compiled for sm_87 but is unused — see below) | runs CPU-only |
| models | `~/.cache/qmd/models/` (~2.3 GB) | embeddinggemma-300M-Q8_0, qwen3-reranker-0.6b-q8_0, qmd-query-expansion-1.7B — **all pre-cached (offline)** |
| build tools | cmake 3.22, gcc/g++, CUDA 12.6 | present; used for the (now-unused) CUDA compile |
| inotify-tools | 3.22 (apt) | for the reindex watcher |

#### CPU-only (why not GPU)
qmd runs **entirely on CPU** (`QMD_LLAMA_GPU=false`, `QMD_FORCE_CPU=1`) for both
the daemon and the watcher. This is deliberate, not a shortcut:

- **GPU is blocked by the resident 35B MLC model.** ggml-cuda's VMM pooled
  allocator reserves a fixed **32 GB of GPU virtual address space**
  (`CUDA_POOL_VMM_MAX_SIZE = 1<<35`) on first allocation. With MLC occupying the
  GPU, `cuMemAddressReserve` for that 32 GB VA range fails — so *any* qmd process
  (daemon or embed) crashes at model load with "CUDA error: out of memory". This
  is a **virtual-address reservation** failure, not a RAM shortage (~12 GB is
  free; CUDA confirms it). It happens even for a single qmd GPU process with
  nothing else but MLC on the GPU.
- **CPU is also the better choice here.** The models are small (300M / 0.6B /
  1.7B) and fast on the 12-core CPU, and running them on CPU keeps them from
  contending with the 35B for GPU compute during a conversation.
- **To force GPU anyway** (not recommended): shrink `CUDA_POOL_VMM_MAX_SIZE` in
  `.../node-llama-cpp/llama/llama.cpp/ggml/src/ggml-cuda/ggml-cuda.cu` (e.g.
  `1<<32` = 4 GB) and rebuild node-llama-cpp (`source build`). Untested; qmd would
  then contend with the 35B on the GPU.

#### Offline
All three models are cached under `~/.cache/qmd/models`; the daemon downloads
nothing on start/restart (verified). Keep that cache to stay offline.

#### Services (boot-persistent)
Unit + env files live in [`qmd/`](qmd/) and are installed to
`/etc/systemd/system/` (`User=alfie`, `enabled`):
- `alfie-qmd-daemon.service` → `qmd mcp --http --port 8181` (CPU)
- `alfie-qmd-watcher.service` → `qmd/qmd-watcher.sh` (inotify → `qmd update && qmd embed`, CPU)
- `qmd/qmd.env` — shared `EnvironmentFile` (PATH, CPU/offline flags)

Ops:
```bash
sudo systemctl status alfie-qmd-daemon alfie-qmd-watcher
sudo systemctl restart alfie-qmd-daemon
qmd status && qmd collection list           # index health
QMD_FORCE_CPU=1 qmd update && QMD_FORCE_CPU=1 qmd embed   # manual reindex (CPU)
```

#### Reproduce QMD from scratch
```bash
curl -fsSL https://bun.sh/install | bash                       # Bun
#  install Node 24 (arm64) to ~/.local/node   (ABI must match better-sqlite3)
export PATH="$HOME/.local/node/bin:$HOME/.bun/bin:$PATH"
bun install -g @tobilu/qmd                                      # qmd (Blocks postinstalls)
node ~/.bun/install/global/node_modules/node-llama-cpp/dist/cli/cli.js postinstall
# node-llama-cpp builds a CPU llama.cpp from source on first model use.
qmd collection add obsidian ~/obsidian --mask '**/*.md'
QMD_FORCE_CPU=1 qmd update && QMD_FORCE_CPU=1 qmd embed \
    && QMD_FORCE_CPU=1 qmd query "warm the models"              # downloads + caches all 3 models
sudo apt-get install -y inotify-tools
sudo cp qmd/alfie-qmd-*.service /etc/systemd/system/ && sudo systemctl daemon-reload
sudo systemctl enable --now alfie-qmd-daemon alfie-qmd-watcher
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
colcon test --packages-select alfie_agent          # parser, vault, qmd_search, harness units
```
The unit tests are pure Python (no ROS/LLM/QMD needed); the harness/LLM/QMD are
verified live against the running servers.

## Deferred (not in this iteration)
Facts/knowledge-graph memory, `vault_recall` (graph-expanded search), skills,
autonomy/workers, session persistence + compaction, discord/email, inner voice.
The tool-module interface is kept MCP-shaped so these can be added incrementally.
