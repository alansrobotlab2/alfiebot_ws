# Problem statement — persistent system-prompt prefix reuse for the hybrid (GDN) Qwen3.6 build

Audience: the engineer/Claude working on the custom `~/mlc-llm` build.
Context: this build serves **Qwen3.6 35B-A3B q4f16_1** in `--mode interactive` for a
voice agent (`alfie_agent` → HTTP `/v1/chat/completions`). Every request carries the
**same ~800-token system prompt** (identity + tool specs), followed by a short, varying
user turn.

## Symptom

The **first turn of every conversation** re-prefills the entire system prompt. Measured
on-device (Orin, prefill ≈ 300 tok/s):

| case | TTFT |
|---|---|
| first turn, cold system prompt | **~2.1 s** |
| in-conversation follow-up (strict continuation) | ~0.3 s |

Follow-ups are fast because they *extend* the previous finished sequence. First turns are
slow because nothing in the cache is an extension of `[system + new_user]`.

## Root cause (confirmed in source)

This is a **hybrid model**: attention KV **plus** a GDN linear-attention **recurrent
`rnn_state`**. The prefix cache *can* fork a new request off the shared system-prompt
prefix of a recycled parent sequence, but forking at an **interior** offset requires
rolling the parent's recurrent state back by `pop_n = parent_len − match_offset` tokens —
and that is gated on the rnn_state history ring buffer:

- `cpp/serve/prefix_cache.h` (~L31-39): `PrefixCacheCanRollbackCallback` — reuse/fork is
  allowed **iff** `pop_n <= rnn_state.available_history_num`.
- `cpp/serve/prefix_cache.cc` `InsertSequence`:
  - reuse path (~L104-123): gated on `pop_n <= history` **and** `match_offset > 0.9 *
    parent_len` (the >90% gate alone already excludes "match only the system head").
  - fork path (~L134-142): each candidate skipped when `parent_len − match_offset >
    history`.
- `available_history_num` is sized by **`max_history_size`** (`cpp/serve/model.cc` ~L951
  `rnn_max_history`; `cpp/serve/config.cc` ~L997 log line).

So interior system-prompt reuse only happens when the parent's tokens *after* the system
prompt number `<= max_history_size`. For pure-attention models `can_rollback` is always
true (position-independent KV pages), which is why vLLM/SGLang/OpenAI can reuse "whatever
% of the prefix matches" for free. The GDN recurrent state is the wall here: it only
exists at the parent's **latest** position, so reconstructing it at an interior offset
needs either stored history (bounded, memory-expensive) or recompute (= prefill).

`max_history_size` is **not free**: it sizes a per-position recurrent-state slab. At
`gpu_memory_utilization=0.5`, `max_history_size=256` **OOMs at load**; `32` fits.

## Current workaround (already shipped in alfiebot, no engine change)

1. `max_history_size` 4 → **16** on the serve node (16 and 32 give identical latency;
   16 keeps ~5 GB less RNN state — see memory note below).
2. Keep a **short-tailed "warm" parent** `[system, "hi", <1 tok>]` resident (agent calls a
   1-token completion on the system prompt on startup, after every turn, and on idle).
   A real first turn then **forks** from it: `pop_n ≈ 13` (the warm tail) `<= 16` → reuse.

Result: **first-turn TTFT ~2.1 s → ~0.3-0.6 s**, verified end-to-end.

Note the **RNN-state memory cost**: at load the engine reports RNN state ≈ 0.3 GB *per
history slot* for this 35B model (`max_history_size=16` ≈ 5 GB, `32` ≈ 9.8 GB, `256`
OOMs at `gpu_memory_utilization=0.5`). So we cannot simply raise `max_history_size` to
cover long divergent tails — the cost is linear and large.

### Why this is not good enough

- Fragile: depends on the warm parent staying resident (only 1 recycling slot in
  interactive; a `summarize()` call with a different system prompt evicts it, so we must
  re-warm on a schedule).
- Wastes a GPU completion per conversation just to hold a cache entry.
- `max_history_size=32` costs real VRAM for a ring buffer we don't otherwise need, and
  still fails for any parent whose post-system tail exceeds 32 tokens (e.g. the first turn
  right after history trimming).

## What we'd like from the engine

A **persistent, pinnable prefix** (like vLLM automatic-prefix-caching / a "prompt cache"),
made to work for the hybrid path. The key realization that makes this cheap:

> The system prompt is **byte-identical on every request**, so the GDN recurrent state at
> the **end of the system prompt** is a **constant**. Snapshot it once.

Concretely, any of:

1. **Pin a prefix + snapshot its terminal recurrent state.** Prefill the constant system
   prompt once into a read-only entry that stores (a) its attention KV pages and (b) the
   GDN recurrent state **at the last system-prompt token**. Every request forks from that
   snapshot with `pop_n = 0` (no rollback → `can_rollback` trivially true), prefilling only
   the new user tokens. O(1) memory (one snapshot), independent of `max_history_size`, no
   warm-parent hack. Expose via an API/flag, e.g. a "system prefix" or `cache_prompt`
   marker, or auto-detect the longest common prefix across recent requests and snapshot it.
2. **Store the recurrent state at recycled-sequence page boundaries** so an interior fork
   can restore state at the boundary directly, instead of rolling back token-by-token
   within a bounded ring buffer. Decouples reuse coverage from `max_history_size`.
3. If neither is feasible short-term: at minimum, **document `max_history_size` as the
   prefix-reuse knob for hybrid models** (its name suggests "conversation history", which
   is misleading) and make its memory cost visible / tunable independent of KV.

## Repro / validation harness

- Send `[system(~800 tok), <unique short user>]` twice with **different** user messages.
  - Broken: both ~2.1 s (no interior reuse).
  - Fixed: second+ requests ~0.3 s (system prefix reused; only user tokens prefill).
- Watch `pop_n` vs `available_history_num` at the `can_rollback` call sites above.
- `~/mlc-llm/.envrc.local` sets the vendored TVM/mlc_llm env; serve cmd is in
  `src/alfie_llm/alfie_llm/mlc_llm_serve_node.py::_serve_cmd`.

## Constraints

- Orin (sm_87), unified memory; must keep `--model-lib` explicit (JIT re-resolves to a
  FlashInfer variant that segfaults on sm_87).
- Interactive single-stream use; ~800-token constant system prompt; short turns.
- Do **not** rely on `prefix_cache_max_num_recycling_seqs=-1` — sizes `max_num_sequence+N`
  KV slots to 0 and the engine reload deadlocks (server never binds :8000).
