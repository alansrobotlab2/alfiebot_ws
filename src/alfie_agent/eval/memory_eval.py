#!/usr/bin/env python3
"""
memory_eval — does memory injection help Alfie answer, and what does it cost?

A small LloydMemEval-style A/B (lloyd: eval/run_memory_eval.py). Seeds a throwaway
vault (curated memory, lloyd-format facts, episode notes, a plain note), indexes
it with a private qmd daemon, then asks recall questions through the real
harness + MLC server under three arms:

  tools    base system prompt only; the model may still call vault_search /
           fact_get / memory tools (this is Alfie before this branch, plus tools)
  profile  + curated MEMORY.md and the recent-memory window in the system prompt
  full     + the per-turn prefetch <context> block (prefetch.py)

A reply is correct if it contains any expected keyword (case-insensitive).
Reports accuracy, mean turn time and tool calls per arm.

Requests are strictly sequential (the Orin's MLC server has one working slot
and deadlocks on concurrent forked requests). Do not run while Alfie is mid-
conversation. Needs the qmd fork built in ~/qmd and Node 24 in ~/.local/node.

    python3 src/alfie_agent/eval/memory_eval.py [--arms tools,profile,full]
"""
import argparse
import json
import os
import signal
import subprocess
import sys
import tempfile
import time
from datetime import datetime, timedelta, timezone
from pathlib import Path

import requests

from alfie_agent import harness, memory, prompt_builder, tools
from alfie_agent.llm_client import LLMClient
from alfie_agent.prefetch import Prefetcher
from alfie_agent.tools import facts, memory_tools

NODE = Path.home() / ".local/node/bin/node"
QMD = Path.home() / "qmd/bin/qmd"
QMD_PORT = 8182

MEMORY_ENTRIES = [
    "Alan built Alfie and is usually the one talking to him.",
    "Alan prefers short answers in the morning.",
]

FACTS = [
    ("Alan", "preference", "Alan's favorite soda is Diet Coke."),
    ("Alan", "preference", "Alan likes his coffee black."),
    ("Alan", "relationship", "Alan has a dog named Biscuit."),
    ("Biscuit", "general", "Biscuit is a golden retriever who is afraid of the vacuum."),
    ("Maria", "relationship", "Maria is Alan's sister and lives in Portland."),
    ("Maria", "preference", "Maria is allergic to peanuts."),
    ("Alfie", "hardware", "Alfie's right gripper was replaced with a softer silicone pad."),
]

NOTES = {
    "notes/garage.md": "# Garage inventory\n\nThe spare servo batteries are in "
                       "the blue bin on the top shelf of the garage.\n",
    "notes/wifi.md": "# Network\n\nThe workshop wifi network is called "
                     "Lab-Five and the router is behind the 3D printer.\n",
}

# Episodes: (hours ago, summary). Older than the 30-minute recent window except
# the last, which the profile arm's recent window carries.
EPISODES = [
    (26, "Alan asked Alfie to help plan a trip to the Oregon coast in March."),
    (22, "Alan and Alfie practiced picking up a red soda can from the kitchen table."),
    (0.2, "Alan said he was about to go repaint the workshop door green."),
]

QUESTIONS = [
    ("What's my favorite soda?", ["diet coke"]),
    ("How do I like my coffee?", ["black"]),
    ("What's my dog's name?", ["biscuit"]),
    ("What kind of dog is Biscuit?", ["golden", "retriever"]),
    ("What is Biscuit scared of?", ["vacuum"]),
    ("Where does Maria live?", ["portland"]),
    ("Is Maria allergic to anything?", ["peanut"]),
    ("Who is Maria?", ["sister"]),
    ("Who built you?", ["alan"]),
    ("Where are the spare servo batteries?", ["blue bin", "garage"]),
    ("What's the workshop wifi called?", ["lab-five", "lab five"]),
    ("What did we practice yesterday?", ["soda can", "can", "pick"]),
    ("What trip was I planning?", ["oregon", "coast"]),
    ("What color am I painting the workshop door?", ["green"]),
    ("What was changed on your right gripper?", ["silicone", "softer", "pad"]),
]


def seed_vault(root):
    tools.configure(root)
    for e in MEMORY_ENTRIES:
        memory_tools.call_tool("memory_add", {"entry": e})
    for ent, cat, text in FACTS:
        facts.add_facts(ent, cat, [text])
    for rel, text in NOTES.items():
        p = root / rel
        p.parent.mkdir(parents=True, exist_ok=True)
        p.write_text(text)
    now = datetime.now(timezone.utc)
    for hours, summary in EPISODES:
        memory.record_episode([{"role": "user", "content": "-"}],
                              lambda t, s=summary: {"summary": s, "facts": []},
                              now=now - timedelta(hours=hours))


def start_qmd(root, cfg):
    cfg.mkdir()
    (cfg / "index.yml").write_text(
        f"collections:\n  obsidian:\n    path: {root}\n    pattern: \"**/*.md\"\n")
    # INDEX_PATH as well as QMD_CONFIG_DIR: the config dir alone does NOT move
    # the index, and `qmd update` would repoint the live ~/.cache/qmd index's
    # `obsidian` collection at this throwaway vault.
    env = {**os.environ, "QMD_CONFIG_DIR": str(cfg),
           "INDEX_PATH": str(cfg / "index.sqlite"), "QMD_FORCE_CPU": "1",
           "QMD_LLAMA_GPU": "false", "QMD_HOST": "127.0.0.1",
           "PATH": f"{NODE.parent}:{os.environ['PATH']}"}
    for cmd in (["update"], ["embed"]):
        subprocess.run([str(NODE), str(QMD), *cmd], env=env, check=True,
                       stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    # Own session: bin/qmd re-execs into a child, so stop the whole group.
    proc = subprocess.Popen([str(NODE), str(QMD), "mcp", "--http", "--port",
                             str(QMD_PORT)], env=env, stdout=subprocess.DEVNULL,
                            stderr=subprocess.DEVNULL, start_new_session=True)
    url = f"http://127.0.0.1:{QMD_PORT}/query"
    for _ in range(100):
        try:
            requests.get(f"http://127.0.0.1:{QMD_PORT}/health", timeout=1)
            break
        except requests.RequestException:
            time.sleep(0.2)
    return proc, url


def run_arm(arm, llm, base_prompt, soul, specs):
    profile = memory.profile_text() if arm != "tools" else None
    recent = memory.recent_preamble() if arm != "tools" else None
    system = prompt_builder.build_system_prompt(soul, specs, recent_memory=recent,
                                                profile=profile)
    pf = Prefetcher(primary_user="Alan") if arm == "full" else None
    rows = []
    for q, expect in QUESTIONS:
        if pf:
            pf.reset()
        t0 = time.monotonic()
        context = pf.build(q)[0] if pf else ""
        prefetch_ms = (time.monotonic() - t0) * 1000
        calls = []

        def call_tool(name, args):
            calls.append(name)
            return tools.call_tool(name, args)

        text = f"{context}\n\n{q}" if context else q
        reply = harness.run_turn(text, [], system_prompt=system, llm=llm,
                                 call_tool=call_tool, is_current=lambda: True)
        dt = time.monotonic() - t0
        ok = any(k in (reply or "").lower() for k in expect)
        rows.append({"q": q, "ok": ok, "s": round(dt, 2), "tools": calls,
                     "prefetch_ms": round(prefetch_ms), "ctx_chars": len(context),
                     "reply": reply})
        print(f"  [{arm:7}] {'OK ' if ok else 'MISS'} {dt:5.2f}s "
              f"tools={','.join(calls) or '-':24} {q} -> {(reply or '')[:80]}")
    return rows


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--arms", default="tools,profile,full")
    ap.add_argument("--llm", default="http://localhost:8000/v1")
    ap.add_argument("--out", default=None, help="write per-question JSON here")
    args = ap.parse_args()

    with tempfile.TemporaryDirectory(prefix="alfie-memeval-") as tmp:
        root, cfg = Path(tmp) / "vault", Path(tmp) / "qmd"
        root.mkdir()
        seed_vault(root)
        proc, url = start_qmd(root, cfg)
        try:
            tools.configure(root, qmd_url=url, qmd_skip_rerank=True)
            tools.warmup_vault_search()
            specs = tools.list_tools()
            soul = prompt_builder.load_soul(root)
            base = prompt_builder.build_system_prompt(soul, specs)
            llm = LLMClient(args.llm, "dist/qwen3_6-35B-A3B-q4f16_1_fused",
                            timeout=(5, 60), temperature=0.0, max_tokens=200)
            results = {}
            for arm in args.arms.split(","):
                results[arm] = run_arm(arm, llm, base, soul, specs)
        finally:
            os.killpg(proc.pid, signal.SIGTERM)

    print("\narm      accuracy  mean_s  tool_calls/q  ctx_chars/q")
    for arm, rows in results.items():
        n = len(rows)
        print(f"{arm:8} {sum(r['ok'] for r in rows):3}/{n:<4}  "
              f"{sum(r['s'] for r in rows) / n:6.2f}  "
              f"{sum(len(r['tools']) for r in rows) / n:12.2f}  "
              f"{sum(r['ctx_chars'] for r in rows) / n:11.0f}")
    if args.out:
        Path(args.out).write_text(json.dumps(results, indent=2))
    return 0


if __name__ == "__main__":
    sys.exit(main())
