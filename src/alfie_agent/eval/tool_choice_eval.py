#!/usr/bin/env python3
"""
tool_choice_eval — does the model pick the right tool for a spoken request?

Each request runs as a fresh one-turn conversation through the real harness and
MLC server with the agent's real system prompt; tools are faked (they record the
call and return a plausible result), so nothing moves and nothing is fetched. A
case passes if the first tool called is the expected one (None = no tool; a
tuple lists every acceptable first tool). Each request goes through the real
prefetch with the real skill registry, as in the agent, so the skills leg is
part of what is tested: a command must start a skill, a question about one
must not.

Strictly sequential requests (one MLC slot). Don't run mid-conversation.

    PYTHONPATH=src/alfie_agent:src/alfie_gr00t \
        python3 src/alfie_agent/eval/tool_choice_eval.py
"""
import argparse
import sys
import time

from alfie_agent import harness, prompt_builder, tools
from alfie_agent.llm_client import LLMClient
from alfie_agent.prefetch import Prefetcher
from alfie_agent.skills import SkillRegistry

CASES = [
    ("What's the weather like in Portland Oregon today?", "weather"),
    ("Do I need an umbrella tomorrow?", "weather"),
    ("How are you doing? Check your systems.", "robot_status"),
    ("Are your motors too hot?", "robot_status"),
    ("Set a timer for ten minutes for the pasta.", "timer"),
    ("How long is left on my timer?", "timer"),
    ("Cancel the pasta timer.", "timer"),
    ("Look to your left.", "turn_head"),
    ("Look at the red cup.", "turn_head"),
    ("Nod your head.", "gesture"),
    ("Pick up the soda can.", "skill"),
    ("Stop what you're doing with your arms.", "task"),
    # Skills: a command runs one ...
    ("Clean the room.", "skill"),
    ("Please tidy up this room.", "skill"),
    ("Clean up.", "skill"),
    ("Can you pick up two cans and throw them away?", "skill"),
    # ... a question about one, or about a past run, does not.
    ("How did the clean the room challenge go?", (None, "task")),
    ("What do you do when you clean the room?", (None, "skill_list")),
    ("Tell me about the clean the room challenge.", (None, "skill_list", "task")),
    ("Are you good at picking up cans?", (None, "skill_list")),
    ("Is the room clean?", (None, "look")),
    ("Which room are we in?", "identify_room"),
    ("What can you do?", (None, "skill_list")),
    ("Who won the most recent World Series?", "web_search"),
    ("What time does the Oregon Zoo open?", "web_search"),
    ("Ask Lloyd what's on my calendar tomorrow.", "ask_lloyd"),
    ("Did I get any email from Maria today?", "ask_lloyd"),
    ("What do you see?", "look"),
    ("What's seventeen times twenty three?", "calculate"),
    ("Remember that my sister Maria is allergic to peanuts.", "memory_add"),
    ("Learn my voice, I'm Alan.", "learn_voice"),
    ("Tell me a joke.", None),
    ("What's the capital of France?", None),
    ("Good morning!", None),
]

FAKE = {
    "weather": {"place": "Portland, Oregon", "now": {"conditions": "overcast", "temperature": "59F"}},
    "robot_status": {"estop": "clear", "hottest_servo_c": 41, "cpu_c": 52},
    "timer": {"ok": True},
    "turn_head": {"ok": True},
    "gesture": {"ok": True},
    "task": {"state": "idle", "active": False,
             "last": {"task": "clean the room", "ago_s": 240,
                      "result": "Done. I put 3 cans in the trash. The room is clean."}},
    "skill": {"ok": True, "started": "clean the room", "rounds": 3, "limit_s": 774},
    "skill_list": {"skills": ["clean_room(rounds?:int) — Find soda cans, pick each "
                              "one up and put it in the trash can."], "total": 3},
    "identify_room": {"room": "living room", "confidence": 0.8},
    "web_search": {"results": [{"title": "Result", "url": "https://example.com", "snippet": "An answer."}]},
    "ask_lloyd": {"answer": "Nothing scheduled."},
    "look": {"seeing": True, "summary": "a cup (ahead of you, close)"},
    "calculate": {"result": 391},
    "memory_add": {"success": True},
    "learn_voice": {"ok": True, "result": "learned Alan's voice"},
}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--llm", default="http://localhost:8000/v1")
    args = ap.parse_args()
    tools.configure("/nonexistent-vault")
    specs = tools.list_tools()
    system = prompt_builder.build_system_prompt(
        prompt_builder.load_soul(None), specs)
    llm = LLMClient(args.llm, "dist/qwen3_6-35B-A3B-q4f16_1_fused",
                    timeout=(5, 60), temperature=0.0, max_tokens=200)
    prefetcher = Prefetcher(skills=SkillRegistry())
    passed = 0
    for text, want in CASES:
        calls = []

        def call_tool(name, a):
            calls.append(name)
            return FAKE.get(name, {"ok": True})
        t0 = time.monotonic()
        prefetcher.reset()
        context, _ = prefetcher.build(text)
        model_text = f"{context}\n\n{text}" if context else text
        reply = harness.run_turn(model_text, [], system_prompt=system, llm=llm,
                                 call_tool=call_tool, is_current=lambda: True,
                                 max_tool_iters=2)
        got = calls[0] if calls else None
        ok = got in want if isinstance(want, tuple) else got == want
        passed += ok
        print(f"{'OK  ' if ok else 'MISS'} {time.monotonic() - t0:4.1f}s "
              f"want={str(want):12.12} got={','.join(calls) or '-':22} {text} -> "
              f"{(reply or '')[:60]}")
    print(f"\n{passed}/{len(CASES)} correct")
    return 0


if __name__ == "__main__":
    sys.exit(main())
