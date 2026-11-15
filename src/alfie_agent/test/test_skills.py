"""Unit tests for the skill registry (real task registry, tmp skill files)."""
import os
import shutil

import pytest

from alfie_agent import skills
from alfie_agent.prefetch import stems

PROMPT_SKILL = """
name: house_tour
kind: prompt
description: Show a visitor around the house.
triggers: [tour, visitor]
params:
  rooms: {type: integer, default: 2, min: 1, max: 5}
  guest: {type: string}
instructions: |
  Greet the guest by name, then identify each room and say one thing about it.
"""


@pytest.fixture
def reg():
    return skills.SkillRegistry()


@pytest.fixture
def tmp_reg(tmp_path):
    shutil.copy(skills.SKILL_DIR / "clean_room.yaml", tmp_path)
    errors = []
    return skills.SkillRegistry(skill_dir=tmp_path, log=errors.append), tmp_path, errors


def test_packaged_skills_and_generated_task_skills(reg):
    assert reg.names() == ["clean_room", "pick_can", "trash_can"]
    assert reg.get("pick_can")["steps"] == [{"task": "pick_can"}]
    assert skills.signature(reg.get("clean_room")) == "clean_room(rounds?:int)"
    assert skills.line(reg.get("pick_can")) == "pick_can() — Find a soda can and pick it up."


def test_resolve_builds_a_plan_from_the_task_registry(reg):
    plan = reg.resolve("clean_room")["plan"]
    assert plan["title"] == "clean the room" and plan["rounds"] == 3
    pick, trash = plan["steps"]
    assert pick["text"] == "find the can and pick it up" and pick["attempts"] == 2
    assert pick["holds_object"] and not trash["holds_object"]
    assert pick["pose"]["back"] == 0.34 and len(pick["pose"]["right_arm"]) == 6
    assert "can {round}" in pick["failed"] and plan["say"]["done"].startswith("Done.")
    assert reg.resolve("clean_room", {"rounds": 2})["plan"]["rounds"] == 2
    assert reg.resolve("clean_room", {"rounds": "4"})["plan"]["rounds"] == 4
    assert reg.resolve("clean_room", {"rounds": 5.0})["plan"]["rounds"] == 5
    single = reg.resolve("trash_can")["plan"]
    assert single["rounds"] == 1 and [s["id"] for s in single["steps"]] == ["trash_can"]


@pytest.mark.parametrize("args, error", [
    ({"rounds": 0}, "1 to 10"),
    ({"rounds": 11}, "1 to 10"),
    ({"rounds": 2.5}, "must be integer"),
    ({"rounds": "many"}, "must be integer"),
    ({"rounds": True}, "must be integer"),
    ({"cans": 2}, "takes ['rounds']"),
])
def test_resolve_rejects_bad_arguments(reg, args, error):
    assert error in reg.resolve("clean_room", args)["error"]


def test_resolve_unknown_skill_suggests_close_names(reg):
    res = reg.resolve("clean_the_room")
    assert "no skill named" in res["error"] and res["did_you_mean"][0] == "clean_room"
    assert "did_you_mean" not in reg.resolve("make me a sandwich")
    assert "error" in reg.resolve(None)


def test_search_ranks_by_shared_words(reg):
    def found(text):
        return [s["name"] for s in reg.search(stems(text))]
    assert found("clean the room") == ["clean_room"]
    assert found("clean up") == ["clean_room"]
    assert found("please tidy up in here") == ["clean_room"]
    assert found("how did the clean the room challenge go") == ["clean_room"]
    assert found("pick up the soda can")[0] == "pick_can"     # the more specific one
    assert found("what time is it") == []
    assert len(reg.search(stems("pick up cans and put them in the trash"), limit=2)) == 2


def test_listing(reg):
    assert reg.listing()["total"] == 3
    assert [s.split("(")[0] for s in reg.listing()["skills"]] == [
        "clean_room", "pick_can", "trash_can"]
    assert [s.split("(")[0] for s in reg.listing("tidy")["skills"]] == ["clean_room"]
    assert reg.listing("juggling")["skills"] == []


def test_prompt_skill_returns_instructions(tmp_reg):
    reg, path, errors = tmp_reg
    (path / "house_tour.yaml").write_text(PROMPT_SKILL)
    assert "house_tour" in reg.names() and not errors       # picked up without a restart
    assert "needs guest" in reg.resolve("house_tour")["error"]
    res = reg.resolve("house_tour", {"guest": "Maria"})
    assert res["instructions"].startswith("Greet the guest")
    assert res["args"] == {"rooms": 2, "guest": "Maria"} and "plan" not in res
    assert [s["name"] for s in reg.search(stems("give our visitor a tour"))] == ["house_tour"]


def test_changed_and_removed_files_are_reloaded(tmp_reg):
    reg, path, _ = tmp_reg
    f = path / "clean_room.yaml"
    f.write_text(f.read_text().replace("default: 3", "default: 5"))
    os.utime(f, ns=(1, 1))               # same-tick edits still change the stamp
    assert reg.resolve("clean_room")["plan"]["rounds"] == 5
    f.unlink()
    assert reg.names() == ["pick_can", "trash_can"]


@pytest.mark.parametrize("bad", [
    "name: x\nkind: dance\ndescription: d\n",
    "name: x\nkind: sequence\ndescription: d\nsteps: [juggle]\n",
    "name: x\nkind: sequence\ndescription: d\n",
    "name: x\nkind: sequence\ndescription: d\nsteps: [pick_can]\nrepeat: rounds\n",
    "name: x\nkind: sequence\nsteps: [pick_can]\n",
    "name: x\nkind: prompt\ndescription: d\n",
    "name: x\nkind: prompt\ndescription: d\ninstructions: i\nparams: {n: {type: list}}\n",
    "name: x\nkind: prompt\ndescription: d\ninstructions: i\ncolour: red\n",
    "kind: prompt\ndescription: d\ninstructions: i\n",
    "name: [unclosed\n",
])
def test_a_bad_file_is_skipped_and_reported(tmp_reg, bad):
    reg, path, errors = tmp_reg
    (path / "bad.yaml").write_text(bad)
    assert reg.names() == ["clean_room", "pick_can", "trash_can"]
    assert len(errors) == 1 and "bad.yaml" in errors[0]


def test_broken_task_registry_keeps_the_previous_skills(tmp_path):
    tasks = tmp_path / "tasks.yaml"
    from alfie_gr00t import task_registry
    shutil.copy(task_registry.default_path(), tasks)
    errors = []
    reg = skills.SkillRegistry(tasks_path=tasks, log=errors.append)
    tasks.write_text("tasks: {x: {text: ''}}\n")
    assert reg.names() == ["clean_room", "pick_can", "trash_can"]
    assert len(errors) == 1 and "task registry" in errors[0]
