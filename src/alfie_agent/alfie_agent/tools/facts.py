"""
facts — entity facts in lloyd's fact-file format, plus the ``fact_get`` tool.

Durable facts extracted at idle compaction (``memory.record_episode``) are filed
per entity and category, in the same shape lloyd's extractor writes
(scripts/memory/next-gen-memory/fact_extractor.py::write_fact_file)::

    memory/facts/<Entity>/<Entity>-<category>.md
    ---
    type: facts
    entity: Alan
    category: preference
    facts:
    - id: pref-001
      fact: Alan likes Diet Coke.
      confidence: 0.8
      created_at: 2026-10-22T...
      source_doc: memory/episodes/2026-10-22-....md
      provenance: ALFIE_VOICE
      expired_at: null
      invalid_at: null
    last_updated: ...
    ---
    # Alan - Preference ...

Same frontmatter, category vocabulary and ``<prefix>-NNN`` id scheme, so the
files stay readable by lloyd's tooling if they are ever synced across. Only the
cheap half of lloyd's pipeline lives here: exact-duplicate refusal and the
bitemporal fields. Entity resolution, contradiction handling and graph edges are
left to lloyd's nightly pipeline (too heavy for the Orin's single LLM slot).

Writes go through the audited ``vault`` tools; reads go straight to disk (the
prefetch facts leg must stay ~ms, see prefetch.py).
"""
import re
from datetime import datetime, timezone

import yaml

from . import vault

FACTS_DIR = "memory/facts"

# lloyd's CATEGORY_VOCAB and fact-id prefixes (app/fact_ids.py).
CATEGORY_PREFIX = {
    "state": "stat", "event": "evnt", "decision": "deci", "preference": "pref",
    "goal": "goal", "skill": "skil", "relationship": "rel", "capability": "capa",
    "constraint": "cons", "configuration": "conf", "hardware": "hard",
    "research": "resr", "general": "fact",
}
_ALIASES = {"preferences": "preference", "likes": "preference",
            "relationships": "relationship", "relations": "relationship",
            "events": "event", "goals": "goal", "skills": "skill",
            "config": "configuration", "status": "state", "fact": "general",
            "facts": "general", "info": "general"}

# Entity names become directory names: keep letters, digits, space, - and _.
_ENTITY_BAD = re.compile(r"[^\w \-]")
_FRONT_RE = re.compile(r"^---\n(.*?)\n---\n", re.DOTALL)


def normalize_category(raw):
    c = (raw or "general").strip().lower()
    c = _ALIASES.get(c, c)
    return c if c in CATEGORY_PREFIX else "general"


def normalize_entity(raw):
    name = " ".join(_ENTITY_BAD.sub("", raw or "").split())[:48]
    return name


def _now():
    return datetime.now(timezone.utc).isoformat()


def _next_id(facts, category):
    prefix = CATEGORY_PREFIX[category]
    nums = [0]
    for f in facts:
        m = re.match(rf"^{prefix}-(\d+)$", str(f.get("id", "")))
        if m:
            nums.append(int(m.group(1)))
    return f"{prefix}-{max(nums) + 1:03d}"


def _norm_text(text):
    return " ".join(re.findall(r"\w+", (text or "").lower()))


def parse_fact_file(text):
    """Return the frontmatter dict of a fact file ({} if unparseable)."""
    m = _FRONT_RE.match(text or "")
    if not m:
        return {}
    try:
        data = yaml.safe_load(m.group(1))
    except yaml.YAMLError:
        return {}
    return data if isinstance(data, dict) else {}


def render_fact_file(entity, category, facts, now):
    front = {"type": "facts", "entity": entity, "category": category,
             "facts": facts, "last_extracted": now, "last_updated": now}
    lines = [f"# {entity} - {category.title()}", "",
             f"**Entity:** {entity}", f"**Category:** {category}",
             f"**Fact Count:** {len(facts)}", "", "## Facts", ""]
    for f in facts:
        lines += [f"### {f.get('id')}", "", f"**Fact:** {f.get('fact', '')}",
                  f"**Confidence:** {f.get('confidence', 0.0)}",
                  f"**Status:** {'invalid' if f.get('invalid_at') else 'current'}",
                  ""]
    return ("---\n" + yaml.safe_dump(front, default_flow_style=False,
                                     sort_keys=False, allow_unicode=True)
            + "---\n\n" + "\n".join(lines))


def _rel(entity, category):
    return f"{FACTS_DIR}/{entity}/{entity}-{category}.md"


def _read(rel):
    res = vault.call_tool("vault_read", {"path": rel})
    if not isinstance(res, dict) or "error" in res:
        return []
    facts = parse_fact_file(res.get("text", "")).get("facts") or []
    return [f for f in facts if isinstance(f, dict)]


def add_facts(entity, category, texts, *, source_doc=None, confidence=0.8):
    """
    File ``texts`` under ``entity``/``category``; returns the number added.

    Refuses exact (normalized) duplicates the entity already holds in any
    category, as lloyd's ``_refuse_held_facts`` does.
    """
    entity = normalize_entity(entity)
    category = normalize_category(category)
    if not entity:
        return 0
    held = {_norm_text(f.get("fact")) for f in all_facts(entity)}
    rel = _rel(entity, category)
    facts = _read(rel)
    now = _now()
    added = 0
    for text in texts:
        text = " ".join(str(text or "").split())
        key = _norm_text(text)
        if not key or key in held:
            continue
        held.add(key)
        fact = {"id": _next_id(facts, category), "fact": text,
                "confidence": confidence, "created_at": now,
                "provenance": "ALFIE_VOICE", "expired_at": None,
                "invalid_at": None}
        if source_doc:
            fact["source_doc"] = source_doc
        facts.append(fact)
        added += 1
    if added:
        vault.call_tool("vault_write", {
            "path": rel, "content": render_fact_file(entity, category, facts, now)})
    return added


# --- reads (direct disk; fast path for prefetch) ------------------------------

def _facts_root():
    return vault._VAULT / FACTS_DIR if vault._VAULT is not None else None


def entities():
    """Return the known entity names (fact directory names)."""
    root = _facts_root()
    if root is None or not root.is_dir():
        return []
    return sorted(p.name for p in root.iterdir() if p.is_dir())


def all_facts(entity, category=None, include_invalid=False):
    """Return an entity's current facts as dicts with a ``category`` key added."""
    root = _facts_root()
    if root is None:
        return []
    edir = root / entity
    if not edir.is_dir():
        return []
    out = []
    for path in sorted(edir.glob("*.md")):
        try:
            data = parse_fact_file(path.read_text(encoding="utf-8"))
        except OSError:
            continue
        cat = data.get("category") or "general"
        if category and cat != normalize_category(category):
            continue
        for f in data.get("facts") or []:
            if not isinstance(f, dict):
                continue
            if not include_invalid and (f.get("invalid_at") or f.get("expired_at")):
                continue
            out.append({**f, "category": cat})
    return out


def resolve_entity(name):
    """Case-insensitive match of ``name`` to a known entity, or None."""
    want = normalize_entity(name).lower()
    for e in entities():
        if e.lower() == want:
            return e
    return None


# --- MCP-shaped module interface ---

def _fact_get(params):
    name = (params.get("entity") or "").strip()
    if not name:
        return {"error": "entity is required"}
    entity = resolve_entity(name)
    if entity is None:
        return {"entity": name, "facts": [],
                "note": "no facts on file; known entities: "
                        + (", ".join(entities()[:20]) or "none")}
    facts = all_facts(entity, params.get("category"))
    if not facts and params.get("category"):
        # The model guesses categories ("location"); a miss on the guess should
        # not read as "nothing known".
        facts = all_facts(entity)
    return {"entity": entity,
            "facts": [{"category": f["category"], "fact": f.get("fact", "")}
                      for f in facts[-20:]]}


_TOOLS = [
    {
        "name": "fact_get",
        "description": "Look up what you know about a person, place or thing "
                       "by name (optional category, e.g. preference).",
        "parameters": {
            "type": "object",
            "properties": {"entity": {"type": "string"},
                           "category": {"type": "string"}},
            "required": ["entity"],
        },
    },
]


def list_tools():
    return list(_TOOLS)


def call_tool(name, arguments):
    if name != "fact_get":
        return {"error": f"unknown tool: {name}"}
    return _fact_get(arguments or {})
