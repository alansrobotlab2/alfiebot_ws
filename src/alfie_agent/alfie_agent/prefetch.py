"""
prefetch — per-turn automatic context injection ("subliminal", slimmed down).

Ported in design (not code) from lloyd's app/prefetch.py. Before each turn, a few
cheap retrieval legs run in parallel under a hard wall-clock budget, and whatever
finishes in time is rendered into a small ``<context>`` block that is prepended to
the model's copy of the user message — never the system prompt, so the pinned base
prefix (agent_node._pin_base_prefix) is untouched. History keeps exactly what was
said, as with the interruption note.

Legs (all CPU, no LLM call — the Orin has one LLM slot and the turn needs it):
  * **facts**   — entities named in the message (plus the recognized speaker,
                  else ``primary_user``, and ``User`` for first-person
                  questions) → up to 2 entities × 3 facts from memory/facts/.
                  Plain file reads, ~ms.
  * **vault**   — one keyword (BM25, OR-mode) qmd query over content words plus
                  the episode's focus keywords. Lexical on purpose: on a small
                  vault the vector leg returns *something* for every query. A hit
                  is kept only if its title+snippet holds at least two of the
                  message's own keywords (one, for a one-keyword message) — the
                  rank-fused qmd score is relative, so it cannot gate relevance.
  * **earlier** — temporal questions only ("yesterday", "last time"): episode
                  summaries older than the recent window already in the prompt.

Kept from lloyd: the budget (``budget_ms``; late legs are abandoned, not awaited),
``SessionFocus``-style keyword weights decaying per turn (so "what about the
other one?" keeps its topic), carrying a late vault result to the next turn, and
skipping short messages. Dropped: skills/backlog/ambient legs, the per-N-turn
topic-extraction LLM call, and the 2–8 KB block size — every injected token is
prefill latency here, so the block is capped at ``max_chars``.
"""
import re
import time
from concurrent.futures import ThreadPoolExecutor, wait
from datetime import datetime, timedelta, timezone

from alfie_agent import memory
from alfie_agent.tools import facts as fact_store
from alfie_agent.tools import qmd_search

FOCUS_DECAY = 0.75        # per turn, as lloyd
FOCUS_MIN_WEIGHT = 0.5
FOCUS_TOP_K = 3
MIN_CHARS = 10            # shorter messages ("stop", "look left") skip prefetch
CARRY_MAX_AGE_S = 900
VAULT_LIMIT = 2
SNIPPET_MAX = 180
FACTS_PER_ENTITY = 3
MAX_ENTITIES = 2
EARLIER_MAX = 3
EARLIER_DAYS = 3

# Files the system prompt already carries, or that the facts leg covers.
_VAULT_SKIP = ("memory/facts/", "alfie/MEMORY.md", "alfie/SOUL.md")

_STOPWORDS = set("""
a about above after again all also am an and any are as at be because been
before being below between both but by can could did do does doing down during
each few for from further had has have having he her here hers him his how i if
in into is it its itself just know let like me more most my myself no nor not
now of off on once only or other our ours out over own please really remember
same say she should so some such tell than that the their them then there these
they thing think this those through to too under until up very was we were what
when where which while who whom why will with would you your yours alfie hey hi
hello okay ok yeah yes um uh get got go going want see look
""".split())

# Only explicit references to a past conversation. lloyd's list also fires on
# "today"/"recent"/"before", which on a voice robot mostly means the weather.
_TEMPORAL_RE = re.compile(
    r"\b(yesterday|earlier|last\s+time|last\s+night|this\s+morning|"
    r"this\s+afternoon|this\s+evening|what\s+did\s+(?:we|i)|"
    r"did\s+(?:we|i)\s+(?:talk|discuss|say|tell|ask)|talked\s+about|"
    r"we\s+discussed)\b", re.I)
_FIRST_PERSON_RE = re.compile(r"\b(i|me|my|mine|i'm|i've|myself)\b", re.I)
_SECOND_PERSON_RE = re.compile(r"\b(you|your|yours|yourself)\b", re.I)
SELF_ENTITY = "Alfie"


def keywords(text):
    """Content words of ``text`` (lowercase, stopwords and short words removed)."""
    return [w for w in re.findall(r"[a-z0-9']+", (text or "").lower())
            if len(w) >= 3 and w not in _STOPWORDS]


def stem(word):
    """Crude suffix strip so "planning"/"plan" and "batteries"/"battery" match."""
    word = word.removesuffix("'s")
    for suffix, repl in (("ies", "y"), ("ing", ""), ("ed", ""), ("es", ""),
                         ("s", "")):
        if word.endswith(suffix) and len(word) - len(suffix) >= 3:
            word = word[:-len(suffix)] + repl
            break
    # "planning" -> "plann": undouble a trailing consonant.
    if len(word) > 3 and word[-1] == word[-2] and word[-1] not in "aeiouls":
        word = word[:-1]
    return word


def stems(text):
    """Stemmed content words of ``text``."""
    return {stem(w) for w in keywords(text)}


class Focus:
    """Keyword weights for the live episode, decayed every turn (lloyd's
    SessionFocus without the secondary-model topics)."""

    def __init__(self):
        self.weights = {}

    def update(self, text):
        for k in list(self.weights):
            self.weights[k] *= FOCUS_DECAY
            if self.weights[k] < 0.1:
                del self.weights[k]
        for kw in set(keywords(text)):
            self.weights[kw] = self.weights.get(kw, 0.0) + 1.0

    def top(self, exclude=()):
        ranked = sorted(self.weights.items(), key=lambda kv: -kv[1])
        return [k for k, w in ranked if w >= FOCUS_MIN_WEIGHT
                and k not in exclude][:FOCUS_TOP_K]


class Prefetcher:
    """Builds the per-turn ``<context>`` block. One instance per agent node."""

    def __init__(self, *, budget_ms=150, max_chars=1200, recent_window_min=30,
                 primary_user=""):
        self.primary_user = primary_user.strip()
        self.budget_s = budget_ms / 1000.0
        self.max_chars = max_chars
        self.recent_window_min = recent_window_min
        self._pool = ThreadPoolExecutor(max_workers=3,
                                        thread_name_prefix="prefetch")
        self.focus = Focus()
        self._pending = None      # (monotonic ts, vault future) from a late turn

    def reset(self):
        """New episode: forget focus and any carried-over result."""
        self.focus = Focus()
        self._pending = None

    # --- legs -----------------------------------------------------------------

    def _facts_leg(self, text, kws, speaker=""):
        lowered = (text or "").lower()
        picked = []
        for ent in fact_store.entities():
            if ent.lower() == "user":
                continue
            if re.search(rf"\b{re.escape(ent.lower())}\b", lowered):
                picked.append(ent)
        if _SECOND_PERSON_RE.search(text or ""):
            ent = fact_store.resolve_entity(SELF_ENTITY)
            if ent and ent not in picked:
                picked.append(ent)
        if _FIRST_PERSON_RE.search(text or ""):
            # "?" = an unrecognized voice: not primary_user, whoever that is.
            first = "" if speaker == "?" else (speaker or self.primary_user)
            for name in (first, "User"):
                ent = fact_store.resolve_entity(name) if name else None
                if ent and ent not in picked:
                    picked.append(ent)
        # Entities talked about earlier in the episode stay in play.
        for kw in self.focus.top():
            ent = fact_store.resolve_entity(kw)
            if ent and ent not in picked:
                picked.append(ent)
        want = {stem(k) for k in kws}
        out = []
        for ent in picked[:MAX_ENTITIES]:
            rows = fact_store.all_facts(ent)
            # Facts sharing a word with the message first, then the newest.
            rows.sort(key=lambda f: (len(want & stems(f.get("fact"))),
                                     str(f.get("created_at", ""))), reverse=True)
            for f in rows[:FACTS_PER_ENTITY]:
                out.append(f"{ent} ({f['category']}): {f.get('fact', '')}")
        return out

    def _vault_leg(self, query, kws):
        res = qmd_search.query([{"type": "lex", "query": query}],
                               limit=VAULT_LIMIT + 3, rerank=False,
                               timeout=(0.5, 2.0))
        if "error" in res:
            return {"error": res["error"]}
        cutoff = datetime.now(timezone.utc) - timedelta(minutes=self.recent_window_min)
        want = {stem(k) for k in kws}
        need = min(2, len(want))
        rows = []
        for r in res["results"]:
            path = r.get("file", "")
            if path.startswith(_VAULT_SKIP) or _recent_episode(path, cutoff):
                continue
            # Markdown headings repeat the title (episode notes: the timestamp).
            snippet = re.sub(r"(^|\s)#+ [^.]*?\d{2}:\d{2}\s*", " ",
                             r.get("snippet") or "").strip()
            have = stems(f"{r.get('title', '')} {snippet}")
            if len(have & want) < need:
                continue
            snippet = snippet[:SNIPPET_MAX]
            if snippet:
                rows.append(f"{r.get('title') or path} ({path}): {snippet}")
        return {"rows": rows[:VAULT_LIMIT], "ms": res.get("meta", {}).get("ms")}

    def _earlier_leg(self, text):
        if not _TEMPORAL_RE.search(text or ""):
            return []
        now = datetime.now(timezone.utc)
        newest = now - timedelta(minutes=self.recent_window_min)
        oldest = now - timedelta(days=EARLIER_DAYS)
        yesterday = None
        if re.search(r"\byesterday\b", text, re.I):
            yesterday = (now.astimezone() - timedelta(days=1)).date()
        rows = []
        for e in memory._read_recent_entries():
            try:
                ts = datetime.fromisoformat(e.get("ts", ""))
            except (TypeError, ValueError):
                continue
            local = ts.astimezone()
            if yesterday is not None and local.date() != yesterday:
                continue
            if oldest <= ts < newest and (e.get("summary") or "").strip():
                rows.append(f"{local.strftime('%a %H:%M')}: {e['summary'].strip()}")
        return rows[-EARLIER_MAX:]

    # --- public API -------------------------------------------------------------

    def build(self, text, speaker=""):
        """
        Return ``(context_block, stats)`` for this user message.

        ``speaker`` (a recognized voice) answers first-person questions in place
        of ``primary_user``; ``"?"`` (a voice speaker ID didn't recognize)
        suppresses ``primary_user``.

        ``context_block`` is "" when nothing relevant was found (or the message is
        too short); ``stats`` is a small dict for the turn's log line.
        """
        t0 = time.monotonic()
        self.focus.update(text)
        kws = keywords(text)
        stats = {"facts": 0, "vault": 0, "earlier": 0, "late": [], "carried": 0}
        if len((text or "").strip()) < MIN_CHARS or not kws:
            return "", {**stats, "skipped": True, "ms": 0}

        query = " ".join(kws + self.focus.top(exclude=set(kws)))
        futs = {
            "facts": self._pool.submit(self._facts_leg, text, kws, speaker),
            "vault": self._pool.submit(self._vault_leg, query, kws),
            "earlier": self._pool.submit(self._earlier_leg, text),
        }
        remaining = max(0.0, self.budget_s - (time.monotonic() - t0))
        wait(futs.values(), timeout=remaining)

        sections = {"facts": [], "vault": [], "earlier": []}
        for name, fut in futs.items():
            if not fut.done():
                stats["late"].append(name)
                continue
            try:
                res = fut.result()
            except Exception:  # a broken leg never breaks a turn
                continue
            if name == "vault":
                res = res.get("rows", []) if isinstance(res, dict) else []
            sections[name] = res
            stats[name] = len(res)

        # A vault result that missed the previous turn's budget rides now.
        carried = self._take_pending()
        for row in carried:
            if row not in sections["vault"]:
                sections["vault"].append(row)
                stats["carried"] += 1
        if "vault" in stats["late"]:
            self._pending = (time.monotonic(), futs["vault"])

        block = render(sections, self.max_chars)
        stats["ms"] = round((time.monotonic() - t0) * 1000)
        stats["chars"] = len(block)
        return block, stats

    def _take_pending(self):
        pending, self._pending = self._pending, None
        if pending is None:
            return []
        ts, fut = pending
        if time.monotonic() - ts > CARRY_MAX_AGE_S or not fut.done():
            return []
        try:
            res = fut.result()
        except Exception:
            return []
        return res.get("rows", []) if isinstance(res, dict) else []


def _recent_episode(path, cutoff):
    """True for an episode note newer than ``cutoff`` (already in the prompt)."""
    m = re.match(r"memory/episodes/(\d{4}-\d{2}-\d{2}-\d{6})\.md$", path)
    if not m:
        return False
    try:
        ts = datetime.strptime(m.group(1), "%Y-%m-%d-%H%M%S").replace(
            tzinfo=timezone.utc)
    except ValueError:
        return False
    return ts >= cutoff


_HEADINGS = {"facts": "Known facts:", "vault": "Notes:",
             "earlier": "Earlier conversations:"}


def render(sections, max_chars):
    """Render the legs into a ``<context>`` block of at most ``max_chars``.

    Sections are filled in priority order (facts, notes, earlier) and a row that
    would cross the cap is dropped whole, never cut mid-sentence.
    """
    head = ("<context>\nRetrieved automatically from your memory for this "
            "message. Use it only if it is relevant; do not mention it otherwise.")
    tail = "\n</context>"
    out = [head]
    used = len(head) + len(tail)
    for name in ("facts", "vault", "earlier"):
        rows = sections.get(name) or []
        if not rows:
            continue
        heading = "\n" + _HEADINGS[name]
        added = []
        for row in rows:
            line = "\n- " + row
            cost = len(line) + (0 if added else len(heading))
            if used + cost > max_chars:
                break
            added.append(line)
            used += cost
        if added:
            out.append(heading)
            out.extend(added)
    if len(out) == 1:
        return ""
    return "".join(out) + tail
