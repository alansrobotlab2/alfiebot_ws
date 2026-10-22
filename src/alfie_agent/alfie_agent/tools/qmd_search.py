"""
qmd_search — semantic + keyword vault search via the QMD daemon.

Ported from lloyd's QMD client (agent_mcp/vault.py::_qmd_post). QMD is a local
search daemon over the obsidian vault providing hybrid BM25 (FTS5) + vector
(embeddinggemma) search with an optional cross-encoder reranker. We run lloyd's
fork (github.com/alansrobotlab2/qmd, branch ``lloyd``) rather than upstream
tobi/qmd: its in-memory vector index, honoured rerank switch, ``lexMode:"or"``
and per-phase ``meta`` timings all matter on the Orin's CPU (see qmd/README.md).
It speaks a plain ``POST /query`` REST endpoint; we send a lexical + vector search
pair and read back ``results[].{file,title,snippet,score}``.

``query()`` is the one low-level entry point, shared by the ``vault_search`` tool
and the per-turn prefetch (prefetch.py), which passes its own tight timeout.

Configured via ``configure(qmd_url)`` (the ROS node passes its ``qmd_url`` param).
If the daemon is down the tool returns a clear error dict rather than raising, so
the agent can tell the user search is unavailable instead of crashing the turn.
"""
import logging
import re
from urllib.parse import unquote

import requests

_log = logging.getLogger(__name__)

_QMD_URL = None
_SKIP_RERANK = False
# (connect, read). Set by configure() from the rerank mode:
#   * rerank OFF (default): lex+vec is fast — cold ~1.2 s (embedding-model load on
#     first query), warm ~75 ms. A tight read timeout means a hung daemon fails a
#     voice turn fast instead of blocking it.
#   * rerank ON: the 0.6 B cross-encoder runs per query on CPU here and measured
#     >30 s — it does NOT complete within any usable interactive budget. We give
#     it more headroom but warn loudly, because enabling it effectively breaks
#     vault_search on this hardware.
_TIMEOUT = (5, 10)
_READ_TIMEOUT_NO_RERANK = 10
_READ_TIMEOUT_RERANK = 45


def configure(qmd_url, skip_rerank=False):
    """Point vault search at the QMD daemon's /query URL."""
    global _QMD_URL, _SKIP_RERANK, _TIMEOUT
    _QMD_URL = qmd_url
    _SKIP_RERANK = bool(skip_rerank)
    read = _READ_TIMEOUT_NO_RERANK if _SKIP_RERANK else _READ_TIMEOUT_RERANK
    _TIMEOUT = (5, read)
    if not _SKIP_RERANK:
        _log.warning(
            "qmd_search: reranker ENABLED (skip_rerank=False). The 0.6B "
            "cross-encoder runs per query on CPU and measured >30 s here — "
            "vault_search will likely time out. Set qmd_skip_rerank=True.")


def _err(msg):
    return {"error": msg}


def warmup():
    """
    Fire one throwaway search so the daemon loads its embedding model now.

    The first real query is otherwise ~1.2 s (cold model load) vs ~75 ms warm;
    calling this at node startup moves that cost off the user's first turn. Best
    effort — a down or slow daemon is swallowed (the real query path reports it).
    """
    if not _QMD_URL:
        return
    try:
        _vault_search({"query": "warmup", "max_results": 1})
        _log.info("qmd_search: vault search warmed up")
    except Exception:  # never let warmup break startup
        pass


def query(searches, *, limit=5, collections=None, rerank=None, lex_or=True,
          timeout=None):
    """
    POST one structured search to the QMD daemon.

    ``searches`` is a list of ``{"type": "lex"|"vec", "query": str}`` legs.
    Returns ``{"results": [...], "meta": {...}}`` with vault-relative ``file``
    paths and cleaned snippets, or an ``{"error": ...}`` dict. Never raises.
    """
    if not _QMD_URL:
        return _err("vault search not configured (no qmd_url)")
    collections = collections or ["obsidian"]
    payload = {
        "searches": searches,
        "limit": int(limit),
        "collections": collections,
        # qmd's REST /query reads the boolean `rerank` field. The reranker is a
        # 0.6B cross-encoder run per query — ~18 s on CPU — so leaving it on
        # tanks latency. Off by default (unneeded in agent use).
        "rerank": (not _SKIP_RERANK) if rerank is None else bool(rerank),
    }
    if lex_or:
        # Fork-only: OR the BM25 terms. Spoken queries are loose natural
        # language, and upstream's AND semantics drop most of them.
        payload["lexMode"] = "or"
    try:
        r = requests.post(_QMD_URL, json=payload, timeout=timeout or _TIMEOUT)
        r.raise_for_status()
        data = r.json()
    except requests.exceptions.ConnectionError:
        return _err("vault search unavailable (qmd daemon not running)")
    except requests.exceptions.Timeout:
        return _err("vault search timed out")
    except Exception as e:
        return _err(f"vault search failed: {e}")

    results = [
        {
            "file": _strip_collection(row.get("file", ""), collections),
            "title": row.get("title", ""),
            "snippet": clean_snippet(row.get("snippet", "")),
            "score": row.get("score", 0),
        }
        for row in data.get("results", [])
    ]
    return {"results": results, "meta": data.get("meta") or {}}


def _vault_search(params):
    q = (params.get("query") or "").strip()
    if not q:
        return _err("query is required")
    res = query(
        [{"type": "lex", "query": q},    # BM25 / FTS5
         {"type": "vec", "query": q}],   # embedding similarity
        limit=int(params.get("max_results") or 5),
        collections=params.get("collections") or ["obsidian"],
    )
    if "error" in res:
        return res
    return {"query": q, "results": res["results"]}


# qmd snippets are line-numbered hunks: "1: @@ -1,3 @@ (0 before, 6 after)\n2: ..."
_HUNK_RE = re.compile(r"^@@ .*?@@.*$")
_LINENO_RE = re.compile(r"^\d+: ?")


def clean_snippet(snippet):
    """Strip qmd's hunk header, ``NN:`` line prefixes and frontmatter fences."""
    lines = []
    in_front = False
    for raw in (snippet or "").splitlines():
        line = _LINENO_RE.sub("", raw)
        if _HUNK_RE.match(line):
            continue
        if line.strip() == "---":
            in_front = not in_front
            continue
        if in_front or not line.strip():
            continue
        lines.append(line.strip())
    return " ".join(lines)


def _strip_collection(file, collections):
    """
    Drop qmd's leading ``<collection>/`` prefix so the path is vault-relative.

    qmd namespaces results by collection (e.g. ``qmd://obsidian/notes/x.md``;
    older builds omit the scheme), but vault_read expects paths relative to the
    vault root (``notes/x.md``). Strip the scheme and collection segment so the
    search -> read chain lines up.
    """
    if file.startswith("qmd://"):
        file = unquote(file[len("qmd://"):])
    for coll in collections:
        prefix = coll + "/"
        if file.startswith(prefix):
            return file[len(prefix):]
    return file


# --- MCP-shaped module interface (matches the other tool modules) ---

_TOOLS = [
    {
        "name": "vault_search",
        "description": (
            "Search the vault by meaning or keyword when you don't know the "
            "exact path; returns files with snippets. Follow up with vault_read."
        ),
        "parameters": {
            "type": "object",
            "properties": {
                "query": {"type": "string",
                          "description": "What to search for."},
                "max_results": {"type": "integer",
                                "description": "Max results to return (default 5)."},
            },
            "required": ["query"],
        },
    },
]

_HANDLERS = {"vault_search": _vault_search}


def list_tools():
    return list(_TOOLS)


def call_tool(name, arguments):
    handler = _HANDLERS.get(name)
    if handler is None:
        return _err(f"unknown tool: {name}")
    return handler(arguments or {})
