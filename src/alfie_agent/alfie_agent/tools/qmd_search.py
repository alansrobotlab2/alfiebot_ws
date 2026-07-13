"""
qmd_search — semantic + keyword vault search via the QMD daemon.

Ported from lloyd's QMD client (agent_mcp/vault.py::_qmd_post). QMD (tobi/qmd,
npm @tobilu/qmd) is a local search daemon over the obsidian vault providing hybrid
BM25 (FTS5) + vector (embeddinggemma) search with an optional cross-encoder
reranker. It speaks a plain ``POST /query`` REST endpoint; we send a lexical +
vector search pair and read back ``results[].{file,title,snippet,score}``.

Configured via ``configure(qmd_url)`` (the ROS node passes its ``qmd_url`` param).
If the daemon is down the tool returns a clear error dict rather than raising, so
the agent can tell the user search is unavailable instead of crashing the turn.
"""
import requests

_QMD_URL = None
_SKIP_RERANK = False
# (connect, read). Read is generous: a cold CPU query loads the embedding model on
# first use; the reranker adds more (skip it by default — see qmd_skip_rerank).
_TIMEOUT = (5, 30)


def configure(qmd_url, skip_rerank=False):
    """Point vault search at the QMD daemon's /query URL."""
    global _QMD_URL, _SKIP_RERANK
    _QMD_URL = qmd_url
    _SKIP_RERANK = bool(skip_rerank)


def _err(msg):
    return {"error": msg}


def _vault_search(params):
    if not _QMD_URL:
        return _err("vault search not configured (no qmd_url)")
    query = (params.get("query") or "").strip()
    if not query:
        return _err("query is required")
    limit = int(params.get("max_results") or 5)
    collections = params.get("collections") or ["obsidian"]

    payload = {
        "searches": [
            {"type": "lex", "query": query},   # BM25 / FTS5
            {"type": "vec", "query": query},   # embedding similarity
        ],
        "limit": limit,
        "collections": collections,
        # qmd's REST /query reads the boolean `rerank` field (NOT `skipRerank`,
        # which it silently ignores). The reranker is a 0.6B cross-encoder run per
        # query — ~18 s on CPU — so leaving it on tanks latency. Off by default
        # (unneeded in agent use); this is the single biggest latency lever.
        "rerank": not _SKIP_RERANK,
    }

    try:
        r = requests.post(_QMD_URL, json=payload, timeout=_TIMEOUT)
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
            "snippet": row.get("snippet", ""),
            "score": row.get("score", 0),
        }
        for row in data.get("results", [])
    ]
    return {"query": query, "results": results}


def _strip_collection(file, collections):
    """
    Drop qmd's leading ``<collection>/`` prefix so the path is vault-relative.

    qmd namespaces results by collection (e.g. ``obsidian/notes/x.md``), but
    vault_read expects paths relative to the vault root (``notes/x.md``). Strip
    the collection segment so the search -> read chain lines up.
    """
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
