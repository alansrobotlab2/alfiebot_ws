"""Unit tests for the QMD vault_search client (HTTP mocked)."""
from unittest import mock

import requests

from alfie_agent.tools import qmd_search


def _fake_response(payload):
    resp = mock.Mock()
    resp.json.return_value = payload
    resp.raise_for_status.return_value = None
    return resp


def test_not_configured_returns_error():
    qmd_search.configure(None)
    r = qmd_search.call_tool("vault_search", {"query": "x"})
    assert "error" in r and "not configured" in r["error"]


def test_missing_query():
    qmd_search.configure("http://localhost:8181/query")
    assert "error" in qmd_search.call_tool("vault_search", {})


def test_search_builds_payload_and_parses_results():
    qmd_search.configure("http://localhost:8181/query")
    payload = {"results": [
        {"file": "notes/a.md", "title": "a", "snippet": "hello", "score": 0.9,
         "docid": "#1", "line": 1},
    ]}
    with mock.patch.object(qmd_search.requests, "post",
                           return_value=_fake_response(payload)) as post:
        out = qmd_search.call_tool("vault_search", {"query": "greeting", "max_results": 3})
    # request shape matches lloyd's QMD contract
    sent = post.call_args.kwargs["json"]
    assert sent["limit"] == 3
    assert sent["collections"] == ["obsidian"]
    assert [s["type"] for s in sent["searches"]] == ["lex", "vec"]
    assert all(s["query"] == "greeting" for s in sent["searches"])
    # response is narrowed to the four contract fields
    assert out["results"] == [
        {"file": "notes/a.md", "title": "a", "snippet": "hello", "score": 0.9}
    ]


def test_collection_prefix_stripped_from_paths():
    # qmd returns 'obsidian/notes/a.md'; vault_read needs 'notes/a.md'.
    qmd_search.configure("http://localhost:8181/query")
    payload = {"results": [
        {"file": "obsidian/notes/a.md", "title": "a", "snippet": "x", "score": 0.5},
    ]}
    with mock.patch.object(qmd_search.requests, "post",
                           return_value=_fake_response(payload)):
        out = qmd_search.call_tool("vault_search", {"query": "q"})
    assert out["results"][0]["file"] == "notes/a.md"


def test_skip_rerank_flag():
    qmd_search.configure("http://localhost:8181/query", skip_rerank=True)
    with mock.patch.object(qmd_search.requests, "post",
                           return_value=_fake_response({"results": []})) as post:
        qmd_search.call_tool("vault_search", {"query": "q"})
    assert post.call_args.kwargs["json"].get("skipRerank") is True


def test_daemon_down_returns_clean_error():
    qmd_search.configure("http://localhost:8181/query")
    with mock.patch.object(qmd_search.requests, "post",
                           side_effect=requests.exceptions.ConnectionError()):
        r = qmd_search.call_tool("vault_search", {"query": "q"})
    assert "error" in r and "not running" in r["error"]


def test_timeout_returns_clean_error():
    qmd_search.configure("http://localhost:8181/query")
    with mock.patch.object(qmd_search.requests, "post",
                           side_effect=requests.exceptions.Timeout()):
        r = qmd_search.call_tool("vault_search", {"query": "q"})
    assert "error" in r and "timed out" in r["error"]


def test_list_tools_shape():
    assert {t["name"] for t in qmd_search.list_tools()} == {"vault_search"}
