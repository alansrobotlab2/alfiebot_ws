"""
web — web search and page fetch, ported (voice-sized) from lloyd's http_tools.py.

  * ``web_search(query)`` — DuckDuckGo via ``ddgs`` (no API key), the top 3
    results as title / url / snippet. Often enough to answer from directly.
  * ``web_fetch(url, question?)`` — GET the page and extract its readable text
    with trafilatura (lloyd's extractor), then return only the ~1500 characters
    most relevant to ``question`` (lloyd returns up to 50 KB; every character
    here is prefill latency before Alfie can speak).

Safety, from lloyd's egress rules:
  * only http(s), and every hop of a redirect chain must resolve to a public
    address — never loopback, private, link-local or reserved (the LLM server,
    qmd, the robot's own services and the LAN are off limits);
  * capped download size and timeouts;
  * results are web content, i.e. untrusted: the harness refuses motion tools
    for the rest of a turn once a ``TAINTING_TOOLS`` result has entered it.
"""
import ipaddress
import re
import socket
from urllib.parse import urljoin, urlparse

import requests

TAINTING_TOOLS = frozenset({"web_search", "web_fetch"})

_UA = ("Mozilla/5.0 (X11; Linux aarch64) AppleWebKit/537.36 (KHTML, like Gecko) "
       "Chrome/131.0.0.0 Safari/537.36")
_TIMEOUT = (4, 8)
_MAX_BYTES = 2_000_000
_MAX_REDIRECTS = 5
_RESULTS = 3
_PASSAGE_CHARS = 1500


def _err(msg):
    return {"error": msg}


# --- search -----------------------------------------------------------------------

def _web_search(params):
    query = " ".join(str(params.get("query") or "").split())
    if not query:
        return _err("query is required")
    try:
        from ddgs import DDGS
        raw = list(DDGS(timeout=6).text(query, max_results=_RESULTS))
    except ImportError:
        return _err("web search isn't installed (pip: ddgs).")
    except Exception as e:
        return _err(f"web search failed: {type(e).__name__}")
    if not raw:
        return {"query": query, "results": [], "note": "no results"}
    return {"query": query, "results": [
        {"title": r.get("title", ""), "url": r.get("href", ""),
         "snippet": (r.get("body") or "")[:300]} for r in raw]}


# --- fetch ------------------------------------------------------------------------

def public_host(host):
    """True if every address ``host`` resolves to is globally routable."""
    try:
        infos = socket.getaddrinfo(host, None, proto=socket.IPPROTO_TCP)
    except (socket.gaierror, UnicodeError):
        return False
    if not infos:
        return False
    for info in infos:
        addr = ipaddress.ip_address(info[4][0].split("%", 1)[0])
        if not addr.is_global or addr.is_multicast:
            return False
    return True


def _check_url(url):
    p = urlparse(url)
    if p.scheme not in ("http", "https") or not p.hostname:
        return "only http(s) web addresses can be fetched"
    if not public_host(p.hostname):
        return "that address is local or private; I only fetch public web pages"
    return None


def _get(url):
    """GET with the public-address check on every redirect hop; returns (url, html)."""
    for _ in range(_MAX_REDIRECTS + 1):
        bad = _check_url(url)
        if bad:
            raise PermissionError(bad)
        r = requests.get(url, headers={"User-Agent": _UA}, timeout=_TIMEOUT,
                         allow_redirects=False, stream=True)
        if r.is_redirect or r.status_code in (301, 302, 303, 307, 308):
            url = urljoin(url, r.headers.get("location", ""))
            r.close()
            continue
        r.raise_for_status()
        body = b""
        for chunk in r.iter_content(65536):
            body += chunk
            if len(body) > _MAX_BYTES:
                break
        r.close()
        return url, body.decode(r.encoding or "utf-8", errors="replace")
    raise PermissionError("too many redirects")


def best_passage(text, question, limit=_PASSAGE_CHARS):
    """The window of paragraphs sharing the most words with ``question``.

    Without a question, the start of the page. Never cuts mid-paragraph unless a
    single paragraph is over the limit.
    """
    from alfie_agent.prefetch import stems   # lazy: prefetch imports tools
    paras = [p.strip() for p in re.split(r"\n\s*\n|\n", text) if p.strip()]
    if not paras:
        return ""
    words = stems(question or "")
    para_stems = [stems(p) for p in paras]
    best, best_score = 0, -1
    if words:
        # Score a window by how many distinct question words it covers, so a
        # paragraph answering the question beats several that repeat one word.
        for i in range(len(paras)):
            covered, size = set(), 0
            for p, ps in zip(paras[i:], para_stems[i:]):
                if size + len(p) > limit and size:
                    break
                covered |= words & ps
                size += len(p) + 1
            if len(covered) > best_score:
                best, best_score = i, len(covered)
    out, size = [], 0
    for p in paras[best:]:
        if size + len(p) > limit:
            if not out:
                out.append(p[:limit])
            break
        out.append(p)
        size += len(p) + 1
    return "\n".join(out)


def _web_fetch(params):
    url = str(params.get("url") or "").strip()
    if not url:
        return _err("url is required")
    if not urlparse(url).scheme:
        url = "https://" + url
    try:
        import trafilatura
    except ImportError:
        return _err("page reading isn't installed (pip: trafilatura).")
    try:
        final, html = _get(url)
    except PermissionError as e:
        return _err(str(e))
    except requests.exceptions.RequestException as e:
        return _err(f"couldn't fetch that page ({type(e).__name__}).")
    text = trafilatura.extract(html, include_comments=False, include_tables=True) or ""
    if not text.strip():
        return _err("that page had no readable text.")
    title = ""
    m = re.search(r"<title[^>]*>(.*?)</title>", html, re.I | re.S)
    if m:
        title = " ".join(m.group(1).split())[:120]
    return {"url": final, "title": title,
            "text": best_passage(text, params.get("question"))}


_TOOLS = [
    {
        "name": "web_search",
        "description": "Search the web for current or general facts you don't "
                       "know (news, hours, prices, how-tos). Returns top results "
                       "with snippets.",
        "parameters": {
            "type": "object",
            "properties": {"query": {"type": "string"}},
            "required": ["query"],
        },
    },
    {
        "name": "web_fetch",
        "description": "Read a web page (e.g. a web_search result); pass the "
                       "question to get the most relevant passage.",
        "parameters": {
            "type": "object",
            "properties": {"url": {"type": "string"},
                           "question": {"type": "string"}},
            "required": ["url"],
        },
    },
]

_HANDLERS = {"web_search": _web_search, "web_fetch": _web_fetch}


def list_tools():
    return list(_TOOLS)


def call_tool(name, arguments):
    handler = _HANDLERS.get(name)
    if handler is None:
        return _err(f"unknown tool: {name}")
    return handler(arguments or {})
