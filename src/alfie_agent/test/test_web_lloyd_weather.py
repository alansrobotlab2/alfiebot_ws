"""Unit tests for web, lloyd and weather tools (network mocked)."""
from unittest import mock

import pytest

from alfie_agent.tools import lloyd, weather, web


# --- web --------------------------------------------------------------------------

def test_best_passage_picks_relevant_window():
    text = "\n".join(["Intro about the zoo."] * 5
                     + ["The zoo opens at 9 am and closes at 4 pm daily."]
                     + ["Unrelated footer."] * 5)
    out = web.best_passage(text, "what time does the zoo open", limit=120)
    assert "opens at 9 am" in out and len(out) <= 120


def test_best_passage_without_question_is_page_start():
    assert web.best_passage("first\nsecond\nthird", None, limit=12) == "first\nsecond"


@pytest.mark.parametrize("url", ["http://localhost:8000/", "http://127.0.0.1/",
                                 "http://10.0.0.5/", "http://[::1]/",
                                 "ftp://example.com/x", "http://169.254.169.254/"])
def test_fetch_refuses_local_and_odd_urls(url):
    assert "error" in web.call_tool("web_fetch", {"url": url})


def test_fetch_rechecks_redirect_targets():
    resp = mock.Mock(is_redirect=True, status_code=302,
                     headers={"location": "http://127.0.0.1:8000/"})
    with mock.patch.object(web, "public_host", side_effect=lambda h: h != "127.0.0.1"), \
            mock.patch.object(web.requests, "get", return_value=resp):
        r = web.call_tool("web_fetch", {"url": "https://example.com/"})
    assert "local or private" in r["error"]


def test_search_shapes_results():
    fake = mock.Mock()
    fake.return_value.text.return_value = [
        {"title": "T", "href": "https://x", "body": "b" * 400}]
    with mock.patch.dict("sys.modules", {"ddgs": mock.Mock(DDGS=fake)}):
        r = web.call_tool("web_search", {"query": "  zoo  hours "})
    assert r["query"] == "zoo hours"
    assert r["results"] == [{"title": "T", "url": "https://x", "snippet": "b" * 300}]


# --- lloyd ------------------------------------------------------------------------

def test_parse_sse_events():
    lines = ["event: text_delta", 'data: {"text": "hi"}', "",
             "event: done", 'data: {"response": "Done.", "session_id": "s1"}', ""]
    assert list(lloyd.parse_sse(lines)) == [
        ("text_delta", {"text": "hi"}),
        ("done", {"response": "Done.", "session_id": "s1"})]


def test_speakable_strips_markdown():
    out = lloyd.speakable("**Yes.** See [the doc](http://x).\n- one\n- two\n```code```")
    assert out == "Yes. See the doc. one two"


def test_ask_lloyd_flow_keeps_session():
    lloyd.configure("http://lloyd:8080")
    lloyd.reset_session()
    resp = mock.MagicMock(status_code=200)
    resp.__enter__.return_value = resp
    resp.iter_lines.return_value = iter([
        "event: done", 'data: {"response": "You have 2 meetings.", "session_id": "abc"}', ""])
    with mock.patch.object(lloyd.requests, "post", return_value=resp) as post:
        r = lloyd.call_tool("ask_lloyd", {"request": "what's on my calendar"})
        assert r == {"answer": "You have 2 meetings."}
        sent = post.call_args.kwargs["json"]
        assert sent["session_id"] == "" and sent["text"].endswith("what's on my calendar")
        resp.iter_lines.return_value = iter(["event: done", 'data: {"response": "ok"}', ""])
        lloyd.call_tool("ask_lloyd", {"request": "and tomorrow"})
        assert post.call_args.kwargs["json"]["session_id"] == "abc"


def test_ask_lloyd_unconfigured_and_refused():
    lloyd.configure("")
    assert "not connected" in lloyd.call_tool("ask_lloyd", {"request": "x"})["error"]
    lloyd.configure("http://lloyd:8080")
    resp = mock.MagicMock(status_code=403)
    resp.__enter__.return_value = resp
    with mock.patch.object(lloyd.requests, "post", return_value=resp):
        assert "trusted network" in lloyd.call_tool("ask_lloyd", {"request": "x"})["error"]


# --- weather ----------------------------------------------------------------------

def test_weather_needs_a_place():
    weather.configure(home="")
    assert "home location" in weather.call_tool("weather", {})["error"]


def test_weather_formats_forecast():
    weather.configure(home="Portland, Oregon")
    weather._geo_cache.clear()
    geo = mock.Mock(**{"json.return_value": {"results": [
        {"name": "Portland", "admin1": "Maine", "country": "US", "latitude": 1, "longitude": 2},
        {"name": "Portland", "admin1": "Oregon", "country": "US", "latitude": 3, "longitude": 4}]}})
    fc = mock.Mock(**{"json.return_value": {
        "current": {"temperature_2m": 59.4, "apparent_temperature": 57, "weather_code": 3,
                    "wind_speed_10m": 6},
        "daily": {"weather_code": [61, 0], "temperature_2m_max": [62, 58],
                  "temperature_2m_min": [46, 42],
                  "precipitation_probability_max": [80, 1]}}})
    with mock.patch.object(weather.requests, "get", side_effect=[geo, fc]) as get:
        r = weather.call_tool("weather", {})
    assert get.call_args_list[1].kwargs["params"]["latitude"] == 3
    assert r["place"] == "Portland, Oregon"
    assert r["now"]["temperature"] == "59F" and r["today"]["conditions"] == "light rain"
