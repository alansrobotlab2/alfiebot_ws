"""
weather — current conditions and a short forecast via Open-Meteo (no API key).

Two GETs: Open-Meteo's geocoder turns a place name into coordinates (cached),
then the forecast API returns current conditions plus today and tomorrow. With no
place given, ``configure(home=...)`` (the ``weather_location`` ROS param) is used.
Units follow ``configure(units=...)``: "fahrenheit" (default, with mph) or
"celsius" (with km/h). A failure is a clean ``{"error": ...}``.
"""
import requests

_GEO_URL = "https://geocoding-api.open-meteo.com/v1/search"
_FORECAST_URL = "https://api.open-meteo.com/v1/forecast"
_TIMEOUT = (3, 6)

_home = ""
_units = "fahrenheit"
_geo_cache = {}

# WMO weather interpretation codes, as Open-Meteo documents them.
_WMO = {
    0: "clear", 1: "mostly clear", 2: "partly cloudy", 3: "overcast",
    45: "foggy", 48: "foggy with rime", 51: "light drizzle", 53: "drizzle",
    55: "heavy drizzle", 56: "freezing drizzle", 57: "freezing drizzle",
    61: "light rain", 63: "rain", 65: "heavy rain", 66: "freezing rain",
    67: "freezing rain", 71: "light snow", 73: "snow", 75: "heavy snow",
    77: "snow grains", 80: "light showers", 81: "showers", 82: "heavy showers",
    85: "snow showers", 86: "heavy snow showers", 95: "thunderstorms",
    96: "thunderstorms with hail", 99: "thunderstorms with hail",
}


def configure(home="", units="fahrenheit"):
    global _home, _units
    _home = (home or "").strip()
    _units = "celsius" if str(units).lower().startswith("c") else "fahrenheit"


def _err(msg):
    return {"error": msg}


def _splits(place):
    """(name, hint) candidates: "Portland, Oregon" -> ("Portland", "Oregon");
    without a comma, "Portland Oregon" tries the whole string, then each split
    ("Portland" + "Oregon"), since the geocoder only matches a bare name."""
    if "," in place:
        name, _, hint = place.partition(",")
        return [(name.strip(), hint.strip())]
    words = place.split()
    return [(place, "")] + [(" ".join(words[:i]), " ".join(words[i:]))
                            for i in range(len(words) - 1, 0, -1)]


def _geocode(place):
    key = place.lower()
    if key in _geo_cache:
        return _geo_cache[key]
    top = None
    for name, hint in _splits(place):
        r = requests.get(_GEO_URL, params={"name": name, "count": 10,
                                           "language": "en"}, timeout=_TIMEOUT)
        r.raise_for_status()
        rows = r.json().get("results") or []
        hint = hint.lower()
        if hint:
            rows = [x for x in rows if hint in " ".join(
                str(x.get(k, "")) for k in ("admin1", "country", "country_code")).lower()]
        if rows:
            top = rows[0]
            break
    if top is None:
        return None
    loc = {"lat": top["latitude"], "lon": top["longitude"],
           "name": ", ".join(str(top[k]) for k in ("name", "admin1") if top.get(k))}
    _geo_cache[key] = loc
    return loc


def _weather(params):
    place = (params.get("place") or "").strip() or _home
    if not place:
        return _err("I don't have a home location set; ask which city.")
    try:
        loc = _geocode(place)
        if loc is None:
            return _err(f"I couldn't find a place called {place}.")
        imperial = _units == "fahrenheit"
        r = requests.get(_FORECAST_URL, params={
            "latitude": loc["lat"], "longitude": loc["lon"],
            "current": "temperature_2m,apparent_temperature,weather_code,"
                       "wind_speed_10m,precipitation",
            "daily": "weather_code,temperature_2m_max,temperature_2m_min,"
                     "precipitation_probability_max",
            "timezone": "auto", "forecast_days": 2,
            "temperature_unit": _units,
            "wind_speed_unit": "mph" if imperial else "kmh",
        }, timeout=_TIMEOUT)
        r.raise_for_status()
        data = r.json()
    except requests.exceptions.RequestException as e:
        return _err(f"the weather service isn't reachable ({type(e).__name__}).")
    cur, day = data.get("current", {}), data.get("daily", {})
    deg = "F" if imperial else "C"

    def daily(i):
        return {"conditions": _WMO.get(day["weather_code"][i], "unknown"),
                "high": f"{round(day['temperature_2m_max'][i])}{deg}",
                "low": f"{round(day['temperature_2m_min'][i])}{deg}",
                "chance_of_rain": f"{day['precipitation_probability_max'][i]}%"}
    try:
        return {
            "place": loc["name"],
            "now": {"conditions": _WMO.get(cur.get("weather_code"), "unknown"),
                    "temperature": f"{round(cur['temperature_2m'])}{deg}",
                    "feels_like": f"{round(cur['apparent_temperature'])}{deg}",
                    "wind": f"{round(cur['wind_speed_10m'])} {'mph' if imperial else 'km/h'}"},
            "today": daily(0),
            "tomorrow": daily(1),
        }
    except (KeyError, IndexError, TypeError):
        return _err("the weather service returned something unexpected.")


_TOOLS = [
    {
        "name": "weather",
        "description": "Current weather and today/tomorrow forecast for a place "
                       "(defaults to home).",
        "parameters": {
            "type": "object",
            "properties": {"place": {"type": "string"}},
            "required": [],
        },
    },
]


def list_tools():
    return list(_TOOLS)


def call_tool(name, arguments):
    if name != "weather":
        return _err(f"unknown tool: {name}")
    return _weather(arguments or {})
