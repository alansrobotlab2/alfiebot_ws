"""
voice — ``learn_voice``: enroll (or forget) the current speaker's voiceprint.

Speaker ID runs in the ASR node (alfie_asr/speaker_id.py): every transcript
carries the recognized speaker, and the agent passes it to the model as
"(Speaking: <name>)". This tool asks the ASR node to learn a voice from the
utterances it heard in the last ~90 s. agent_node wires it via
``configure(enroll=...)``, where ``enroll(name, forget) -> result dict`` calls
the ``asr/enroll_speaker`` service.
"""

_enroll = None


def configure(enroll=None):
    global _enroll
    _enroll = enroll


def _err(msg):
    return {"error": msg}


def _learn_voice(params):
    if _enroll is None:
        return _err("voice recognition isn't available right now.")
    name = " ".join(str(params.get("name") or "").split())
    if not name:
        return _err("I need the person's name.")
    return _enroll(name, bool(params.get("forget"))) or _err("no reply")


_TOOLS = [
    {
        "name": "learn_voice",
        "description": "Learn the voice of the person talking so you recognize "
                       "them later ('learn my voice, I'm Alan'); forget=true "
                       "deletes it. They should have said a few sentences.",
        "parameters": {
            "type": "object",
            "properties": {"name": {"type": "string"},
                           "forget": {"type": "boolean"}},
            "required": ["name"],
        },
    },
]


def list_tools():
    return list(_TOOLS)


def call_tool(name, arguments):
    if name != "learn_voice":
        return _err(f"unknown tool: {name}")
    return _learn_voice(arguments or {})
