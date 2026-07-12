"""
alfie_chat — the voice-to-voice conversation stack only.

Brings up just the speech pipeline (no micro-ROS drivers, cameras, or Foxglove):

  mic capture -> ASR -> agent -> LLM (Qwen3.6 35B-A3B) -> TTS -> reSpeaker
  + DOA / LED conversational feedback

Use this for developing or demoing the conversation loop on its own. The full
robot bring-up (alfie_bringup.py) launches these same nodes alongside everything
else.
"""
from launch import LaunchDescription
from launch_ros.actions import Node


def voice_node(package, executable, name):
    return Node(
        package=package,
        namespace='alfie',
        executable=executable,
        name=name,
        output='screen',
        emulate_tty=True,
        sigterm_timeout='5',
        sigkill_timeout='10',
        respawn=True,
    )


def generate_launch_description():
    return LaunchDescription([
        # Local LLM server (custom mlc_llm build, Qwen3.6 35B-A3B). Takes ~30 s to
        # load; publishes /alfie/llm/ready when the endpoint is up.
        voice_node('alfie_llm', 'mlc_llm_serve_node', 'mlc_llm_node'),

        # Audio capture + reSpeaker USB control (DOA / LEDs) + LED behaviour.
        voice_node('alfie_mic', 'audio_publisher', 'audio_publisher_node'),
        voice_node('alfie_mic', 'respeaker_control', 'respeaker_control_node'),
        voice_node('alfie_mic', 'led_behavior', 'led_behavior_node'),

        # ASR (Parakeet + Silero VAD) -> asrresult.
        voice_node('alfie_asr', 'parakeet_asr_node', 'asr_node'),

        # Wake word (openWakeWord) -> wake (opens conversation window) + barge_in.
        voice_node('alfie_wakeword', 'wakeword_node', 'wakeword_node'),

        # TTS (Piper -> reSpeaker) + speaking/level for the LED pulse.
        voice_node('alfie_tts', 'alfietts', 'tts_node'),

        # Conversation bridge: asrresult -> LLM -> speechrequest.
        voice_node('alfie_agent', 'agent_node', 'agent_node'),
    ])
