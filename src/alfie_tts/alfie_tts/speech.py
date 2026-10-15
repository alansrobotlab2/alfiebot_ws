"""
tts_node — Piper speech out of the reSpeaker, one reply streamed as clauses.

The agent publishes a reply as a run of `speechrequest`s while the model is
still writing it: each carries one clause, its `turn_id`, and
`more_follows=true`; an empty request with `more_follows=false` ends the reply.
(A request with `more_follows=false` and text — the old one-shot form — is a
whole reply on its own.)

Two threads, so a clause is synthesised while the previous one plays and the
voice never has dead air between clauses (lloyd's "clauses play back to back"):

  * the synth thread turns queued requests into audio;
  * the play thread keeps ONE output stream open for the whole reply, writing
    silence while it waits for the next clause or while paused, so `speaking`
    stays high from the first clause to the last instead of flickering between
    them (which re-opened the agent's follow-up window mid-reply).

Control:
  `tts/pause` (Bool)  true pauses at once, false resumes from the same sample —
                      the agent's barge-in "pause, then decide".
  `tts/stop` (Empty)  and `barge_in` (Empty): stop now, drop everything queued,
                      and ignore late clauses from any turn up to the newest one
                      seen (a generation counter, so nothing from before the
                      stop is ever spoken).
Out:
  `speaking` (Speaking), `tts/level` (RMS per ~24 ms window for the LED pulse),
  `tts/spoken` (String) each clause once its audio has been written out, so the
  agent can tell the next turn what the listener actually heard.
"""
import os
import queue
import subprocess
import threading
import time

import numpy as np
import sounddevice as sd
from piper.voice import PiperVoice
from piper.config import SynthesisConfig

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from ament_index_python.packages import get_package_share_directory

from std_msgs.msg import Bool, Empty, Float32, String
from alfie_msgs.msg import SpeechRequest
from alfie_msgs.msg import Speaking

# Substring (case-insensitive) used to locate the reSpeaker's PulseAudio sink.
# The gen2 reSpeaker XVF3800 drives the 5W speaker AND provides the AEC far-end
# reference, so TTS must play through it. Match by name (the sink name carries a
# unit-specific serial and the ALSA card index is not stable across replugs).
RESPEAKER_SINK_MATCH = ('respeaker', 'seeed')

# PortAudio device to play through, in order of preference. We must open the
# sound server explicitly: ALSA's `default` on this Jetson is the Tegra APE
# card, so an unqualified stream bypasses PipeWire entirely and the audio is
# silently discarded regardless of which sink pactl has made default.
OUTPUT_DEVICE_PREFERENCE = ('pipewire', 'pulse')

# Audio is written in small windows with an RMS level published per window on
# `tts/level`, paced to playback by the blocking stream write. ~24 ms at the
# ~21 kHz output rate.
LEVEL_WINDOW_SAMPLES = 512
# Silence written per loop while waiting for the next clause or while paused.
IDLE_WRITE_S = 0.02
# Appended to the end of every reply (not every clause) so the last syllable is
# never the audio sitting in a buffer that gets cut (lloyd's tail_silence_ms).
TAIL_SILENCE_S = 0.25
# A reply whose end marker never arrives (agent crashed mid-turn) is closed
# after this long with nothing to say.
MORE_FOLLOWS_TIMEOUT_S = 30.0
# A pause the agent never resolves (it died mid-decision) resumes on its own.
MAX_PAUSE_S = 15.0


class AlfieTTS(Node):
    def __init__(self):

        super().__init__('alfie_speech')

        # Get package voices directory
        package_share_directory = get_package_share_directory('alfie_tts')
        voices_dir = os.path.join(package_share_directory, 'voices')

        self.model_path = os.path.join(voices_dir, 'en_US-libritts_r-medium.onnx')
        self.config_path = os.path.join(voices_dir, 'en_US-libritts_r-medium.onnx.json')
        self.speaker_id = 623
        #candidates = [65, 66, 67, 76, 515, 617, 623, 903 ]
        self.length_scale = 0.95  # >1.0 slows down speech (1.1 = 10% slower)
        self.noise_scale = 0.667  # controls pitch variation
        self.noise_w = 0.8  # controls phoneme duration variation
        self.playback_rate_scale = 0.95  # <1.0 lowers pitch, >1.0 raises pitch
        self.use_cuda = False
        self.latency = 0.15
        self.speaking = False

        # Generation counter: bumped by a stop; queued work from an older
        # generation is dropped wherever it is found.
        self._gen = 0
        self._dropped_through = 0   # ignore requests with turn_id <= this
        self._max_turn_seen = 0
        self._paused = False
        self._pause_time = 0.0
        self._requests = queue.Queue()
        self._audio = queue.Queue()
        self._stream = None
        self._stream_lock = threading.Lock()
        self._volume = None

        # Resolve and select the reSpeaker sink as the default PulseAudio sink.
        # (PulseAudio resamples our 22.05 kHz synthesis to the reSpeaker's 16 kHz.)
        self.output_sink = self._resolve_respeaker_sink()
        if self.output_sink:
            try:
                subprocess.run(["pactl", "set-default-sink", self.output_sink], check=True)
                self.get_logger().info(f"Set default sink to {self.output_sink}")
            except Exception as e:
                self.get_logger().warn(f"Failed to set default sink: {e}")
        else:
            self.get_logger().error(
                "reSpeaker PulseAudio sink not found; TTS has no output device. "
                "Check the reSpeaker connection.")

        # Which PortAudio device to hand to sounddevice (see
        # OUTPUT_DEVICE_PREFERENCE). Resolved once; the index is stable for the
        # life of the process.
        self.output_device = self._resolve_output_device()

        best_effort = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        reliable = QoSProfile(depth=50, reliability=ReliabilityPolicy.RELIABLE)

        self.speaking_pub = self.create_publisher(Speaking, 'speaking', best_effort)
        # Live speech amplitude (RMS 0..1) for the speaking-state LED pulse.
        self.level_pub = self.create_publisher(Float32, 'tts/level', best_effort)
        self.spoken_pub = self.create_publisher(String, 'tts/spoken', reliable)

        self.voice = PiperVoice.load(self.model_path, self.config_path, use_cuda=self.use_cuda)
        self.out_rate = int(self.voice.config.sample_rate * self.playback_rate_scale)
        self.syn_config = SynthesisConfig(
            speaker_id=self.speaker_id,
            length_scale=self.length_scale,
            noise_scale=self.noise_scale,
            noise_w_scale=self.noise_w,
        )

        # Callbacks only enqueue or flip flags; the blocking work lives on the two
        # threads below. Speech requests get their own mutually exclusive group so
        # they are queued strictly in arrival order (a reentrant group let the
        # end-of-reply marker overtake the clause before it), and control gets
        # another so a stop is never stuck behind a request.
        speech_group = MutuallyExclusiveCallbackGroup()
        control_group = MutuallyExclusiveCallbackGroup()
        self.create_subscription(SpeechRequest, 'speechrequest',
                                 self.speech_request_callback, reliable,
                                 callback_group=speech_group)
        self.create_subscription(Empty, 'barge_in', self.stop_callback, best_effort,
                                 callback_group=control_group)
        self.create_subscription(Empty, 'tts/stop', self.stop_callback, reliable,
                                 callback_group=control_group)
        self.create_subscription(Bool, 'tts/pause', self.pause_callback, reliable,
                                 callback_group=control_group)

        threading.Thread(target=self._synth_loop, daemon=True).start()
        threading.Thread(target=self._play_loop, daemon=True).start()

        # Publish initial state (not speaking)
        self.publish_speaking(False)

    def _resolve_respeaker_sink(self):
        """Return the PulseAudio sink name for the reSpeaker, or None."""
        try:
            out = subprocess.run(
                ["pactl", "list", "sinks", "short"],
                check=True, capture_output=True, text=True).stdout
        except Exception as e:
            self.get_logger().warn(f"Could not list PulseAudio sinks: {e}")
            return None
        for line in out.splitlines():
            name = line.split('\t')[1] if '\t' in line else line
            low = name.lower()
            if any(m in low for m in RESPEAKER_SINK_MATCH):
                return name
        return None

    def _resolve_output_device(self):
        """Return the PortAudio index of the sound server, or None for default."""
        try:
            devices = sd.query_devices()
        except Exception as e:
            self.get_logger().warn(f"Could not query audio devices: {e}")
            return None
        for want in OUTPUT_DEVICE_PREFERENCE:
            for index, dev in enumerate(devices):
                if dev['max_output_channels'] > 0 and dev['name'].lower() == want:
                    self.get_logger().info(f"Using output device '{dev['name']}' (index {index})")
                    return index
        self.get_logger().warn(
            "No PipeWire/PulseAudio output device found; falling back to the "
            "ALSA default, which may not reach the reSpeaker speaker.")
        return None

    def publish_speaking(self, is_speaking):
        msg = Speaking()
        msg.is_speaking = is_speaking
        self.speaking = is_speaking
        self.speaking_pub.publish(msg)

    def publish_level(self, level):
        msg = Float32()
        msg.data = float(level)
        self.level_pub.publish(msg)

    def set_volume(self, volume):
        # A pactl subprocess per utterance was measurable latency; only call it
        # when the requested volume actually changes.
        if not self.output_sink or volume == self._volume:
            return
        try:
            subprocess.run(
                ["pactl", "set-sink-volume", self.output_sink, f"{int(volume)}%"],
                check=True)
            self._volume = volume
        except Exception as e:
            self.get_logger().warn(f"Failed to set sink volume: {e}")

    # --- callbacks (never block) ---

    def speech_request_callback(self, msg):
        if msg.turn_id and msg.turn_id <= self._dropped_through:
            return  # a clause of a turn that was stopped
        self._max_turn_seen = max(self._max_turn_seen, msg.turn_id)
        if msg.text:
            self.get_logger().info(f"Speech request [{msg.turn_id}]: {msg.text}")
        self._requests.put((self._gen, msg.text, msg.turn_id, msg.more_follows, msg.volume))

    def stop_callback(self, msg):
        # User started talking over the robot, or the agent superseded the
        # reply: stop now and forget everything queued.
        self._gen += 1
        self._dropped_through = max(self._dropped_through, self._max_turn_seen)
        self._paused = False
        for q in (self._requests, self._audio):
            try:
                while True:
                    q.get_nowait()
            except queue.Empty:
                pass
        if self.speaking:
            self.get_logger().info("Stop received; interrupting speech.")
        # Abort the stream directly to flush buffered audio immediately.
        with self._stream_lock:
            if self._stream is not None:
                try:
                    self._stream.abort()
                except Exception:
                    pass

    def pause_callback(self, msg):
        if bool(msg.data) != self._paused:
            self._paused = bool(msg.data)
            self._pause_time = time.monotonic()
            self.get_logger().info("Paused." if self._paused else "Resumed.")

    def _is_paused(self):
        if self._paused and time.monotonic() - self._pause_time > MAX_PAUSE_S:
            self.get_logger().warn("Pause never resolved; resuming.")
            self._paused = False
        return self._paused

    # --- synthesis thread ---

    def _synth_loop(self):
        while rclpy.ok():
            gen, text, turn_id, more, volume = self._requests.get()
            if gen != self._gen:
                continue
            audio = None
            if text.strip():
                parts = []
                try:
                    for chunk in self.voice.synthesize(text, self.syn_config):
                        if gen != self._gen:
                            break
                        parts.append(np.frombuffer(chunk.audio_int16_bytes, dtype=np.int16))
                except Exception as e:
                    self.get_logger().error(f"Synthesis failed: {e}")
                if gen != self._gen:
                    continue
                audio = np.concatenate(parts) if parts else None
                if audio is None:
                    continue
            self._audio.put((gen, text, audio, more, volume))

    # --- playback thread ---

    def _open_stream(self):
        stream = sd.RawOutputStream(
            samplerate=self.out_rate, channels=1, dtype='int16',
            latency=self.latency, device=self.output_device)
        stream.start()
        with self._stream_lock:
            self._stream = stream
        return stream

    def _close_stream(self, stream, drain):
        with self._stream_lock:
            self._stream = None
        try:
            if drain:
                stream.write(np.zeros(int(self.out_rate * TAIL_SILENCE_S), np.int16).tobytes())
                stream.stop()   # waits for queued audio to actually play
            else:
                stream.abort()
            stream.close()
        except Exception:
            pass

    def _play_loop(self):
        idle = np.zeros(int(self.out_rate * IDLE_WRITE_S), np.int16).tobytes()
        stream = None
        gen = self._gen
        expecting_more = False
        more_deadline = 0.0
        while rclpy.ok():
            if stream is not None and gen != self._gen:
                # Stopped mid-reply.
                self._close_stream(stream, drain=False)
                stream, expecting_more = None, False
                self.publish_level(0.0)
                self.publish_speaking(False)
            try:
                item = self._audio.get(timeout=IDLE_WRITE_S if stream else 0.2)
            except queue.Empty:
                if stream is None:
                    continue
                waiting = expecting_more and time.monotonic() < more_deadline
                if self._is_paused() or waiting or not self._requests.empty():
                    try:
                        stream.write(idle)  # hold the stream (and `speaking`) open
                    except Exception:
                        pass
                    continue
                # Reply finished: let the tail play out, then drop `speaking`.
                self._close_stream(stream, drain=True)
                stream = None
                self.publish_level(0.0)
                self.publish_speaking(False)
                continue

            item_gen, text, audio, more, volume = item
            if item_gen != self._gen:
                continue
            gen = item_gen
            expecting_more = more
            more_deadline = time.monotonic() + MORE_FOLLOWS_TIMEOUT_S
            if audio is None:
                continue  # end-of-reply marker
            if stream is None:
                self.set_volume(volume)
                try:
                    stream = self._open_stream()
                except Exception as e:
                    self.get_logger().error(f"Could not open output stream: {e}")
                    continue
                self.publish_speaking(True)
            if self._play_clause(stream, audio, gen):
                m = String()
                m.data = text
                self.spoken_pub.publish(m)

    def _play_clause(self, stream, audio, gen):
        """Write one clause; True if it played to the end (not stopped)."""
        idle = np.zeros(int(self.out_rate * IDLE_WRITE_S), np.int16).tobytes()
        i = 0
        while i < len(audio):
            if gen != self._gen:
                return False
            if self._is_paused():
                # Hold position; silence keeps the stream (and AEC reference) live.
                try:
                    stream.write(idle)
                except Exception:
                    return False
                continue
            win = audio[i:i + LEVEL_WINDOW_SAMPLES]
            i += LEVEL_WINDOW_SAMPLES
            rms = float(np.sqrt(np.mean(np.square(win.astype(np.float32))))) / 32768.0
            self.publish_level(rms)
            try:
                stream.write(win.tobytes())
            except Exception as e:
                # A stop's abort() unblocks write() with an error; that's expected.
                if gen == self._gen:
                    self.get_logger().error(f"Playback error: {e}")
                return False
        return gen == self._gen


def main(args=None):
    rclpy.init(args=args)
    alfietts = AlfieTTS()

    executor = MultiThreadedExecutor()
    executor.add_node(alfietts)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        alfietts.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
