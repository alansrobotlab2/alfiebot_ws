"""
agent_node — the conversation bridge with a tool-calling brain.

Closes the loop between ASR and TTS: it subscribes to transcripts (`asrresult`),
runs an agentic turn against the local MLC-LLM server (Qwen3.6 35B-A3B) — which
may call tools such as reading/writing notes in the obsidian vault — and
publishes the spoken reply as `speechrequest`s for TTS, clause by clause while
the model is still writing. It also publishes a `generating` flag (true while a
turn is in flight) that drives the "thinking" LED.

The thinking is delegated to `harness.run_turn`: MLC can't do native OpenAI
tool-calling, so tools are described in the system prompt and Qwen emits
`<tool_call>` blocks that the harness parses and dispatches (see harness.py,
prompt_builder.py, tools/). This node keeps all ROS I/O and turn management.

The voice behaviour is ported from lloyd's 2026-09/10 voice rework
(architecture/voice.md there):

  * **Streaming reply.** `speakable.ClauseStream` cuts the streamed text into
    clauses (the first may be cut at a comma once 20 chars long) and each goes
    to TTS as soon as it is complete, tagged with the turn id and
    `more_follows=true`; an empty request with `more_follows=false` ends the
    reply. Text the model writes before a tool call is said before it runs.
  * **Fillers.** If nothing has been said 2.5 s into a turn, or a tool is about
    to run first, a short contextual line ("Let me take a look.") covers the
    silence; a long tool phase gets a progress line every 8 s.
  * **Wake gating, two paths.** The acoustic `wake` event opens a listening
    window, and so does a transcript that *opens* with a wake phrase. A woken
    utterance that only mentions Alfie mid-sentence ("did Alfie...") is dropped.
    The window stays open while a turn runs or Alfie speaks, and reopens after
    each reply: `followup_window_s`, or `question_window_s` when the reply ended
    with a question.
  * **Conversation words.** Backchannels ("yeah", "okay", "great") are not
    turns unless Alfie just asked a question; closers ("that's all", "thanks
    Alfie") end the conversation; stoppers stop him.
  * **Barge-in: pause, then decide.** Sustained speech (`vad/speech` for
    `barge_min_speech_s`, after `barge_warmup_s` of the reply) or the
    wakeword node's energy gate (`barge_in/soft`) *pauses* TTS. The transcript
    that follows decides: words -> stop and answer them; empty, a backchannel,
    or Alfie's own echo -> resume where he paused; no words within
    `false_interruption_s` of the speaker going quiet -> resume. Hard
    `barge_in` (the "stop" word, or the wake word over speech) stops at once.
  * **Interruption note.** The next turn is told what the listener actually
    heard (from `tts/spoken`) and not to resume or repeat the interrupted reply.
  * **Latency.** Every turn logs one `[latency]` line (timeline.py): end of
    speech -> transcript -> turn -> first token -> first clause -> first audio.
  * Turns are invalidated by a monotonically increasing turn id: a new request,
    a stopper or a barge-in bumps the id, and any in-flight worker notices
    (before each LLM call and tool call, and on every streamed chunk) and bails.
"""
import threading
import time
from datetime import datetime, timezone

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

from std_msgs.msg import Bool, Empty, String
from alfie_msgs.msg import ASRResult, SpeechRequest, Speaking
from alfie_msgs.srv import Detect, EnrollSpeaker
from std_srvs.srv import Trigger

from alfie_agent import conversation as convo
from alfie_agent import harness, memory, prompt_builder, tools
from alfie_agent.body import Body
from alfie_agent.prefetch import Prefetcher
from alfie_agent.llm_client import LLMClient
from alfie_agent.speakable import ClauseStream
from alfie_agent.tools import lloyd as lloyd_tool
from alfie_agent.timeline import TurnTimeline

SPEAK_VOLUME = 100
# A transcript arriving within this long of a wake may be the wake phrase
# itself, mis-heard: if it did not match a wake phrase and is this short, it
# is acknowledged rather than answered.
WAKE_UTTERANCE_SUPPRESS_S = 3.0
MAX_WAKE_UTTERANCE_WORDS = 2
# Own-voice echo: transcripts arriving while Alfie speaks, or this long after,
# are compared against the reply being spoken.
ECHO_MEMORY_S = 2.0
TICK_S = 0.1
# _heard_speaker value for "speaker ID ran and this voice isn't enrolled".
UNKNOWN_VOICE = '?'


class AgentNode(Node):
    def __init__(self):
        super().__init__('agent_node')

        # --- parameters (workspace idiom: declare_parameter with defaults) ---
        self.llm_base_url = self.declare_parameter(
            'llm_base_url', 'http://localhost:8000/v1').value
        model_id_fallback = self.declare_parameter(
            'model_id', 'dist/qwen3_6-35B-A3B-q4f16_1_fused').value
        # Shared-with-lloyd obsidian vault; point this at your placeholder folder
        # until the real vault is hooked up.
        self.vault_root = self.declare_parameter('vault_root', '~/obsidian').value
        # QMD semantic-search daemon (tobi/qmd). If it's not running, vault_search
        # returns a clean error and the rest of the agent still works.
        qmd_url = self.declare_parameter(
            'qmd_url', 'http://localhost:8181/query').value
        # Skip the cross-encoder reranker by default: qmd runs on CPU here, and
        # the reranker (0.6B) roughly doubles query latency for a small quality
        # gain. lex+vec results are already strong. (lloyd defaults this too.)
        qmd_skip_rerank = bool(self.declare_parameter('qmd_skip_rerank', True).value)
        # alfie_room room-recognition service (HTTP). If it's not running,
        # identify_room/learn_room return a clean error and the agent still works.
        room_url = self.declare_parameter(
            'room_url', 'http://localhost:8182/').value
        # alfie_nanoowl on-demand detection service (ROS). If it's not running,
        # the `look` tool returns a clean error and the agent still works.
        nanoowl_service = self.declare_parameter(
            'nanoowl_service', 'nanoowl/detect').value
        # Budget for a single on-demand detection (first call re-encodes the
        # prompt and may warm the model, so keep it generous).
        self._nanoowl_timeout = float(
            self.declare_parameter('nanoowl_timeout', 10.0).value)
        # clear_estop refuses until the latch has been held this long, so Alfie
        # can't clear it on hearing its own "E-stop engaged" announcement, and
        # someone has had a moment to look at the robot first.
        self._estop_min_hold = float(
            self.declare_parameter('estop_reset_min_hold_sec', 5.0).value)
        self.max_tool_iters = int(self.declare_parameter('max_tool_iters', 4).value)
        # Kept declared for launch-file compatibility; superseded by the
        # token-budget trim below (active depth is now bounded by tokens, not a
        # fixed pair count).
        self.declare_parameter('max_history_turns', 6)
        max_tokens = int(self.declare_parameter('max_tokens', 200).value)
        temperature = float(self.declare_parameter('temperature', 0.7).value)

        # --- conversation + barge-in (lloyd's measured defaults) ---
        self.followup_window_s = float(
            self.declare_parameter('followup_window_s', 6.0).value)
        self.question_window_s = float(
            self.declare_parameter('question_window_s', 20.0).value)
        self.text_wake = bool(self.declare_parameter('text_wake', True).value)
        # 'pause' (pause, then decide on the transcript) or 'off'.
        self.barge_mode = self.declare_parameter('barge_mode', 'pause').value
        self.barge_min_speech_s = float(
            self.declare_parameter('barge_min_speech_s', 0.5).value)
        self.barge_warmup_s = float(
            self.declare_parameter('barge_warmup_s', 2.0).value)
        self.false_interruption_s = float(
            self.declare_parameter('false_interruption_s', 2.0).value)
        self.max_pause_s = float(self.declare_parameter('max_pause_s', 10.0).value)
        self.echo_overlap = float(self.declare_parameter('echo_overlap', 0.6).value)
        self.filler_after_s = float(
            self.declare_parameter('filler_after_s', 2.5).value)
        self.progress_every_s = float(
            self.declare_parameter('progress_every_s', 8.0).value)

        # --- context management (stateless episodes + recent-memory injection) ---
        # Active depth is bounded within a live episode by a token budget; on idle
        # the episode is compacted into the memory subsystem and reset. A fresh
        # wake injects a window of recent-interaction summaries.
        self.active_token_budget = int(
            self.declare_parameter('active_token_budget', 2000).value)
        self.recent_window_min = int(
            self.declare_parameter('recent_window_min', 30).value)
        self.recent_max_items = int(
            self.declare_parameter('recent_max_items', 5).value)
        self.compaction_enabled = bool(
            self.declare_parameter('compaction_enabled', True).value)
        # Body tools (body.py / tools/robot.py): agent head motion on the mux's
        # `agent` head source, and the GR00T task run cap.
        head_enabled = bool(self.declare_parameter('head_enabled', True).value)
        task_max_s = float(self.declare_parameter('task_max_s', 120.0).value)
        # Outside services: weather (Open-Meteo) and lloyd's agent API (over the
        # tailnet; '' disables ask_lloyd).
        weather_location = self.declare_parameter('weather_location', '').value
        weather_units = self.declare_parameter('weather_units', 'fahrenheit').value
        lloyd_url = self.declare_parameter('lloyd_url', '').value
        lloyd_timeout = float(self.declare_parameter('lloyd_timeout', 90.0).value)
        # Curated memory (alfie/MEMORY.md): byte cap, injected per episode.
        memory_max_bytes = int(
            self.declare_parameter('memory_max_bytes', 1500).value)
        # Per-turn context prefetch (prefetch.py): a small <context> block
        # retrieved under a hard budget and prepended to the user message.
        self.prefetch_enabled = bool(
            self.declare_parameter('prefetch_enabled', True).value)
        self.prefetcher = Prefetcher(
            budget_ms=int(self.declare_parameter('prefetch_budget_ms', 150).value),
            max_chars=int(self.declare_parameter('prefetch_max_chars', 1200).value),
            recent_window_min=self.recent_window_min,
            # Whose facts a first-person question ("what's my favorite...")
            # loads; '' = only the generic `User` entity.
            primary_user=self.declare_parameter('primary_user', '').value)

        # On-demand vision: a service client for the nanoowl detector. Created
        # before tools.configure so the `look` tool can be wired to it.
        self.nanoowl_cli = self.create_client(Detect, nanoowl_service)

        # E-stop: track the command_mux latch (latched topic) and a reset client
        # for the `clear_estop` tool.
        self._estopped = None          # None until the first estop_state arrives
        self._estop_since = 0.0        # monotonic time the latch last engaged
        self.create_subscription(
            Bool, 'estop_state', self._on_estop_state,
            QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                       durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.estop_reset_cli = self.create_client(Trigger, 'estop_reset')
        # Voice enrollment on the ASR node, for the `learn_voice` tool.
        self.enroll_cli = self.create_client(EnrollSpeaker, 'asr/enroll_speaker')

        # Robot-facing ROS I/O for the body tools; announcements (timers, task
        # timeouts) are spoken from _on_tick once Alfie is idle.
        self._announcements = []
        self.body = Body(self, announce=self._announce,
                         estopped=lambda: self._estopped,
                         task_max_s=task_max_s, head_enabled=head_enabled)

        # --- brain: tools, system prompt, LLM client ---
        tools.configure(self.vault_root, qmd_url=qmd_url,
                        qmd_skip_rerank=qmd_skip_rerank, room_url=room_url,
                        see_detect=self._nanoowl_detect,
                        estop_reset=self._estop_reset,
                        memory_max_bytes=memory_max_bytes,
                        body=self.body,
                        weather_location=weather_location,
                        weather_units=weather_units,
                        lloyd_url=lloyd_url, lloyd_timeout=lloyd_timeout,
                        enroll_voice=self._enroll_voice)
        # Warm the QMD daemon off the init path: the first vault search is ~1.2 s
        # (cold embedding-model load) vs ~75 ms warm, so pay that now on a
        # background thread instead of on the user's first turn.
        threading.Thread(target=tools.warmup_vault_search, daemon=True).start()
        # Keep the soul + tool specs so the system prompt can be rebuilt per
        # episode with the recent-memory window folded in (see _open_conversation).
        self._tool_specs = tools.list_tools()
        self._soul = prompt_builder.load_soul(self.vault_root)
        self.system_prompt = prompt_builder.build_system_prompt(
            self._soul, self._tool_specs)
        self.llm = LLMClient(self.llm_base_url, model_id_fallback,
                             timeout=(5, 60), temperature=temperature,
                             max_tokens=max_tokens)

        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        # Clauses must not be dropped mid-reply: RELIABLE with a deep queue.
        speech_qos = QoSProfile(depth=50, reliability=ReliabilityPolicy.RELIABLE)

        self.speech_pub = self.create_publisher(SpeechRequest, 'speechrequest', speech_qos)
        self.generating_pub = self.create_publisher(Bool, 'generating', qos)
        # TTS control: pause/resume (true/false) and stop. A separate stop topic
        # so the agent never hears its own stop as a `barge_in`.
        self.tts_pause_pub = self.create_publisher(Bool, 'tts/pause', speech_qos)
        self.tts_stop_pub = self.create_publisher(Empty, 'tts/stop', speech_qos)

        self.create_subscription(ASRResult, 'asrresult', self.on_asrresult, qos)
        self.create_subscription(Empty, 'barge_in', self.on_barge_in, qos)
        self.create_subscription(Empty, 'barge_in/soft', self.on_soft_barge, qos)
        self.create_subscription(Speaking, 'speaking', self.on_speaking, qos)
        self.create_subscription(Bool, 'vad/speech', self.on_vad, qos)
        self.create_subscription(String, 'tts/spoken', self.on_tts_spoken, speech_qos)

        # Wake events open the conversation window. RELIABLE to match wakeword_node.
        wake_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        self.create_subscription(String, 'wake', self.on_wake, wake_qos)

        # The LLM server (35B MoE) takes ~30 s to load. Gate on its latched
        # `llm/ready` so transcripts heard during startup are dropped instead of
        # hitting a connection-refused. Latched QoS so we still get the retained
        # value if the agent starts after the server is already up.
        ready_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                               durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(Bool, 'llm/ready', self.on_llm_ready, ready_qos)
        self._llm_ready = False

        self._lock = threading.Lock()
        # Seeded from wall-clock seconds so turn ids keep increasing across agent
        # restarts: TTS drops every request at or below the last turn it
        # interrupted, so a counter restarting at 0 would be silenced until it
        # caught up. (uint32 on the wire: good until 2106.)
        self._turn = int(time.time())
        # Active context for the *live* episode only (reset on idle compaction).
        self._episode_turns = []    # list of {"role", "content"}
        self._episode_speakers = set()  # voices recognized this episode
        self._heard_speaker = ''    # speaker of the transcript being handled
        # System prompt for the live episode: the base prompt, or (on a fresh
        # wake) the base prompt with a recent-memory window folded in. Rebuilt at
        # episode start so the window rides inside the position-0 system message.
        self._episode_system_prompt = self.system_prompt
        self._compacting = False    # a background compaction is in flight
        self._generating = False    # a turn is generating (mirrors the LED flag)

        # The turn being generated and/or spoken (0 = none).
        self._active_turn = 0
        self._active_text = ''      # what the user asked in the active turn
        self._turn_running = False  # the active turn's worker is still generating
        self._timeline = None       # TurnTimeline of the active turn
        self._timeline_logged = False
        self._said_any = False      # the active turn has published any clause
        self._last_said = 0.0       # monotonic time of the last clause published
        self._in_tools = False      # the active turn is running a tool
        self._fillers = 0           # fillers/progress lines spoken this turn
        self._heard = []            # clauses of the active turn TTS finished playing
        self._interrupt_note = None  # told to the next turn after an interruption
        self._last_reply = ''
        self._reply_said = []       # clauses of the current reply, for echo rejection

        # Wake-gating state.
        self._listen_until = 0.0    # monotonic deadline; window open while now < it
        self._wake_time = 0.0       # monotonic time of the last wake event
        self._wake_fresh = False    # the last wake started a new conversation
        self._closed = False        # a closer ended the conversation
        self._speaking = False
        self._speak_start = 0.0
        self._speak_end = 0.0

        # Barge-in state.
        self._vad_speech = False
        self._vad_change = 0.0      # monotonic time of the last vad edge
        self._paused = False
        self._pause_time = 0.0

        # Idle detector: when the follow-up window lapses with un-compacted turns,
        # summarize the episode into memory (GPU is free then) and reset context.
        self.create_timer(2.0, self._on_idle_tick)
        # Barge-in decisions, fillers and progress lines.
        self.create_timer(TICK_S, self._on_tick)

        self.publish_generating(False)
        self.get_logger().info(
            f'AgentNode initialized with {len(self._tool_specs)} tool(s), '
            f'vault={self.vault_root} (waiting for llm/ready; say the wake word to talk).')

    # --- ROS callbacks (kept short; real work happens on worker threads) ---

    def on_llm_ready(self, msg):
        was_ready = self._llm_ready
        self._llm_ready = bool(msg.data)
        if self._llm_ready and not was_ready:
            self.get_logger().info('LLM is ready; waiting for wake word.')
            # Pin the constant base system prompt once. It stays resident for the
            # life of the server, so EVERY first turn forks from it (~0.31s) instead
            # of a cold prefill (~2.3s) — no per-turn re-warm, survives episode and
            # summarize() prompt changes. Fires on each ready rising-edge so a server
            # restart (fresh cache) re-pins. Best-effort, off-thread.
            threading.Thread(target=self._pin_base_prefix, daemon=True).start()
        elif not self._llm_ready and was_ready:
            self.get_logger().warn('LLM went not-ready; pausing.')

    def on_wake(self, msg):
        phrase = (msg.data or '').strip().lower()
        self._open_conversation(f"wake '{phrase}'")

    def _open_conversation(self, why):
        """Open (or extend) the listening window; a closed window starts a new
        episode with the recent-memory window folded into its system prompt."""
        now = time.monotonic()
        fresh = not self._window_open(now)
        self._listen_until = max(self._listen_until, now + self.followup_window_s)
        self._wake_time = now
        self._wake_fresh = fresh
        self._closed = False
        if fresh:
            preamble = profile = None
            try:
                preamble = memory.recent_preamble(
                    window_min=self.recent_window_min,
                    max_items=self.recent_max_items)
                profile = memory.profile_text()
            except Exception as e:  # memory is best-effort; never block a wake
                self.get_logger().warn(f'recent memory unavailable: {e}')
            episode_prompt = prompt_builder.build_system_prompt(
                self._soul, self._tool_specs, recent_memory=preamble,
                profile=profile)
            with self._lock:
                self._episode_turns = []
                self._episode_system_prompt = episode_prompt
            self.prefetcher.reset()
            lloyd_tool.reset_session()
            loaded = [n for n, v in (('memory', profile), ('recent', preamble)) if v]
            self.get_logger().info(
                f"{why}: new conversation, listening"
                f"{' (' + ' + '.join(loaded) + ' loaded)' if loaded else ''}.")
        else:
            self.get_logger().info(f'{why}: window extended.')
        return fresh

    def _window_open(self, now=None):
        now = time.monotonic() if now is None else now
        return now < self._listen_until or self._turn_running or self._speaking

    def on_speaking(self, msg):
        was_speaking = self._speaking
        self._speaking = msg.is_speaking
        now = time.monotonic()
        if msg.is_speaking and not was_speaking:
            self._speak_start = now
            if self._timeline is not None:
                self._timeline.mark('first_audio')
                self._maybe_log_timeline()
        # When Alfie finishes a reply, reopen the follow-up window so the user gets
        # the full gap to respond hands-free — measured from the end of the
        # reply, and held longer when the reply asked something (lloyd's
        # question window). Gated on an active episode so a stray TTS with no
        # live conversation can't hold the window open on background audio.
        if was_speaking and not msg.is_speaking:
            self._speak_end = now
            self._paused = False
            with self._lock:
                active = bool(self._episode_turns)
            if active and not self._closed:
                window = (self.question_window_s
                          if convo.ends_with_question(self._last_reply)
                          else self.followup_window_s)
                self._listen_until = now + window
                self.get_logger().info(f'Reply done; follow-up window open ({window:.0f}s).')
            if self._timeline is not None and 'first_audio' in self._timeline.marks:
                self._maybe_log_timeline(final=True)

    def on_vad(self, msg):
        if bool(msg.data) != self._vad_speech:
            self._vad_speech = bool(msg.data)
            self._vad_change = time.monotonic()

    def on_tts_spoken(self, msg):
        if msg.data:
            self._heard.append(msg.data)

    def on_soft_barge(self, msg):
        # The wakeword node's energy gate: a loud mic while Alfie speaks. A
        # candidate, not a verdict — pause and let the transcript decide.
        if self._barge_allowed(time.monotonic()):
            self._pause('energy')

    def on_barge_in(self, msg):
        # Hard barge-in: the "stop" word, or the wake word said over speech.
        # TTS has already stopped itself; cancel the turn and keep a note of
        # what was heard for the next one.
        self._paused = False
        self._cancel_active('barge-in')
        stopped = self.body.stop_all()
        if stopped:
            self.get_logger().warn(f'Barge-in stopped motion: {", ".join(stopped)}')

    def on_asrresult(self, msg):
        text = (msg.asrresult or '').strip()
        # A recognized name; UNKNOWN_VOICE when speaker ID ran on enough audio
        # and matched nobody; '' when it couldn't tell (short, or ID off).
        self._heard_speaker = msg.speaker or (
            UNKNOWN_VOICE if msg.speaker_score > 0.0 else '')
        tl = TurnTimeline(msg.speech_end or None)
        tl.mark('asr_done')
        if msg.asr_seconds:
            tl.note(f'asr {msg.asr_seconds * 1000:.0f}ms/{msg.duration:.1f}s '
                    f'({msg.reason}, p={msg.turn_prob:.2f})')
        now = time.monotonic()

        # 0. Hard-wired stop: while the body is moving (a GR00T task, a head
        # motion), a stop word halts it at once — no wake word, no LLM.
        if convo.is_motion_stop(text) and \
                (self.body.task_running() or self.body.head_moving()):
            stopped = self.body.stop_all()
            self.get_logger().warn(f'Motion stop ({", ".join(stopped)}): {text!r}')
            self.tts_stop_pub.publish(Empty())
            self._paused = False
            self._cancel_active('motion stop')
            self._announce('Stopped.')
            return

        # 1. A paused reply is waiting on exactly this decision.
        if self._paused:
            self._decide_pause(text, tl)
            return
        if not text:
            return
        if not self._llm_ready:
            self.get_logger().warn('LLM not ready; ignoring transcript.',
                                   throttle_duration_sec=5.0)
            return

        # 2. Alfie's own voice leaking past the AEC.
        if self._is_echo(text, now):
            self.get_logger().info(f'Echo of own voice dropped: {text}')
            return

        # 3. Wake gating: the acoustic window, or a transcript that opens with a
        #    wake phrase.
        after_wake = convo.split_wake(text)
        if not self._window_open(now):
            if not (self.text_wake and after_wake is not None):
                self.get_logger().info(f'No wake word; ignoring: {text}',
                                       throttle_duration_sec=5.0)
                return
            self._open_conversation('transcript wake')
            if not after_wake:
                self.get_logger().info('Wake acknowledged; listening for a command.')
                return
        elif self._wake_fresh and convo.mentions_name_midsentence(text) and \
                self._wake_during(msg, now):
            # The acoustic wake fired on a *mention* ("did Alfie finish?").
            self.get_logger().info(f'Mention, not an address; ignoring: {text}')
            with self._lock:
                if not self._episode_turns:
                    self._listen_until = 0.0
            return

        command = after_wake if after_wake is not None else text
        if not command:
            # Bare wake phrase ("Hey Alfie") — acknowledge and keep listening.
            self._listen_until = max(self._listen_until, now + self.followup_window_s)
            self.get_logger().info('Wake acknowledged; listening for a command.')
            return
        if after_wake is None and (now - self._wake_time) < WAKE_UTTERANCE_SUPPRESS_S \
                and self._wake_fresh and len(command.split()) <= MAX_WAKE_UTTERANCE_WORDS:
            # A short transcript right after a wake that did not match a wake
            # phrase = the (mis-heard) wake phrase itself. Keep listening.
            self._listen_until = now + self.followup_window_s
            self.get_logger().info(f'Ignoring wake utterance: {command}')
            return

        # 4. Conversation words.
        busy = self._turn_running or self._speaking
        if convo.is_closer(command) or (busy and convo.is_stopper(command)):
            self.get_logger().info(f'Closer: {command}')
            if busy:
                self.tts_stop_pub.publish(Empty())
                self._cancel_active('closer')
            if convo.is_closer(command):
                self._close_conversation()
            return
        if convo.is_backchannel(command, after_question=self._last_asked(busy)):
            self.get_logger().info(f'Backchannel, not a turn: {command}')
            if not busy:
                self._listen_until = max(self._listen_until, now + self.followup_window_s)
            return

        # 5. A new request supersedes the running turn, speaking or silent.
        if busy:
            self.get_logger().info('New request interrupts the running turn.')
            self.tts_stop_pub.publish(Empty())
            self._cancel_active('new request')
        self._start_turn(command, tl)

    def _close_conversation(self):
        """A closer: shut the window and keep it shut when the reply's audio
        stops (the stop's `speaking` edge must not reopen it)."""
        self._listen_until = 0.0
        self._closed = True

    # --- barge-in ---

    def _barge_allowed(self, now):
        return (self.barge_mode == 'pause' and self._speaking and not self._paused
                and now - self._speak_start >= self.barge_warmup_s)

    def _pause(self, why):
        self._paused = True
        self._pause_time = time.monotonic()
        m = Bool()
        m.data = True
        self.tts_pause_pub.publish(m)
        self.get_logger().info(f'Barge-in candidate ({why}): paused, deciding.')

    def _resume(self, why):
        if not self._paused:
            return
        self._paused = False
        m = Bool()
        m.data = False
        self.tts_pause_pub.publish(m)
        self.get_logger().info(f'False interruption ({why}): resuming.')

    def _decide_pause(self, text, tl):
        now = time.monotonic()
        if not text:
            return self._resume('empty transcript')
        if self._is_echo(text, now):
            return self._resume(f'own echo: {text!r}')
        command = convo.split_wake(text)
        command = text if command is None else command
        if not command:
            return self._resume('bare wake')
        if convo.is_backchannel(command, after_question=self._last_asked(True)):
            return self._resume(f'backchannel: {command!r}')
        self._paused = False
        self.tts_stop_pub.publish(Empty())
        if convo.is_closer(command) or convo.is_stopper(command):
            self.get_logger().info(f'Barge-in stop: {command}')
            self._cancel_active('stopped')
            if convo.is_closer(command):
                self._close_conversation()
            return
        self.get_logger().info(f'Barge-in committed: {command}')
        self._cancel_active('barge-in')
        self._start_turn(command, tl)

    def _is_echo(self, text, now):
        # Only the reply being spoken (or just finished) can leak back in.
        if not (self._speaking or now - self._speak_end < ECHO_MEMORY_S):
            return False
        return convo.is_echo(text, ' '.join(self._reply_said), self.echo_overlap)

    def _last_asked(self, busy):
        # While he is still talking, the "last sentence" is the reply under way.
        said = ' '.join(self._heard) if busy and self._heard else self._last_reply
        return convo.ends_with_question(said)

    def _wake_during(self, msg, now):
        """Did the last acoustic wake fire during this utterance?"""
        dur = float(msg.duration or 0.0)
        return now - self._wake_time <= dur + 2.0

    # --- ticks ---

    def _on_tick(self):
        now = time.monotonic()
        # Sustained speech over Alfie -> pause and decide.
        if self._vad_speech and self._barge_allowed(now) and \
                now - self._vad_change >= self.barge_min_speech_s:
            self._pause('sustained speech')
        # No words came: resume. (The transcript normally decides first.)
        if self._paused:
            quiet_for = now - self._vad_change if not self._vad_speech else 0.0
            if quiet_for >= self.false_interruption_s:
                self._resume('no words')
            elif now - self._pause_time >= self.max_pause_s:
                self._resume('pause cap')
        # Announcements (timers, task timeouts) wait for Alfie to be idle.
        if self._announcements and not (self._turn_running or self._speaking
                                        or self._paused):
            self._speak_announcement(self._announcements.pop(0))
        # Fillers: cover silence, never an answer.
        if self._turn_running and not self._paused:
            started = self._timeline.marks.get('turn_start') if self._timeline else None
            if not self._said_any and self._fillers == 0 and started and \
                    time.time() - started >= self.filler_after_s:
                self._say(convo.GENERIC_FILLER, self._active_turn, filler=True)
            elif self._in_tools and self._said_any and self._fillers < 3 and \
                    now - self._last_said >= self.progress_every_s:
                self._say(convo.PROGRESS_LINE, self._active_turn, filler=True)

    # --- turns ---

    def _start_turn(self, command, tl):
        now = time.monotonic()
        self._listen_until = max(self._listen_until, now + self.followup_window_s)
        self.get_logger().info(f'Heard: {command}')
        with self._lock:
            self._turn += 1
            my_turn = self._turn
            note, self._interrupt_note = self._interrupt_note, None
        self._active_turn = my_turn
        self._active_text = command
        self._turn_running = True
        self._said_any = False
        self._in_tools = False
        self._fillers = 0
        self._heard = []
        self._reply_said = []
        self._timeline = tl
        self._timeline_logged = False
        tl.mark('turn_start')
        threading.Thread(target=self._run_turn,
                         args=(command, my_turn, note, tl, self._heard_speaker),
                         daemon=True).start()

    def _cancel_active(self, why):
        """Invalidate the active turn; remember what the listener heard."""
        with self._lock:
            self._turn += 1
        turn_id, text = self._active_turn, self._active_text
        if not turn_id:
            self.publish_generating(False)
            return
        heard = ' '.join(self._heard).strip()
        if self._said_any or self._turn_running:
            if heard:
                self._interrupt_note = (
                    'The listener interrupted your previous spoken reply. They heard '
                    f'only this much of it: "{heard}". Answer what they just said; do '
                    'not resume, repeat or finish the interrupted reply unless asked.')
            else:
                self._interrupt_note = (
                    'The listener interrupted your previous reply before hearing any '
                    'of it. Answer what they just said; do not answer the earlier '
                    'request unless asked.')
            # Record only what was actually spoken. No "[interrupted]" marker:
            # the model imitates its own past turns and starts *saying* the
            # marker; the interrupt note above already explains the cut-off.
            # Nothing heard -> no pair (turns must stay user/assistant pairs).
            if heard:
                with self._lock:
                    self._episode_turns.append({"role": "user", "content": text})
                    self._episode_turns.append(
                        {"role": "assistant", "content": heard + '—'})
        self.get_logger().info(f'Turn {turn_id} cancelled ({why}); heard: {heard!r}')
        self._active_turn = 0
        self._turn_running = False
        self._timeline = None
        self.publish_generating(False)

    def _say(self, clause, turn_id, filler=False):
        """Publish one clause of ``turn_id``'s reply to TTS."""
        if not clause or not self._is_current(turn_id):
            return
        if self._timeline is not None and self._active_turn == turn_id:
            self._timeline.mark('first_clause')
            if filler:
                start = self._timeline.marks.get('turn_start', time.time())
                self._timeline.note(f'filler {clause!r} at {time.time() - start:.2f}s')
        out = SpeechRequest()
        out.text = clause
        out.volume = SPEAK_VOLUME
        out.turn_id = turn_id
        out.more_follows = True
        self.speech_pub.publish(out)
        now = time.monotonic()
        self._said_any = True
        self._last_said = now
        if filler:
            self._fillers += 1
        self._reply_said.append(clause)

    def _announce(self, text):
        """Queue something to say unprompted (any thread); spoken from _on_tick."""
        self._announcements.append(text)

    def _speak_announcement(self, text):
        with self._lock:
            self._turn += 1
            turn_id = self._turn
        self.get_logger().info(f'Announce: {text}')
        out = SpeechRequest()
        out.text = text
        out.volume = SPEAK_VOLUME
        out.turn_id = turn_id
        out.more_follows = True
        self.speech_pub.publish(out)
        self._end_reply(turn_id)
        self._last_reply = text
        # Leave a follow-up window open, as after a reply.
        self._listen_until = max(self._listen_until,
                                 time.monotonic() + self.followup_window_s)

    def _end_reply(self, turn_id):
        out = SpeechRequest()
        out.text = ''
        out.volume = SPEAK_VOLUME
        out.turn_id = turn_id
        out.more_follows = False
        self.speech_pub.publish(out)

    def _run_turn(self, text, my_turn, note, tl, speaker=''):
        self.publish_generating(True)
        with self._lock:
            history = list(self._episode_turns)
            system_prompt = self._episode_system_prompt
        # The interruption note and the prefetched <context> block ride in the
        # model's copy of the user message only (the prompt's tail, so the
        # cached prefix is untouched); history keeps exactly what was said.
        context = ''
        if self.prefetch_enabled:
            try:
                context, st = self.prefetcher.build(text, speaker=speaker)
                if not st.get('skipped'):
                    late = f", late={','.join(st['late'])}" if st['late'] else ''
                    self.get_logger().info(
                        f"prefetch {st['ms']}ms: facts={st['facts']} "
                        f"vault={st['vault']} earlier={st['earlier']} "
                        f"carried={st['carried']}{late} -> {st['chars']} chars")
            except Exception as e:  # prefetch is best-effort; never block a turn
                self.get_logger().warn(f'prefetch failed: {e}')
        tl.mark('prefetch')
        model_text = f'({note})\n\n{text}' if note else text
        if speaker == UNKNOWN_VOICE:
            model_text = ("(Voice not recognized: don't assume who is speaking.)\n"
                          + model_text)
        elif speaker:
            # A recognized voice (alfie_asr speaker ID).
            model_text = f'(Speaking: {speaker})\n{model_text}'
            with self._lock:
                self._episode_speakers.add(speaker)
        if context:
            model_text = f'{context}\n\n{model_text}'
        clauses = ClauseStream()
        spoken = []

        def say_all(cs):
            for c in cs:
                spoken.append(c)
                self._say(c, my_turn)

        def on_text(t):
            self._in_tools = False
            say_all(clauses.feed(t))

        def on_tool(name):
            # Whatever the model said before the call is heard before it runs.
            say_all(clauses.flush())
            if not self._said_any:
                self._say(convo.TOOL_FILLERS.get(name, convo.GENERIC_FILLER),
                          my_turn, filler=True)
            self._in_tools = True

        reply = None
        try:
            reply = harness.run_turn(
                model_text, history,
                system_prompt=system_prompt,
                llm=self.llm,
                call_tool=tools.call_tool,
                is_current=lambda: self._is_current(my_turn),
                logger=self.get_logger().info,
                max_tool_iters=self.max_tool_iters,
                on_text=on_text,
                on_tool=on_tool,
                on_first_delta=lambda: tl.mark('first_delta'),
                taint_tools=tools.TAINTING_TOOLS,
                motion_tools=tools.MOTION_TOOLS,
            )
            if reply is not None:
                say_all(clauses.flush())
        except Exception as e:
            self.get_logger().error(f'Turn failed: {e}')
        finally:
            # Always close the reply so TTS drops `speaking` after the last clause.
            self._end_reply(my_turn)

        if not self._is_current(my_turn):
            return  # superseded by a newer turn or a barge-in

        tl.mark('done')
        self._turn_running = False
        self._in_tools = False
        self.publish_generating(False)
        said = ' '.join(spoken).strip() or (reply or '').strip()
        if said:
            self._last_reply = said
            with self._lock:
                self._episode_turns.append({"role": "user", "content": text})
                self._episode_turns.append({"role": "assistant", "content": said})
                # Bound active depth by tokens, not a fixed pair count.
                self._episode_turns = harness.budget_trim(
                    self._episode_turns, self.active_token_budget)
            self.get_logger().info(f'Reply: {said}')
        self._maybe_log_timeline(final=not self._said_any)

        # No re-warm needed: the base system prompt is pinned once at startup and
        # stays resident, so the next turn forks from it directly (the pin is never
        # evicted). This turn's finished sequence is freed immediately
        # (recycling_seqs=0) — holding it would preempt the pin-fork and slow the
        # next distinct first turn.

    def _maybe_log_timeline(self, final=False):
        """Log the turn's [latency] line once it has both finished generating
        and started playing (or, with ``final``, whatever it has)."""
        tl = self._timeline
        if tl is None or self._timeline_logged:
            return
        if final or ('done' in tl.marks and 'first_audio' in tl.marks):
            self._timeline_logged = True
            self.get_logger().info(tl.format())

    # --- helpers ---

    def publish_generating(self, value):
        self._generating = bool(value)
        m = Bool()
        m.data = bool(value)
        self.generating_pub.publish(m)

    def _is_current(self, my_turn):
        with self._lock:
            return my_turn == self._turn

    def _nanoowl_detect(self, prompt=None, threshold=None):
        """Run one on-demand detection via the nanoowl service; return a snapshot
        dict for tools/see.py, or an ``{"error": ...}`` dict if unavailable.

        Called from a tool worker thread while the node spins in the main thread,
        so it uses call_async and blocks the worker on the future (the executor
        resolves it) — never spin from here.
        """
        if not self.nanoowl_cli.wait_for_service(timeout_sec=0.5):
            return {"error": "the vision detector isn't running."}
        req = Detect.Request()
        req.prompt = prompt or ''
        req.threshold = float(threshold) if threshold else 0.0

        done = threading.Event()
        future = self.nanoowl_cli.call_async(req)
        future.add_done_callback(lambda _f: done.set())
        if not done.wait(timeout=self._nanoowl_timeout):
            return {"error": "the vision detector timed out."}

        resp = future.result()
        if resp is None:
            return {"error": "the vision detector call failed."}
        if not resp.ok:
            return {"error": f"vision unavailable: {resp.message}"}

        d = resp.detections
        return {
            "prompt": d.prompt,
            "source_topic": d.source_topic,
            "width": d.image_width,
            "height": d.image_height,
            "detections": [
                {"label": det.label, "score": det.score,
                 "x0": det.x0, "y0": det.y0, "x1": det.x1, "y1": det.y1}
                for det in d.detections
            ],
        }

    def _enroll_voice(self, name, forget=False):
        """Learn or forget a voice via the ASR node, for tools/voice.py.

        Runs on a tool worker thread (see _nanoowl_detect): call_async + wait.
        """
        if not self.enroll_cli.wait_for_service(timeout_sec=0.5):
            return {"error": "voice recognition isn't running."}
        req = EnrollSpeaker.Request()
        req.name = name
        req.forget = bool(forget)
        done = threading.Event()
        future = self.enroll_cli.call_async(req)
        future.add_done_callback(lambda _f: done.set())
        if not done.wait(timeout=5.0):
            return {"error": "voice recognition timed out."}
        resp = future.result()
        if resp is None:
            return {"error": "voice recognition call failed."}
        return {"ok": resp.ok, "result": resp.message}

    def _on_estop_state(self, msg):
        engaged = bool(msg.data)
        if engaged and not self._estopped:
            self._estop_since = time.monotonic()
        self._estopped = engaged

    def _estop_reset(self):
        """Clear the command_mux e-stop for tools/estop.py; returns a result dict.

        Runs on a tool worker thread (see _nanoowl_detect): call_async + wait,
        never spin from here.
        """
        if self._estopped is None:
            return {"error": "I don't know the e-stop state yet."}
        if not self._estopped:
            return {"ok": True, "result": "The e-stop isn't engaged; nothing to clear."}
        held = time.monotonic() - self._estop_since
        if held < self._estop_min_hold:
            return {"error": (
                f"The e-stop only just engaged; wait {self._estop_min_hold - held:.0f} "
                "more seconds and check the robot is clear, then ask again.")}
        if not self.estop_reset_cli.wait_for_service(timeout_sec=0.5):
            return {"error": "the e-stop reset service isn't running."}

        done = threading.Event()
        future = self.estop_reset_cli.call_async(Trigger.Request())
        future.add_done_callback(lambda _f: done.set())
        if not done.wait(timeout=3.0):
            return {"error": "the e-stop reset timed out."}
        resp = future.result()
        if resp is None or not resp.success:
            return {"error": f"the e-stop reset failed: {resp.message if resp else 'no reply'}"}
        self.get_logger().warn('E-stop cleared by voice request (clear_estop tool)')
        return {"ok": True, "result": "E-stop cleared; the robot can move again."}

    # --- idle compaction: episode -> memory subsystem ---

    def _on_idle_tick(self):
        """
        Compact the episode into memory once the conversation goes idle.

        Fires when the follow-up window has lapsed with un-compacted turns: hands
        the episode off for background summarization and resets active context.
        """
        if not self.compaction_enabled:
            return
        now = time.monotonic()
        with self._lock:
            if self._window_open(now):
                return                      # window still open / turn running
            if not self._episode_turns:
                return                      # nothing to compact
            if self._compacting or self._generating or self._speaking:
                return                      # a turn/TTS/compaction is in flight
            turns = self._episode_turns
            speakers = sorted(self._episode_speakers)
            self._episode_turns = []        # hard reset of active context
            self._episode_speakers = set()
            self._episode_system_prompt = self.system_prompt
            self._interrupt_note = None
            self._compacting = True
        self.get_logger().info(
            f'Episode idle; compacting {len(turns) // 2} turn(s) into memory.')
        threading.Thread(target=self._compact, args=(turns, speakers),
                         daemon=True).start()

    def _compact(self, turns, speakers=()):
        try:
            res = memory.record_episode(
                turns,
                summarize_fn=lambda t: harness.summarize(t, self.llm,
                                                         speakers=speakers),
                now=datetime.now(timezone.utc),
            )
            if res:
                self.get_logger().info(f"Stored memory: {res['summary']}")
        except Exception as e:
            self.get_logger().error(f'Compaction failed: {e}')
        finally:
            with self._lock:
                self._compacting = False
        # No re-warm: summarize() ran with a different system prompt, but the pinned
        # BASE prompt is stable and still resident, so the next wake's first turn
        # forks from it regardless. (The pin outlives summarize's transient sequence.)

    def _pin_base_prefix(self):
        """Pin the constant BASE system prompt resident for fast first turns.

        Sends one ``max_tokens=1`` completion on ``[base_system_prompt, 'hi']`` with
        ``debug_config.pinned_system_prompt=True``. The engine keeps that sequence
        resident permanently (never recycled, never LRU-evicted), so every real first
        turn FORKS from it at the system-prompt boundary and prefills only the user
        delta — cold ~2.3s -> ~0.31s (validated on-device 2026-07-13).

        We pin the *base* prompt (``self.system_prompt``, soul + tool specs), NOT the
        episode prompt: the base is constant, so one pin serves every episode with no
        accumulation (there is no unpin API — pinning a new prompt each episode would
        leak rnn_state slots and eventually exhaust them). Episodes that fold in a
        recent-memory preamble still fork from the base and prefill the (small)
        preamble; the server needs --enable-debug + max_num_sequence=2 +
        prefix_cache_max_num_recycling_seqs=0 for this to take effect
        (see mlc_llm_serve_node). Best-effort; a failure just means slower first turns.
        """
        if self._generating:
            return
        try:
            self.llm.stream_completion(
                [{"role": "system", "content": self.system_prompt},
                 {"role": "user", "content": "hi"}],
                is_current=lambda: not self._generating,
                max_tokens=1,
                debug_config={"pinned_system_prompt": True},
            )
            self.get_logger().info('Base system prompt pinned (resident fork parent).')
        except Exception as e:
            self.get_logger().warn(f'prefix pin failed: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = AgentNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
