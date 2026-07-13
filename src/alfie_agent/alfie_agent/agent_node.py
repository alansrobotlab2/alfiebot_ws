"""
agent_node — the conversation bridge with a tool-calling brain.

Closes the loop between ASR and TTS: it subscribes to transcripts (`asrresult`),
runs an agentic turn against the local MLC-LLM server (Qwen3.6 35B-A3B) — which
may call tools such as reading/writing notes in the obsidian vault — and
publishes the spoken reply as a `speechrequest` for TTS. It also publishes a
`generating` flag (true while a turn is in flight) that drives the "thinking" LED,
and honours `barge_in` so a user can interrupt.

The thinking is delegated to `harness.run_turn`: MLC can't do native OpenAI
tool-calling, so tools are described in the system prompt and Qwen emits
`<tool_call>` blocks that the harness parses and dispatches (see harness.py,
prompt_builder.py, tools/). This node keeps all ROS I/O and turn management.

Design notes:
  * Wake-gated: transcripts are only processed while a listening window (opened
    by a `wake` event) is open, so background speech is ignored. The window
    re-opens after each reply for hands-free follow-ups; a bare wake phrase is
    stripped and just (re)opens the window.
  * Gated on `llm/ready` so nothing is sent before the 35B server has loaded.
  * Each turn runs in a worker thread so ROS callbacks never block. While tools
    run the robot is silent — the `generating` LED covers that gap.
  * Turns are invalidated by a monotonically increasing turn id: a new transcript
    or a barge-in bumps the id, and any in-flight worker notices (before each LLM
    call and tool call, and on every streamed chunk) and bails without speaking.
"""
import re
import threading
import time
from datetime import datetime, timezone

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

from std_msgs.msg import Bool, Empty, String
from alfie_msgs.msg import ASRResult, SpeechRequest, Speaking

from alfie_agent import harness, memory, prompt_builder, tools
from alfie_agent.llm_client import LLMClient

# Wake-word gating: a `wake` event opens a listening window; transcripts are only
# processed while it is open. The window is (re)opened on a wake, held open across
# the LLM+TTS latency of a turn, and then reopened for FOLLOWUP_WINDOW_S measured
# from the *end* of Alfie's reply — so the user always gets the full follow-up gap
# after Alfie stops talking, regardless of how long the reply took. That reopen is
# gated on an active episode, so a stray/background reply can't keep it alive
# indefinitely. When it lapses, the wake word is required again (and the next wake
# starts a fresh conversation).
FOLLOWUP_WINDOW_S = 5.0
WAKE_PHRASES_DEFAULT = {'hey alfie', 'alfie'}
# A very short transcript arriving right after a wake is the (possibly mis-heard)
# wake phrase itself, not a command — acknowledge it instead of answering it.
WAKE_UTTERANCE_SUPPRESS_S = 3.0
MAX_WAKE_UTTERANCE_WORDS = 2
SPEAK_VOLUME = 100


class AgentNode(Node):
    def __init__(self):
        super().__init__('agent_node')

        # --- parameters (workspace idiom: declare_parameter with defaults) ---
        self.llm_base_url = self.declare_parameter(
            'llm_base_url', 'http://localhost:8000/v1').value
        model_id_fallback = self.declare_parameter(
            'model_id', 'dist/qwen3_6-35B-A3B-q4f16_1').value
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
        self.max_tool_iters = int(self.declare_parameter('max_tool_iters', 4).value)
        # Kept declared for launch-file compatibility; superseded by the
        # token-budget trim below (active depth is now bounded by tokens, not a
        # fixed pair count).
        self.declare_parameter('max_history_turns', 6)
        max_tokens = int(self.declare_parameter('max_tokens', 200).value)
        temperature = float(self.declare_parameter('temperature', 0.7).value)

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

        # --- brain: tools, system prompt, LLM client ---
        tools.configure(self.vault_root, qmd_url=qmd_url,
                        qmd_skip_rerank=qmd_skip_rerank)
        # Keep the soul + tool specs so the system prompt can be rebuilt per
        # episode with the recent-memory window folded in (see on_wake).
        self._tool_specs = tools.list_tools()
        self._soul = prompt_builder.load_soul(self.vault_root)
        self.system_prompt = prompt_builder.build_system_prompt(
            self._soul, self._tool_specs)
        self.llm = LLMClient(self.llm_base_url, model_id_fallback,
                             timeout=(5, 60), temperature=temperature,
                             max_tokens=max_tokens)

        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)

        self.speech_pub = self.create_publisher(SpeechRequest, 'speechrequest', qos)
        self.generating_pub = self.create_publisher(Bool, 'generating', qos)

        self.create_subscription(ASRResult, 'asrresult', self.on_asrresult, qos)
        self.create_subscription(Empty, 'barge_in', self.on_barge_in, qos)
        self.create_subscription(Speaking, 'speaking', self.on_speaking, qos)

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
        self._turn = 0
        # Active context for the *live* episode only (reset on idle compaction).
        self._episode_turns = []    # list of {"role", "content"}
        # System prompt for the live episode: the base prompt, or (on a fresh
        # wake) the base prompt with a recent-memory window folded in. Rebuilt at
        # episode start so the window rides inside the position-0 system message.
        self._episode_system_prompt = self.system_prompt
        self._compacting = False    # a background compaction is in flight
        self._generating = False    # a turn is generating (mirrors the LED flag)

        # Wake-gating state.
        self._listen_until = 0.0    # monotonic deadline; window open while now < it
        self._wake_time = 0.0       # monotonic time of the last wake event
        self._speaking = False
        self._wake_phrases = set(WAKE_PHRASES_DEFAULT)

        # Idle detector: when the follow-up window lapses with un-compacted turns,
        # summarize the episode into memory (GPU is free then) and reset context.
        self.create_timer(2.0, self._on_idle_tick)

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
        elif not self._llm_ready and was_ready:
            self.get_logger().warn('LLM went not-ready; pausing.')

    def on_wake(self, msg):
        phrase = (msg.data or '').strip().lower()
        now = time.monotonic()
        fresh = now >= self._listen_until   # window was closed -> new conversation
        if phrase:
            self._wake_phrases.add(phrase)
        self._listen_until = now + FOLLOWUP_WINDOW_S
        self._wake_time = now
        if fresh:
            # New episode: reset active context and fold a window of recent
            # memory (summaries of interactions from the past few minutes) into
            # the episode's system prompt.
            preamble = None
            try:
                preamble = memory.recent_preamble(
                    window_min=self.recent_window_min,
                    max_items=self.recent_max_items)
            except Exception as e:  # memory is best-effort; never block a wake
                self.get_logger().warn(f'recent memory unavailable: {e}')
            episode_prompt = prompt_builder.build_system_prompt(
                self._soul, self._tool_specs, recent_memory=preamble)
            with self._lock:
                self._episode_turns = []
                self._episode_system_prompt = episode_prompt
            self.get_logger().info(
                f"Wake '{phrase}': new conversation, listening"
                f"{' (recent memory loaded)' if preamble else ''}.")
        else:
            self.get_logger().info(f"Wake '{phrase}': window extended.")

    def on_speaking(self, msg):
        was_speaking = self._speaking
        self._speaking = msg.is_speaking
        # When Alfie finishes a reply, reopen the follow-up window so the user gets
        # the full FOLLOWUP_WINDOW_S to respond hands-free — measured from the end
        # of the reply, not from when the command was heard (a long reply would
        # otherwise eat the gap). Gated on an active episode so a stray TTS with no
        # live conversation can't hold the window open on background audio.
        if was_speaking and not msg.is_speaking:
            with self._lock:
                active = bool(self._episode_turns)
            if active:
                self._listen_until = time.monotonic() + FOLLOWUP_WINDOW_S
                self.get_logger().info(
                    f'Reply done; follow-up window open ({FOLLOWUP_WINDOW_S:.0f}s).')

    def on_asrresult(self, msg):
        text = (msg.asrresult or '').strip()
        if not text:
            return
        if not self._llm_ready:
            self.get_logger().warn('LLM not ready; ignoring transcript.',
                                   throttle_duration_sec=5.0)
            return
        now = time.monotonic()
        if now >= self._listen_until:
            self.get_logger().info(f'No wake word; ignoring: {text}',
                                   throttle_duration_sec=5.0)
            return
        command, _ = self._strip_wake(text)
        if not command:
            # Bare wake phrase ("Hey Alfie") — acknowledge and keep listening.
            self._listen_until = now + FOLLOWUP_WINDOW_S
            self.get_logger().info('Wake acknowledged; listening for a command.')
            return
        if (now - self._wake_time) < WAKE_UTTERANCE_SUPPRESS_S and \
                len(command.split()) <= MAX_WAKE_UTTERANCE_WORDS:
            # Very short transcript right after a wake = the (mis-heard) wake
            # phrase itself. Keep listening, don't answer it.
            self._listen_until = now + FOLLOWUP_WINDOW_S
            self.get_logger().info(f'Ignoring wake utterance: {command}')
            return
        # Keep the window alive across LLM + TTS for this turn.
        self._listen_until = now + FOLLOWUP_WINDOW_S
        self.get_logger().info(f'Heard: {command}')
        with self._lock:
            self._turn += 1
            my_turn = self._turn
        threading.Thread(target=self._run_turn, args=(command, my_turn),
                         daemon=True).start()

    def on_barge_in(self, msg):
        # Invalidate any in-flight turn and drop the thinking state immediately.
        with self._lock:
            self._turn += 1
        self.publish_generating(False)

    def _strip_wake(self, text):
        """
        Remove a leading wake phrase; return ``(command, had_wake)``.

        Compares word-by-word, case- and punctuation-insensitive, longest first.
        """
        words = text.split()
        low = [re.sub(r'[^a-z0-9]', '', w.lower()) for w in words]
        for ph in sorted(self._wake_phrases, key=lambda p: len(p.split()), reverse=True):
            pw = ph.split()
            if low[:len(pw)] == pw:
                return ' '.join(words[len(pw):]).strip(), True
        return text.strip(), False

    # --- helpers ---

    def publish_generating(self, value):
        self._generating = bool(value)
        m = Bool()
        m.data = bool(value)
        self.generating_pub.publish(m)

    def _is_current(self, my_turn):
        with self._lock:
            return my_turn == self._turn

    def _run_turn(self, text, my_turn):
        self.publish_generating(True)
        with self._lock:
            history = list(self._episode_turns)
            system_prompt = self._episode_system_prompt
        reply = None
        try:
            reply = harness.run_turn(
                text, history,
                system_prompt=system_prompt,
                llm=self.llm,
                call_tool=tools.call_tool,
                is_current=lambda: self._is_current(my_turn),
                logger=self.get_logger().info,
                max_tool_iters=self.max_tool_iters,
            )
        except Exception as e:
            self.get_logger().error(f'Turn failed: {e}')

        if not self._is_current(my_turn):
            return  # superseded by a newer turn or a barge-in

        self.publish_generating(False)
        if reply:
            with self._lock:
                self._episode_turns.append({"role": "user", "content": text})
                self._episode_turns.append({"role": "assistant", "content": reply})
                # Bound active depth by tokens, not a fixed pair count.
                self._episode_turns = harness.budget_trim(
                    self._episode_turns, self.active_token_budget)
            self.get_logger().info(f'Reply: {reply}')
            out = SpeechRequest()
            out.text = reply
            out.volume = SPEAK_VOLUME
            self.speech_pub.publish(out)

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
            if now < self._listen_until:
                return                      # window still open
            if not self._episode_turns:
                return                      # nothing to compact
            if self._compacting or self._generating or self._speaking:
                return                      # a turn/TTS/compaction is in flight
            turns = self._episode_turns
            self._episode_turns = []        # hard reset of active context
            self._episode_system_prompt = self.system_prompt
            self._compacting = True
        self.get_logger().info(
            f'Episode idle; compacting {len(turns) // 2} turn(s) into memory.')
        threading.Thread(target=self._compact, args=(turns,), daemon=True).start()

    def _compact(self, turns):
        try:
            res = memory.record_episode(
                turns,
                summarize_fn=lambda t: harness.summarize(t, self.llm),
                now=datetime.now(timezone.utc),
            )
            if res:
                self.get_logger().info(f"Stored memory: {res['summary']}")
        except Exception as e:
            self.get_logger().error(f'Compaction failed: {e}')
        finally:
            with self._lock:
                self._compacting = False
        # Re-warm the agent's system-prompt prefix. The summarize() call above ran
        # with a *different* system prompt and, since MLC caches only the last
        # sequence, evicted the agent prefix — so the next wake's first reply would
        # otherwise pay a full cold prefill of the ~550-token prompt. A 1-token
        # throwaway completion on the real prompt makes it the cached sequence
        # again. Runs on idle (GPU is free) and is best-effort.
        self._warm_prefix()

    def _warm_prefix(self):
        """Prime MLC's prefix cache with the base system prompt (best-effort).

        Bails immediately if a new turn has started (a live turn will warm the
        cache itself, and we don't want the throwaway competing for the GPU).
        """
        if self._generating:
            return
        try:
            self.llm.stream_completion(
                [{"role": "system", "content": self.system_prompt},
                 {"role": "user", "content": "hi"}],
                is_current=lambda: not self._generating,
                max_tokens=1,
            )
            self.get_logger().info('Prefix cache re-warmed.')
        except Exception as e:
            self.get_logger().warn(f'prefix warm failed: {e}')


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
