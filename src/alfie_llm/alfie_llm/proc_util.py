"""Process-supervision helpers for ROS nodes that spawn background executables.

A node that `Popen`s a long-lived server (mlc_llm, ollama, …) must guarantee the
child is torn down when the node exits — otherwise a killed/respawned node leaves
an orphaned server holding the GPU/port, which is exactly how several duplicate
LLM servers accumulated on the Orin.

Three exit paths must be covered:
  * SIGINT  (Ctrl-C / ros2 launch initial shutdown) — caught as KeyboardInterrupt,
    node's `finally` runs `destroy_node()` -> `terminate_group()`.
  * SIGTERM (ros2 launch escalation, plain `kill <pid>`) — Python's default action
    exits WITHOUT unwinding `finally`, so the child would leak. `install_sigterm_shutdown()`
    re-routes SIGTERM through the KeyboardInterrupt path so cleanup still runs.
  * SIGKILL / crash / OOM — no Python code can run. `spawn_supervised()` arms the
    kernel's parent-death signal (PR_SET_PDEATHSIG) so the child is reaped anyway.
"""
import ctypes
import ctypes.util
import os
import signal
import subprocess
import time

# prctl(PR_SET_PDEATHSIG, sig): the kernel delivers `sig` to the calling process
# when its parent dies, for ANY reason (SIGKILL, segfault, OOM) — the only
# mechanism that reaps a child when the parent cannot run cleanup itself.
_PR_SET_PDEATHSIG = 1
_libc = ctypes.CDLL(ctypes.util.find_library("c") or "libc.so.6", use_errno=True)


def _pdeathsig_preexec(sig):
    """Return a preexec_fn that arms PR_SET_PDEATHSIG in the child.

    Runs in the child after fork()/setsid(), before exec(). The setting is
    preserved across a normal execve(), so it stays armed on the final server
    process. The getppid() guard closes the race where the parent already died
    between fork and prctl (otherwise the child would never be signalled).
    """
    def _apply():
        _libc.prctl(_PR_SET_PDEATHSIG, sig, 0, 0, 0)
        if os.getppid() == 1:  # parent already gone (reparented to init)
            os.kill(os.getpid(), sig)
    return _apply


def spawn_supervised(cmd, death_sig=signal.SIGKILL, **popen_kwargs):
    """Popen `cmd` in its own session, wired to die with the owning node.

    * ``start_new_session=True`` puts the child at the head of its own process
      group so ``terminate_group()`` can signal the whole subtree at once.
    * ``PR_SET_PDEATHSIG`` guarantees the child dies if the node is SIGKILLed or
      crashes without a chance to clean up.
    """
    popen_kwargs.setdefault("start_new_session", True)
    return subprocess.Popen(cmd, preexec_fn=_pdeathsig_preexec(death_sig), **popen_kwargs)


def terminate_group(proc, grace=5.0, first=signal.SIGINT, logger=None):
    """Tear down `proc`'s whole process group, escalating first -> SIGKILL.

    `first` defaults to SIGINT so servers that trap it (mlc_llm, ollama) can
    release the GPU/port cleanly; anything still alive after `grace` seconds is
    SIGKILLed.
    """
    if proc is None or proc.poll() is not None:
        return
    try:
        pgid = os.getpgid(proc.pid)
    except ProcessLookupError:
        return
    try:
        os.killpg(pgid, first)
    except ProcessLookupError:
        return
    deadline = time.monotonic() + grace
    while time.monotonic() < deadline:
        if proc.poll() is not None:
            return
        time.sleep(0.1)
    if logger is not None:
        logger.warn(f"child pgid {pgid} ignored {first!s}; sending SIGKILL")
    try:
        os.killpg(pgid, signal.SIGKILL)
    except ProcessLookupError:
        pass


def _raise_keyboard_interrupt(signum, frame):
    raise KeyboardInterrupt()


def install_sigterm_shutdown():
    """Make SIGTERM raise KeyboardInterrupt so a node's try/finally cleanup runs.

    Call once, after rclpy.init(), before rclpy.spin(). ros2 launch sends SIGINT
    first but escalates to SIGTERM, and a bare `kill <pid>` is SIGTERM too;
    without this the default handler would skip `finally` and leak the child.
    """
    signal.signal(signal.SIGTERM, _raise_keyboard_interrupt)
