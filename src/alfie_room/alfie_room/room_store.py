"""
room_store — the learned-room database (pure, no ROS / no model).

Holds one or more *exemplar embeddings* per named room and answers two questions:
``teach(name, vec)`` (remember this view as room ``name``) and ``classify(vec)``
(which known room does this view look most like?). Embeddings are L2-normalized
unit vectors, so cosine similarity is just a dot product.

Persistence is a single JSON file — ``{"rooms": {name: [{"vec": [...],
"ts": iso}, ...]}}`` — chosen over a binary format so the store stays greppable
and diff-friendly, matching the rest of Alfie's plain-file memory. Vectors are a
few hundred floats each and there are a handful of rooms, so JSON is cheap here.

Classification returns the best room, its score, the runner-up, and the full
known-room list, plus an ``unknown`` flag. A view is called *unknown* when the
best score is below ``sim_threshold`` OR its lead over the runner-up is smaller
than ``margin`` — the first guards against "nothing looks familiar", the second
against "two rooms look equally like this, don't guess".

This module is deliberately dependency-light (numpy only) so it unit-tests
without a camera, a model, or ROS.
"""
import json
import os
import tempfile
from datetime import datetime, timezone

import numpy as np

# Defaults; the node overrides these from ROS params.
DEFAULT_SIM_THRESHOLD = 0.55
DEFAULT_MARGIN = 0.05


def _now_iso():
    return datetime.now(timezone.utc).isoformat(timespec="seconds")


def l2_normalize(vec):
    """Scale ``vec`` to unit length (float32).

    A zero vector is returned unchanged rather than divided by zero.
    """
    arr = np.asarray(vec, dtype=np.float32).reshape(-1)
    norm = float(np.linalg.norm(arr))
    if norm < 1e-12:
        return arr
    return arr / norm


class RoomStore:
    """A persistent set of rooms, each with a list of exemplar embeddings."""

    def __init__(self, path, sim_threshold=DEFAULT_SIM_THRESHOLD,
                 margin=DEFAULT_MARGIN, max_exemplars_per_room=0):
        """``path`` is the JSON file backing the store (created on first write).

        ``max_exemplars_per_room`` caps how many exemplars a room keeps (0 =
        unbounded); when exceeded the oldest is dropped, so re-teaching a room
        slowly refreshes it rather than growing without bound.
        """
        self.path = os.path.expanduser(path) if path else None
        self.sim_threshold = float(sim_threshold)
        self.margin = float(margin)
        self.max_exemplars_per_room = int(max_exemplars_per_room)
        # name -> np.ndarray of shape (n_exemplars, dim), each row L2-normalized.
        self._rooms = {}
        # name -> list of iso timestamps, parallel to the rows above.
        self._ts = {}
        self.load()

    # --- persistence ----------------------------------------------------------

    def load(self):
        """Load the store from disk; a missing/corrupt file yields an empty store."""
        self._rooms = {}
        self._ts = {}
        if not self.path or not os.path.exists(self.path):
            return
        try:
            with open(self.path, "r") as f:
                data = json.load(f)
        except (OSError, ValueError):
            return
        for name, exemplars in (data.get("rooms") or {}).items():
            vecs, stamps = [], []
            for ex in exemplars:
                vec = ex.get("vec") if isinstance(ex, dict) else ex
                if vec is None:
                    continue
                vecs.append(l2_normalize(vec))
                stamps.append(ex.get("ts", "") if isinstance(ex, dict) else "")
            if vecs:
                self._rooms[name] = np.vstack(vecs)
                self._ts[name] = stamps

    def save(self):
        """Atomically write the store to ``self.path`` (temp file + rename)."""
        if not self.path:
            return
        payload = {"rooms": {
            name: [{"vec": self._rooms[name][i].tolist(), "ts": self._ts[name][i]}
                   for i in range(len(self._ts[name]))]
            for name in self._rooms
        }}
        os.makedirs(os.path.dirname(self.path) or ".", exist_ok=True)
        fd, tmp = tempfile.mkstemp(dir=os.path.dirname(self.path) or ".",
                                   suffix=".tmp")
        try:
            with os.fdopen(fd, "w") as f:
                json.dump(payload, f)
            os.replace(tmp, self.path)
        finally:
            if os.path.exists(tmp):
                os.remove(tmp)

    # --- mutation -------------------------------------------------------------

    def teach(self, name, vec, *, save=True):
        """Add ``vec`` as an exemplar of room ``name`` (creating it if new).

        Returns the room's exemplar count after the addition.
        """
        name = (name or "").strip()
        if not name:
            raise ValueError("room name is required")
        row = l2_normalize(vec).reshape(1, -1)
        if name in self._rooms:
            self._rooms[name] = np.vstack([self._rooms[name], row])
            self._ts[name].append(_now_iso())
        else:
            self._rooms[name] = row
            self._ts[name] = [_now_iso()]
        # Trim oldest exemplars if a cap is set.
        cap = self.max_exemplars_per_room
        if cap and len(self._ts[name]) > cap:
            drop = len(self._ts[name]) - cap
            self._rooms[name] = self._rooms[name][drop:]
            self._ts[name] = self._ts[name][drop:]
        if save:
            self.save()
        return len(self._ts[name])

    def forget(self, name, *, save=True):
        """Remove a room entirely. Returns True if it existed."""
        existed = name in self._rooms
        self._rooms.pop(name, None)
        self._ts.pop(name, None)
        if existed and save:
            self.save()
        return existed

    # --- query ----------------------------------------------------------------

    def rooms(self):
        """Return ``{name: exemplar_count}`` for every known room."""
        return {name: int(self._rooms[name].shape[0]) for name in self._rooms}

    def classify(self, vec):
        """Match ``vec`` against known rooms.

        Returns a dict: ``{room, score, unknown, runner_up, runner_up_score,
        scores, known_rooms}``. ``room`` is None (and ``unknown`` True) when the
        store is empty or the match fails the threshold/margin test.
        """
        known = sorted(self._rooms.keys())
        if not self._rooms:
            return {"room": None, "score": 0.0, "unknown": True,
                    "runner_up": None, "runner_up_score": 0.0,
                    "scores": {}, "known_rooms": []}

        q = l2_normalize(vec)
        # Best (max) cosine similarity of the query against each room's exemplars.
        scores = {name: float(np.max(self._rooms[name] @ q)) for name in self._rooms}
        ranked = sorted(scores.items(), key=lambda kv: kv[1], reverse=True)

        best_name, best_score = ranked[0]
        runner_name, runner_score = (ranked[1] if len(ranked) > 1 else (None, 0.0))

        unknown = (best_score < self.sim_threshold or
                   (runner_name is not None and
                    (best_score - runner_score) < self.margin))

        return {
            "room": None if unknown else best_name,
            "score": round(best_score, 4),
            "unknown": unknown,
            "best_guess": best_name,          # the top match even when unknown
            "runner_up": runner_name,
            "runner_up_score": round(runner_score, 4),
            "scores": {k: round(v, 4) for k, v in scores.items()},
            "known_rooms": known,
        }
