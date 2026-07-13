"""Unit tests for the pure room store (no ROS, no model)."""
import numpy as np

from alfie_room.room_store import RoomStore, l2_normalize


def _vec(*xs):
    return np.array(xs, dtype=np.float32)


def test_empty_store_is_unknown(tmp_path):
    store = RoomStore(str(tmp_path / "rooms.json"))
    res = store.classify(_vec(1, 0, 0))
    assert res["room"] is None
    assert res["unknown"] is True
    assert res["known_rooms"] == []


def test_teach_then_recognize(tmp_path):
    store = RoomStore(str(tmp_path / "rooms.json"), sim_threshold=0.5, margin=0.05)
    store.teach("kitchen", _vec(1, 0, 0))
    store.teach("office", _vec(0, 1, 0))

    # A view close to the kitchen exemplar matches kitchen.
    res = store.classify(_vec(0.98, 0.05, 0.0))
    assert res["room"] == "kitchen"
    assert res["score"] > 0.9
    assert set(res["known_rooms"]) == {"kitchen", "office"}


def test_below_threshold_is_unknown(tmp_path):
    store = RoomStore(str(tmp_path / "rooms.json"), sim_threshold=0.8, margin=0.0)
    store.teach("kitchen", _vec(1, 0, 0))
    # Orthogonal-ish view scores low -> unknown, but best_guess still reported.
    res = store.classify(_vec(0.2, 1.0, 0.0))
    assert res["unknown"] is True
    assert res["room"] is None
    assert res["best_guess"] == "kitchen"


def test_small_margin_is_unknown(tmp_path):
    # Two rooms almost equally similar to the query -> refuse to guess.
    store = RoomStore(str(tmp_path / "rooms.json"), sim_threshold=0.1, margin=0.1)
    store.teach("a", _vec(1, 0, 0))
    store.teach("b", _vec(0.99, 0.14, 0.0))
    res = store.classify(_vec(1, 0.07, 0.0))
    assert res["unknown"] is True


def test_persistence_roundtrip(tmp_path):
    path = str(tmp_path / "rooms.json")
    store = RoomStore(path)
    store.teach("kitchen", _vec(1, 0, 0))
    store.teach("kitchen", _vec(0.9, 0.1, 0.0))

    reloaded = RoomStore(path)
    assert reloaded.rooms() == {"kitchen": 2}
    res = reloaded.classify(_vec(1, 0, 0))
    assert res["room"] == "kitchen"


def test_max_exemplars_trims_oldest(tmp_path):
    store = RoomStore(str(tmp_path / "rooms.json"), max_exemplars_per_room=2)
    for _ in range(5):
        store.teach("kitchen", _vec(1, 0, 0))
    assert store.rooms()["kitchen"] == 2


def test_forget(tmp_path):
    store = RoomStore(str(tmp_path / "rooms.json"))
    store.teach("kitchen", _vec(1, 0, 0))
    assert store.forget("kitchen") is True
    assert store.rooms() == {}
    assert store.forget("kitchen") is False


def test_l2_normalize_zero_vector():
    out = l2_normalize(_vec(0, 0, 0))
    assert np.allclose(out, 0.0)
