"""Unit tests for speaker_id with a fake embedder (no model download)."""
import numpy as np

from alfie_asr.speaker_id import SpeakerID, fbank

SR = 16000


def _voice(seed, n=4):
    """A fake 'voice': a fixed direction plus a little per-utterance noise."""
    base = np.random.default_rng(seed).standard_normal(512)
    rng = np.random.default_rng(seed + 100)
    return [base + 0.3 * rng.standard_normal(512) for _ in range(n)]


def _sid(tmp_path, embs):
    it = iter(embs)
    return SpeakerID(None, tmp_path / "v.json", embed_fn=lambda a: next(it),
                     enroll_min_s=2.0)


def test_enroll_identify_and_persist(tmp_path):
    alan, bob = _voice(1), _voice(2)
    sid = _sid(tmp_path, alan[:3] + [alan[3], bob[0]])
    clip = np.zeros(SR * 1, dtype=np.float32)
    for _ in range(3):
        assert sid.identify(clip) == ("", 0.0)       # nobody enrolled yet
    assert sid.enroll("Alan")[0]
    name, score = sid.identify(clip)
    assert name == "Alan" and score > 0.8
    assert sid.identify(clip)[0] == ""                # a different voice
    reloaded = SpeakerID(None, tmp_path / "v.json", embed_fn=lambda a: alan[0])
    assert reloaded.names() == ["Alan"]
    assert (tmp_path / "v.json").stat().st_mode & 0o777 == 0o600


def test_short_audio_is_anonymous_and_not_kept(tmp_path):
    sid = _sid(tmp_path, [])
    assert sid.identify(np.zeros(SR // 2, dtype=np.float32)) == ("", 0.0)
    ok, msg = sid.enroll("Alan")
    assert not ok and "seconds of speech" in msg


def test_enroll_drops_another_voice(tmp_path):
    alan, bob = _voice(1), _voice(2)
    sid = _sid(tmp_path, alan[:3] + bob[:1] + alan[3:4])
    clip = np.zeros(SR, dtype=np.float32)
    for _ in range(4):
        sid.identify(clip)
    ok, msg = sid.enroll("Alan")
    assert ok and "3 utterance" in msg
    assert sid.identify(clip)[0] == "Alan"


def test_forget(tmp_path):
    sid = _sid(tmp_path, _voice(1, 3))
    for _ in range(3):
        sid.identify(np.zeros(SR, dtype=np.float32))
    sid.enroll("Alan")
    assert sid.forget("alan")[0] and sid.names() == []
    assert not sid.forget("Alan")[0]


def test_fbank_shape():
    assert fbank(np.zeros(SR, dtype=np.float32)).shape == (98, 80)
    assert fbank(np.zeros(100, dtype=np.float32)).shape == (0, 80)
