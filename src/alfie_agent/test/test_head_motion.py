"""Unit tests for head_motion's pure keyframe plans."""
import math

import pytest

from alfie_agent import head_motion as hm


def test_bearing_sign_left_is_negative_yaw():
    assert hm.bearing_to_yaw(30.0, 0.0) == pytest.approx(-math.radians(30))
    assert hm.bearing_to_yaw(-30.0, 0.5) == pytest.approx(0.5 + math.radians(30))
    assert hm.bearing_to_yaw(0.0, 0.5, relative=False) == 0.0


def test_elevation_up_is_negative_pitch():
    assert hm.elevation_to_pitch(10.0, 0.1) == pytest.approx(0.1 - math.radians(10))


def test_image_offset_center_is_zero_and_left_is_positive():
    assert hm.image_offset_to_angles(0.5, 0.5, 104.0) == pytest.approx((0.0, 0.0))
    b, e = hm.image_offset_to_angles(0.0, 0.0, 104.0)
    assert b == pytest.approx(52.0, abs=0.1) and e > 0


def test_look_plan_clamps_and_includes_travel():
    frames, end = hm.look_plan((0.0, 0.0, 0.0), (5.0, 2.0, 0.0), hold_s=2.0)
    _, yaw, pitch, roll, speed = frames[0]
    assert yaw == hm.YAW_LIMITS[1] and pitch == hm.PITCH_LIMITS[1]
    assert end == pytest.approx(2.0 + 3.14 / hm.LOOK_SPEED)


@pytest.mark.parametrize("name", hm.GESTURES)
def test_gestures_return_to_start_and_stay_in_limits(name):
    cur = (0.2, 0.1, 0.0)
    frames, end = hm.gesture_plan(name, cur)
    assert frames[-1][1:4] == pytest.approx(cur)
    assert end > frames[-1][0]
    for _, y, p, r, s in frames:
        assert hm.clamp_pose(y, p, r) == (y, p, r) and s > 0


def test_unknown_gesture():
    with pytest.raises(ValueError):
        hm.gesture_plan("dance", (0, 0, 0))


def test_frame_at_picks_latest_started():
    frames = [(0.0, 1, 0, 0, 1), (1.0, 2, 0, 0, 1)]
    assert hm.frame_at(frames, 0.5)[1] == 1 and hm.frame_at(frames, 1.5)[1] == 2
