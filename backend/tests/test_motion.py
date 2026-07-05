"""Tests for motion model: velocity ramping, idle detection, limit enforcement."""

from __future__ import annotations

from armsim.motion import compute_angle_step


def test_no_motion_when_at_target():
    """When current == target, return unchanged and idle."""
    angle, idle = compute_angle_step(45.0, 45.0, 30.0, 0.1, -180.0, 180.0)
    assert angle == 45.0
    assert idle is True


def test_move_toward_target():
    """Joint advances toward target at correct velocity."""
    new_angle, idle = compute_angle_step(0.0, 30.0, 30.0, 0.5, -180.0, 180.0)
    # In 0.5s at 30 deg/s → should move 15 deg
    assert new_angle == 15.0
    assert idle is False


def test_move_toward_target_negative():
    """Joint advances toward negative target."""
    new_angle, idle = compute_angle_step(0.0, -30.0, 30.0, 0.5, -180.0, 180.0)
    assert new_angle == -15.0
    assert idle is False


def test_clamp_at_target():
    """Joint stops exactly at target, not past it."""
    new_angle, idle = compute_angle_step(0.0, 10.0, 30.0, 0.5, -180.0, 180.0)
    # In 0.5s at 30 deg/s → can move 15 deg, but target is only 10 away
    assert new_angle == 10.0
    assert idle is True


def test_idle_detection_within_epsilon():
    """Within ANGLE_EPSILON_DEG of target → idle."""
    new_angle, idle = compute_angle_step(89.9995, 90.0, 30.0, 0.001, -180.0, 180.0)
    # Very close to target — should be treated as idle
    # diff = 0.0005 < 1e-3 epsilon
    assert idle is True


def test_clamp_at_angle_min():
    """Joint cannot move below angle_min."""
    new_angle, idle = compute_angle_step(-170.0, -200.0, 30.0, 0.5, -180.0, 180.0)
    assert new_angle == -180.0


def test_clamp_at_angle_max():
    """Joint cannot move above angle_max."""
    new_angle, idle = compute_angle_step(170.0, 200.0, 30.0, 0.5, -180.0, 180.0)
    assert new_angle == 180.0


def test_different_velocities():
    """Faster velocity → larger step per tick."""
    step_slow, _ = compute_angle_step(0.0, 90.0, 15.0, 0.1, -180.0, 180.0)
    step_fast, _ = compute_angle_step(0.0, 90.0, 60.0, 0.1, -180.0, 180.0)
    assert step_fast > step_slow


def test_zero_dt():
    """Zero elapsed time → no motion."""
    new_angle, idle = compute_angle_step(0.0, 90.0, 30.0, 0.0, -180.0, 180.0)
    assert new_angle == 0.0
    assert idle is False
