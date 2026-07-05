"""Tests for control API: set_angle, wait, parallel moves, error paths."""

from __future__ import annotations

import time
import threading

import pytest

from armsim.joint import Joint
from armsim.arm import Arm
from armsim.errors import (
    JointLimitError,
    ArmNotStartedError,
    ArmAlreadyStartedError,
)


class TestJointControl:
    """Unit tests for individual Joint control methods."""

    def test_set_angle_in_range(self):
        """Setting a valid angle succeeds and marks the joint as moving."""
        j = Joint(name="test", angle_min=-90, angle_max=90)
        j.set_angle(45.0)
        assert j.get_target_angle() == 45.0
        # Target differs from current (0), so joint should be moving.
        assert j.is_moving() is True

    def test_set_angle_out_of_range_raises(self):
        """Setting an angle outside limits raises JointLimitError."""
        j = Joint(name="test", angle_min=0, angle_max=100)
        with pytest.raises(JointLimitError) as exc:
            j.set_angle(150.0)
        assert "test" in str(exc.value)
        assert "150" in str(exc.value)
        # Target should not have changed.
        assert j.get_target_angle() == 0.0

    def test_set_angle_negative_velocity_raises(self):
        """Velocity must be > 0."""
        j = Joint(name="test")
        with pytest.raises(Exception):
            j.set_angle(45.0, velocity=-5.0)

    def test_stop_freezes_at_current(self):
        """stop() sets target to current angle."""
        j = Joint(name="test", initial_angle=30.0)
        j.set_angle(90.0)
        j.stop()
        assert j.get_target_angle() == 30.0
        assert j.is_moving() is False


class TestArmLifecycle:
    """Tests for Arm start/stop lifecycle."""

    def test_start_twice_raises(self):
        """Calling start() twice raises ArmAlreadyStartedError."""
        arm = Arm([Joint(name="j1", link_length=10)])
        arm.start(mode="local", port=0)  # port 0 = OS picks
        try:
            with pytest.raises(ArmAlreadyStartedError):
                arm.start(mode="local", port=0)
        finally:
            arm.stop_server()

    def test_methods_before_start_raise(self):
        """Calling control methods before start() raises ArmNotStartedError."""
        arm = Arm([Joint(name="j1", link_length=10)])
        with pytest.raises(ArmNotStartedError):
            arm.get_end_effector_position()
        with pytest.raises(ArmNotStartedError):
            arm.get_state()
        with pytest.raises(ArmNotStartedError):
            arm.get_joint_positions()


class TestSetAngles:
    """Tests for Arm.set_angles() — parallel multi-joint commands."""

    def test_set_angles_all_or_nothing(self):
        """If one joint's target is invalid, none are applied."""
        j1 = Joint(name="a", angle_min=0, angle_max=100)
        j2 = Joint(name="b", angle_min=0, angle_max=100)
        arm = Arm([j1, j2])

        with pytest.raises(JointLimitError):
            arm.set_angles({"a": 50, "b": 200})
        # Neither target should have changed.
        assert j1.get_target_angle() == 0.0
        assert j2.get_target_angle() == 0.0

    def test_set_angles_unknown_joint_raises(self):
        """Referencing a nonexistent joint raises KeyError."""
        arm = Arm([Joint(name="a", link_length=10)])
        with pytest.raises(KeyError):
            arm.set_angles({"zzz": 45})


class TestArmMotion:
    """Integration tests: start arm, command motion, check positions."""

    def test_single_joint_reaches_target(self):
        """A joint commanded via a running arm reaches its target."""
        arm = Arm([Joint(name="j1", link_length=10, default_velocity=180)])  # fast
        arm.start(mode="local", port=0, tick_rate_hz=60)
        try:
            arm.joints["j1"].set_angle(90.0)
            # Wait for motion.
            arm.joints["j1"].wait(timeout=5.0)
            angle = arm.joints["j1"].get_angle()
            assert abs(angle - 90.0) < 0.01
        finally:
            arm.stop_server()

    def test_parallel_motion(self):
        """Multiple joints move in parallel toward different targets."""
        arm = Arm(
            [
                Joint(name="a", link_length=5, default_velocity=360),
                Joint(name="b", link_length=5, default_velocity=360),
            ]
        )
        arm.start(mode="local", port=0, tick_rate_hz=60)
        try:
            arm.set_angles({"a": 45.0, "b": 60.0})
            arm.wait_until_idle(timeout=5.0)
            assert abs(arm.joints["a"].get_angle() - 45.0) < 0.01
            assert abs(arm.joints["b"].get_angle() - 60.0) < 0.01
        finally:
            arm.stop_server()

    def test_blocking_set_angle(self):
        """blocking=True version of set_angle blocks until target reached."""
        arm = Arm([Joint(name="j1", link_length=10, default_velocity=360)])
        arm.start(mode="local", port=0, tick_rate_hz=60)
        try:
            arm.joints["j1"].set_angle(45.0, blocking=True, timeout=5.0)
            assert abs(arm.joints["j1"].get_angle() - 45.0) < 0.01
        finally:
            arm.stop_server()

    def test_timeout_raises(self):
        """wait() with a short timeout raises TimeoutError."""
        arm = Arm([Joint(name="j1", link_length=10, default_velocity=10)])  # slow
        arm.start(mode="local", port=0, tick_rate_hz=60)
        try:
            arm.joints["j1"].set_angle(90.0)
            with pytest.raises(TimeoutError):
                arm.joints["j1"].wait(timeout=0.01)
        finally:
            arm.stop_server()

    def test_get_end_effector_position(self):
        """EE position updates as the arm moves."""
        arm = Arm([Joint(name="j1", link_length=10, default_velocity=360)])
        arm.start(mode="local", port=0, tick_rate_hz=60)
        try:
            ee = arm.get_end_effector_position()
            assert isinstance(ee, tuple)
            assert len(ee) == 3
        finally:
            arm.stop_server()

    def test_get_state_returns_armstate(self):
        """get_state() returns an ArmState with correct structure."""
        arm = Arm([Joint(name="j1", link_length=10)])
        arm.start(mode="local", port=0, tick_rate_hz=60)
        try:
            s = arm.get_state()
            assert len(s.joints) == 1
            assert s.joints[0].name == "j1"
            assert isinstance(s.end_effector_position, tuple)
        finally:
            arm.stop_server()
