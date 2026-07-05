"""Shared pytest fixtures for armsim tests."""

from __future__ import annotations

import math
import tempfile
from pathlib import Path

import pytest

from armsim.joint import Joint
from armsim.arm import Arm


# ── Fixtures for standard arm configurations ───────────────────────────


@pytest.fixture
def two_joint_planar() -> Arm:
    """2-DOF planar arm: both joints axis="z" (alpha=0), rotating about Z.
    Motion is in the X-Y plane. Matches the classic closed form:
    x = L1*cos(t1)+L2*cos(t1+t2), y = L1*sin(t1)+L2*sin(t1+t2), z=0."""
    return Arm(
        [
            Joint(name="j1", axis="z", link_length=10.0, link_offset=0.0),
            Joint(name="j2", axis="z", link_length=8.0, link_offset=0.0),
        ]
    )


@pytest.fixture
def two_joint_planar_with_offsets() -> Arm:
    """Same arm with link offsets to test 3D behaviour."""
    return Arm(
        [
            Joint(name="j1", axis="z", link_length=10.0, link_offset=5.0),
            Joint(name="j2", axis="z", link_length=8.0, link_offset=0.0),
        ]
    )


@pytest.fixture
def three_joint_with_alpha() -> Arm:
    """3-DOF arm with non-zero alpha on j1, producing genuine 3D motion."""
    return Arm(
        [
            Joint(name="base", axis="z", link_length=0.0, link_offset=2.0, dh_alpha=90.0),
            Joint(name="shoulder", axis="pitch", link_length=15.0, link_offset=0.0),
            Joint(name="elbow", axis="pitch", link_length=10.0, link_offset=0.0),
        ]
    )


@pytest.fixture
def six_joint_industrial() -> Arm:
    """6-axis industrial-style arm matching the config doc example."""
    return Arm(
        [
            Joint(name="base", axis="z", link_length=0.0, link_offset=10.0,
                  angle_min=-180, angle_max=180),
            Joint(name="shoulder", axis="pitch", link_length=30.0,
                  angle_min=-90, angle_max=90),
            Joint(name="elbow", axis="pitch", link_length=25.0,
                  angle_min=0, angle_max=150),
            Joint(name="wrist_roll", axis="z", link_length=0.0,
                  angle_min=-180, angle_max=180),
            Joint(name="wrist_pitch", axis="pitch", link_length=8.0,
                  angle_min=-90, angle_max=90),
            Joint(name="wrist_yaw", axis="yaw", link_length=5.0,
                  angle_min=-180, angle_max=180, default_velocity=45.0),
        ]
    )


@pytest.fixture
def single_joint() -> Arm:
    """Minimal single-joint arm for edge-case testing."""
    return Arm([Joint(name="only", axis="z", link_length=10.0)])


@pytest.fixture
def zero_link_arm() -> Arm:
    """Arm with all zero-length links — end effector always at origin."""
    return Arm(
        [
            Joint(name="a", axis="z", link_length=0.0),
            Joint(name="b", axis="z", link_length=0.0),
            Joint(name="c", axis="z", link_length=0.0),
        ]
    )


@pytest.fixture
def temp_dir() -> str:
    """Temporary directory for writing/reading config files during tests."""
    with tempfile.TemporaryDirectory() as d:
        yield d
