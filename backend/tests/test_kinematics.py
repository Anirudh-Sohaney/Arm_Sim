"""Unit tests for the kinematics core — DH transform math and forward kinematics."""

from __future__ import annotations

import math

import numpy as np
import pytest

from armsim.kinematics import (
    dh_transform,
    dh_transform_elementary,
    forward_kinematics,
    get_end_effector_position,
    project_front,
    project_top,
    total_reach,
)


# ── DH transform tests ──────────────────────────────────────────────────


def test_dh_transform_identity():
    """All-zero DH params produce the identity matrix."""
    T = dh_transform(0.0, 0.0, 0.0, 0.0)
    assert np.allclose(T, np.eye(4))


def test_dh_transform_elementary_matches_closed_form():
    """The elementary-transform implementation must match the closed form."""
    test_cases = [
        (0.0, 0.0, 0.0, 0.0),
        (30.0, 5.0, 10.0, 0.0),
        (45.0, 0.0, 8.0, 90.0),
        (-90.0, 2.0, 15.0, -90.0),
        (180.0, 10.0, 20.0, 45.0),
        (0.0, 0.0, 10.0, 90.0),
        (37.5, 3.2, 12.7, 67.3),
    ]
    for theta, d, a, alpha in test_cases:
        T_closed = dh_transform(theta, d, a, alpha)
        T_elem = dh_transform_elementary(theta, d, a, alpha)
        assert np.allclose(T_closed, T_elem), (
            f"Mismatch for theta={theta}, d={d}, a={a}, alpha={alpha}"
        )


def test_dh_transform_pure_translation():
    """Zero rotation theta=0, alpha=0: only translation."""
    T = dh_transform(0.0, 5.0, 10.0, 0.0)
    expected = np.array(
        [[1.0, 0.0, 0.0, 10.0],
         [0.0, 1.0, 0.0, 0.0],
         [0.0, 0.0, 1.0, 5.0],
         [0.0, 0.0, 0.0, 1.0]],
        dtype=np.float64,
    )
    assert np.allclose(T, expected)


def test_dh_transform_pure_z_rotation():
    """Zero a, d, alpha: pure rotation about Z."""
    T = dh_transform(90.0, 0.0, 0.0, 0.0)
    expected = np.array(
        [[0.0, -1.0, 0.0, 0.0],
         [1.0,  0.0, 0.0, 0.0],
         [0.0,  0.0, 1.0, 0.0],
         [0.0,  0.0, 0.0, 1.0]],
        dtype=np.float64,
    )
    assert np.allclose(T, expected)


def test_dh_transform_alpha_90():
    """alpha=90 degrees rotates the next joint's axis."""
    T = dh_transform(0.0, 0.0, 0.0, 90.0)
    expected = np.array(
        [[1.0, 0.0,  0.0, 0.0],
         [0.0, 0.0, -1.0, 0.0],
         [0.0, 1.0,  0.0, 0.0],
         [0.0, 0.0,  0.0, 1.0]],
        dtype=np.float64,
    )
    assert np.allclose(T, expected)


# ── Forward kinematics tests ────────────────────────────────────────────


class TestTwoJointPlanarXY:
    """Tests against the closed-form 2-DOF planar FK formula.

    Both joints with axis="z" (alpha=0) rotate about world Z, producing
    planar motion in the X-Y plane.  The classic closed form is::

        x = L1*cos(theta1) + L2*cos(theta1 + theta2)
        y = L1*sin(theta1) + L2*sin(theta1 + theta2)
        z = 0

    This is the standard 2-link robot arm formula (compare math doc §4.2
    which describes the X-Z-plane variant; the X-Y-plane result differs
    only in which coordinate carries the sine terms).
    """

    L1 = 10.0
    L2 = 8.0

    @staticmethod
    def _closed_form(theta1_deg: float, theta2_deg: float) -> tuple[float, float, float]:
        t1 = math.radians(theta1_deg)
        t2 = math.radians(theta2_deg)
        x = TestTwoJointPlanarXY.L1 * math.cos(t1) + TestTwoJointPlanarXY.L2 * math.cos(t1 + t2)
        y = TestTwoJointPlanarXY.L1 * math.sin(t1) + TestTwoJointPlanarXY.L2 * math.sin(t1 + t2)
        z = 0.0
        return (x, y, z)

    def test_worked_example(self):
        """Concrete check: theta1=30 deg, theta2=45 deg.
        x = 10*cos30 + 8*cos75 = 10.731
        y = 10*sin30 + 8*sin75 = 12.727
        z = 0
        """
        dh = [(30.0, 0.0, self.L1, 0.0), (45.0, 0.0, self.L2, 0.0)]
        ee = get_end_effector_position(dh)
        expected = self._closed_form(30.0, 45.0)
        assert np.isclose(ee[0], expected[0], atol=1e-4)
        assert np.isclose(ee[1], expected[1], atol=1e-4)
        assert np.isclose(ee[2], expected[2], atol=1e-4)

    @pytest.mark.parametrize("t1,t2", [
        (0, 0), (0, 90), (90, 0), (90, 90),
        (30, 45), (-30, 45), (30, -45),
        (180, 0), (0, 180), (-90, -90),
        (45, 45), (10, 20), (170, 10),
    ])
    def test_angle_grid(self, t1: float, t2: float):
        """Verify DH FK matches closed form across many angle pairs."""
        dh = [(t1, 0.0, self.L1, 0.0), (t2, 0.0, self.L2, 0.0)]
        ee = get_end_effector_position(dh)
        expected = self._closed_form(t1, t2)
        assert np.isclose(ee[0], expected[0], atol=1e-4), f"theta1={t1}, theta2={t2}"
        assert np.isclose(ee[1], expected[1], atol=1e-4), f"theta1={t1}, theta2={t2}"
        assert np.isclose(ee[2], expected[2], atol=1e-4), f"theta1={t1}, theta2={t2}"


def test_single_joint():
    """Single joint at theta=0: end effector at (link_length, 0, 0).
       With link_offset=d, also has Z=d."""
    dh = [(0.0, 5.0, 10.0, 0.0)]
    ee = get_end_effector_position(dh)
    assert np.isclose(ee[0], 10.0, atol=1e-6)
    assert np.isclose(ee[1], 0.0, atol=1e-6)
    assert np.isclose(ee[2], 5.0, atol=1e-6)


def test_single_joint_rotated():
    """Single joint at theta=90 deg with a=10: end effector at (0, 10, d)."""
    dh = [(90.0, 0.0, 10.0, 0.0)]
    ee = get_end_effector_position(dh)
    assert np.isclose(ee[0], 0.0, atol=1e-6)
    assert np.isclose(ee[1], 10.0, atol=1e-6)
    assert np.isclose(ee[2], 0.0, atol=1e-6)


def test_zero_links():
    """All zero-length links: end effector always at origin regardless of angles."""
    dh = [(45.0, 0.0, 0.0, 0.0), (-30.0, 0.0, 0.0, 0.0), (90.0, 0.0, 0.0, 0.0)]
    ee = get_end_effector_position(dh)
    assert np.isclose(ee[0], 0.0, atol=1e-6)
    assert np.isclose(ee[1], 0.0, atol=1e-6)
    assert np.isclose(ee[2], 0.0, atol=1e-6)


def test_intermediate_positions():
    """forward_kinematics returns n+1 positions, with the last == end effector."""
    dh = [(0.0, 0.0, 5.0, 0.0), (0.0, 0.0, 3.0, 0.0)]
    positions = forward_kinematics(dh)
    assert len(positions) == 2
    assert np.isclose(positions[0][0], 5.0, atol=1e-6)
    assert np.isclose(positions[1][0], 8.0, atol=1e-6)


def test_three_joint_out_of_plane():
    """3-DOF with alpha1=90 deg: verify rotating j1 moves end effector out of X-Y plane."""
    dh = [
        (0.0, 2.0, 0.0, 90.0),   # base: alpha=90
        (45.0, 0.0, 15.0, 0.0),  # shoulder
        (45.0, 0.0, 10.0, 0.0),  # elbow
    ]
    ee_zero = get_end_effector_position(dh)

    dh2 = list(dh)
    dh2[0] = (90.0, 2.0, 0.0, 90.0)
    ee_rot = get_end_effector_position(dh2)

    # End effector should have moved — not all components equal.
    diffs = [abs(ee_zero[i] - ee_rot[i]) for i in range(3)]
    assert sum(diffs) > 1e-4, "Expected end effector to move, but it didn't"

    # Distance from origin should be preserved (rotation is distance-preserving).
    dist_zero = math.sqrt(ee_zero[0]**2 + ee_zero[1]**2 + ee_zero[2]**2)
    dist_rot = math.sqrt(ee_rot[0]**2 + ee_rot[1]**2 + ee_rot[2]**2)
    assert np.isclose(dist_zero, dist_rot, atol=1e-4)


def test_projection_front():
    """front_view drops Y."""
    pts = [(1.0, 2.0, 3.0), (4.0, 5.0, 6.0)]
    result = project_front(pts)
    assert result == [(1.0, 3.0), (4.0, 6.0)]


def test_projection_top():
    """top_view drops Z."""
    pts = [(1.0, 2.0, 3.0), (4.0, 5.0, 6.0)]
    result = project_top(pts)
    assert result == [(1.0, 2.0), (4.0, 5.0)]


def test_total_reach():
    assert total_reach([10.0, 8.0, 0.0]) == 18.0
    assert total_reach([]) == 0.0
    assert total_reach([0.0, 0.0, 0.0]) == 0.0
