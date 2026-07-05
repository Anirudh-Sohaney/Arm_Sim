"""Pure-mathematics forward-kinematics module.

No state, no I/O, no threading — every function is a deterministic pure
function of its inputs.  This module is the heart of the simulator:
everything else (the library API, the config schema, the frontend
rendering) is a wrapper around the math described here.

Convention: **standard (distal) DH** — see §2.1 of the math document.
"""

from __future__ import annotations

import math

import numpy as np


def dh_transform(
    theta_deg: float,
    d: float,
    a: float,
    alpha_deg: float,
) -> np.ndarray:
    """Build the 4×4 DH transformation matrix for a single joint.

    Uses the **standard DH** convention:

        T_i = Rot_z(θ) · Trans_z(d) · Trans_x(a) · Rot_x(α)

    Parameters
    ----------
    theta_deg : float
        Joint angle θ in degrees.
    d : float
        Link offset along previous Z axis.
    a : float
        Link length along X axis.
    alpha_deg : float
        Link twist α in degrees.

    Returns
    -------
    np.ndarray
        A 4×4 homogeneous transformation matrix (float64).
    """
    theta = math.radians(theta_deg)
    alpha = math.radians(alpha_deg)

    ct = math.cos(theta)
    st = math.sin(theta)
    ca = math.cos(alpha)
    sa = math.sin(alpha)

    return np.array(
        [
            [ct, -st * ca, st * sa, a * ct],
            [st, ct * ca, -ct * sa, a * st],
            [0.0, sa, ca, d],
            [0.0, 0.0, 0.0, 1.0],
        ],
        dtype=np.float64,
    )


def dh_transform_elementary(
    theta_deg: float,
    d: float,
    a: float,
    alpha_deg: float,
) -> np.ndarray:
    """Build the DH transform as four chained elementary matrices.

    This is an alternative implementation (easier to unit-test each
    elementary transform independently) that must produce identical
    results to :func:`dh_transform`.

    T_i = Rot_z(θ) · Trans_z(d) · Trans_x(a) · Rot_x(α)
    """
    theta = math.radians(theta_deg)
    alpha = math.radians(alpha_deg)

    ct = math.cos(theta)
    st = math.sin(theta)
    ca = math.cos(alpha)
    sa = math.sin(alpha)

    rot_z = np.array(
        [[ct, -st, 0, 0], [st, ct, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]],
        dtype=np.float64,
    )
    trans_z = np.array(
        [[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, d], [0, 0, 0, 1]],
        dtype=np.float64,
    )
    trans_x = np.array(
        [[1, 0, 0, a], [0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]],
        dtype=np.float64,
    )
    rot_x = np.array(
        [[1, 0, 0, 0], [0, ca, -sa, 0], [0, sa, ca, 0], [0, 0, 0, 1]],
        dtype=np.float64,
    )

    return rot_z @ trans_z @ trans_x @ rot_x


def forward_kinematics(
    dh_params: list[tuple[float, float, float, float]],
) -> list[tuple[float, float, float]]:
    """Compute 3D world-space positions of every joint + end effector.

    Given an ordered list of (θ, d, a, α) tuples (one per joint, in
    degrees/units), returns a list of ``n+1`` 3D points: the position
    of every intermediate joint, plus the end-effector point.

    The computation is incremental (single O(n) pass, cumulative
    matrix product) rather than recomputing each T_0_i from scratch.

    Parameters
    ----------
    dh_params : list of tuple
        Each tuple is ``(theta_deg, d, a, alpha_deg)``.

    Returns
    -------
    list of tuple
        ``[(x0,y0,z0), (x1,y1,z1), ..., (xn,yn,zn)]`` where the last
        point is the end-effector position.
    """
    if not dh_params:
        return [(0.0, 0.0, 0.0)]

    positions: list[tuple[float, float, float]] = []
    T = np.eye(4, dtype=np.float64)

    for theta, d, a, alpha in dh_params:
        T = T @ dh_transform(theta, d, a, alpha)
        positions.append((float(T[0, 3]), float(T[1, 3]), float(T[2, 3])))

    return positions


def get_end_effector_position(
    dh_params: list[tuple[float, float, float, float]],
) -> tuple[float, float, float]:
    """Compute just the end-effector 3D position without collecting
    intermediate joint positions.

    Parameters
    ----------
    dh_params : list of tuple
        Each tuple is ``(theta_deg, d, a, alpha_deg)``.

    Returns
    -------
    tuple
        ``(x, y, z)`` in world coordinates.
    """
    positions = forward_kinematics(dh_params)
    return positions[-1]


def project_front(
    positions: list[tuple[float, float, float]],
) -> list[tuple[float, float]]:
    """Project a list of 3D points onto the X-Z plane (front view).

    The Y axis is dropped — viewer looking along -Y.
    Returns raw world-unit (x, z) pairs with no pixel-space adjustment.
    """
    return [(x, z) for x, _y, z in positions]


def project_top(
    positions: list[tuple[float, float, float]],
) -> list[tuple[float, float]]:
    """Project a list of 3D points onto the X-Y plane (top view).

    The Z axis is dropped — viewer looking along -Z (down).
    Returns raw world-unit (x, y) pairs with no pixel-space adjustment.
    """
    return [(x, y) for x, y, _z in positions]


def total_reach(link_lengths: list[float]) -> float:
    """Return the sum of link lengths.

    This is an upper bound on the end-effector's distance from the
    origin, used purely for frontend auto-scaling.  It is *not* a
    rigorous workspace-boundary computation.

    Parameters
    ----------
    link_lengths : list of float
        The DH ``a`` value for each joint in order.

    Returns
    -------
    float
        Sum of all link lengths.
    """
    return sum(link_lengths)
