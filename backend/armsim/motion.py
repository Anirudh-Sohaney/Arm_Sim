"""Motion model: velocity-and-time-based angle interpolation.

Contains the single canonical implementation of the velocity-ramp
formula (math doc §6).  No other module may reimplement angle
interpolation.
"""

from __future__ import annotations

from armsim.constants import ANGLE_EPSILON_DEG


def compute_angle_step(
    current_angle: float,
    target_angle: float,
    velocity_deg_s: float,
    dt: float,
    angle_min: float,
    angle_max: float,
) -> tuple[float, bool]:
    """Compute the new angle after one tick of motion.

    Uses linear ramping: the joint moves toward its target at a
    constant velocity for the elapsed ``dt`` seconds, then clamps
    at the target and joint limits.

    This is the **single source of truth** for motion — no other
    function should implement angle interpolation.

    Parameters
    ----------
    current_angle : float
        Current angle in degrees.
    target_angle : float
        Desired angle in degrees.
    velocity_deg_s : float
        Angular speed in deg/s.
    dt : float
        Elapsed wall-clock seconds since last tick.
    angle_min / angle_max : float
        Joint rotation limits.

    Returns
    -------
    tuple[float, bool]
        ``(new_angle, is_idle)`` — the new angle and whether the joint
        has reached (or overshot) the target within epsilon.
    """
    diff = target_angle - current_angle
    if abs(diff) <= ANGLE_EPSILON_DEG:
        return current_angle, True

    sign = 1.0 if diff > 0 else -1.0
    max_step = velocity_deg_s * dt
    step = min(abs(diff), max_step)

    new_angle = current_angle + sign * step

    # Clamp to limits.
    new_angle = max(angle_min, min(angle_max, new_angle))

    # Check idle.
    is_idle = abs(target_angle - new_angle) <= ANGLE_EPSILON_DEG

    return new_angle, is_idle
