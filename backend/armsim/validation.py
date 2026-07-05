"""Shared validation helpers used identically by config.py and joint.py/arm.py.

Every validation function raises :class:`InvalidConfigError` with a message
that identifies the offending joint and field, never a generic message.
"""

from __future__ import annotations

from armsim.errors import InvalidConfigError


def validate_angle_range(
    name: str,
    angle_min: float,
    angle_max: float,
) -> None:
    """Raise InvalidConfigError if angle_min > angle_max."""
    if angle_min > angle_max:
        raise InvalidConfigError(
            f"joint '{name}': angle_min ({angle_min}) must be <= angle_max ({angle_max})"
        )


def validate_initial_angle(
    name: str,
    initial_angle: float,
    angle_min: float,
    angle_max: float,
) -> None:
    """Raise InvalidConfigError if initial_angle is outside [angle_min, angle_max]."""
    if not (angle_min <= initial_angle <= angle_max):
        raise InvalidConfigError(
            f"joint '{name}': initial_angle {initial_angle} is outside "
            f"angle_min/max range [{angle_min}, {angle_max}]"
        )


def validate_link_length(name: str, link_length: float) -> None:
    """Raise InvalidConfigError if link_length is negative."""
    if link_length < 0:
        raise InvalidConfigError(
            f"joint '{name}': link_length must be >= 0, got {link_length}"
        )


def validate_link_offset(name: str, link_offset: float) -> None:
    """Raise InvalidConfigError if link_offset is negative."""
    if link_offset < 0:
        raise InvalidConfigError(
            f"joint '{name}': link_offset must be >= 0, got {link_offset}"
        )


def validate_velocity(name: str, velocity: float) -> None:
    """Raise InvalidConfigError if velocity is not positive."""
    if velocity <= 0:
        raise InvalidConfigError(
            f"joint '{name}': velocity must be > 0, got {velocity}"
        )


def validate_name(name: str) -> None:
    """Raise InvalidConfigError if name is empty or has leading/trailing whitespace."""
    if not name or not isinstance(name, str):
        raise InvalidConfigError("joint name must be a non-empty string")
    if name != name.strip():
        raise InvalidConfigError(
            f"joint name '{name}' has leading or trailing whitespace"
        )


def validate_unique_names(joint_names: list[str]) -> None:
    """Raise DuplicateJointNameError if any name appears more than once."""
    from armsim.errors import DuplicateJointNameError

    seen: set[str] = set()
    for n in joint_names:
        if n in seen:
            raise DuplicateJointNameError(n)
        seen.add(n)


def validate_tick_rate(tick_rate_hz: float) -> None:
    """Raise InvalidConfigError if tick_rate_hz is not positive."""
    if tick_rate_hz <= 0:
        raise InvalidConfigError(
            f"tick_rate_hz must be > 0, got {tick_rate_hz}"
        )


def validate_axis(name: str, axis: str) -> None:
    """Raise InvalidConfigError if axis is not a valid preset string."""
    from armsim.constants import VALID_AXES

    if axis not in VALID_AXES:
        raise InvalidConfigError(
            f"joint '{name}': axis must be one of {sorted(VALID_AXES)}, got '{axis}'"
        )


def validate_non_empty_joints(joints: list) -> None:
    """Raise InvalidConfigError if the joints list is empty."""
    if not joints:
        raise InvalidConfigError("At least one joint is required")
