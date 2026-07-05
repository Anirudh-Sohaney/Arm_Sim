"""Shared constants for the armsim package.

All tunable defaults live here so they are changed in exactly one place.
"""

# ── Motion defaults ──────────────────────────────────────────────────────
DEFAULT_VELOCITY_DEG_S: float = 30.0
"""Default angular velocity for joints in degrees per second."""

DEFAULT_TICK_RATE_HZ: float = 30.0
"""Default simulation update frequency in Hz."""

# ── Tolerance ─────────────────────────────────────────────────────────────
ANGLE_EPSILON_DEG: float = 1e-3
"""Angles closer than this are considered equal for idle detection."""

# ── Joint angle range defaults ────────────────────────────────────────────
DEFAULT_ANGLE_MIN: float = -180.0
DEFAULT_ANGLE_MAX: float = 180.0

# ── Axis preset → DH alpha mapping ───────────────────────────────────────
AXIS_TO_DH_ALPHA: dict[str, float] = {
    "z": 0.0,
    "pitch": 90.0,
    "yaw": -90.0,
}
"""Mapping from user-friendly axis preset strings to DH alpha values in degrees.

Per 02_MATHEMATICS_KINEMATICS.md §5:
- "z"     → alpha =   0°  (default revolute about local Z)
- "pitch" → alpha =  90°  (perpendicular "up/down" bending)
- "yaw"   → alpha = -90°  (perpendicular "left/right" bending)
"""

VALID_AXES: frozenset[str] = frozenset(AXIS_TO_DH_ALPHA.keys())
"""Set of valid axis preset strings."""

# ── Config schema version ─────────────────────────────────────────────────
SUPPORTED_SCHEMA_VERSION: int = 1
"""Config schema version this build of armsim supports."""
