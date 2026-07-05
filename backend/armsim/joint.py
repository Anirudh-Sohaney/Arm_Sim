"""Joint class: a single revolute degree of freedom in the kinematic chain.

Holds DH parameters, angle limits, current/target angle, velocity,
and motion state.  Must not know anything about other joints, the
server, or rendering — purely a data + motion unit.
"""

from __future__ import annotations

import math
import threading
from typing import TYPE_CHECKING

from armsim.constants import (
    ANGLE_EPSILON_DEG,
    AXIS_TO_DH_ALPHA,
    DEFAULT_ANGLE_MAX,
    DEFAULT_ANGLE_MIN,
    DEFAULT_VELOCITY_DEG_S,
    VALID_AXES,
)
from armsim.errors import InvalidConfigError, JointLimitError
from armsim.validation import (
    validate_angle_range,
    validate_initial_angle,
    validate_link_length,
    validate_link_offset,
    validate_name,
    validate_velocity,
)

if TYPE_CHECKING:
    from armsim.state import SharedState


class Joint:
    """A single revolute (rotating) degree of freedom in the arm's
    kinematic chain.

    Parameters
    ----------
    name : str
        Unique human-readable identifier.
    axis : str
        Rotation-axis preset: ``"z"``, ``"pitch"``, or ``"yaw"``.
    link_length : float
        DH ``a`` — length of the limb segment attached to this joint.
    link_offset : float
        DH ``d`` — offset along the previous joint's Z axis.
    dh_alpha : float | None
        Explicit DH α override; wins over ``axis`` if provided.
    angle_min / angle_max : float
        Joint rotation limits in degrees.
    initial_angle : float
        Starting angle in degrees.
    default_velocity : float
        Angular speed in deg/s for moves that don't specify a velocity.
    """

    def __init__(
        self,
        name: str,
        axis: str = "z",
        link_length: float = 0.0,
        link_offset: float = 0.0,
        dh_alpha: float | None = None,
        angle_min: float = DEFAULT_ANGLE_MIN,
        angle_max: float = DEFAULT_ANGLE_MAX,
        initial_angle: float = 0.0,
        default_velocity: float = DEFAULT_VELOCITY_DEG_S,
    ) -> None:
        # ── Validate name first so error messages can reference it ───────
        validate_name(name)

        # Resolve DH alpha
        if dh_alpha is not None:
            self._dh_alpha: float = dh_alpha
        elif axis not in VALID_AXES:
            raise InvalidConfigError(
                f"joint '{name}': axis must be one of "
                f"{sorted(VALID_AXES)}, got '{axis}'"
            )
        else:
            self._dh_alpha = AXIS_TO_DH_ALPHA[axis]

        # ── Validate ranges ──────────────────────────────────────────────
        validate_angle_range(name, angle_min, angle_max)
        validate_initial_angle(name, initial_angle, angle_min, angle_max)
        validate_link_length(name, link_length)
        validate_link_offset(name, link_offset)
        validate_velocity(name, default_velocity)

        # ── Store ────────────────────────────────────────────────────────
        self.name: str = name
        self.axis: str = axis
        self.link_length: float = link_length  # DH a
        self.link_offset: float = link_offset  # DH d
        self.angle_min: float = angle_min
        self.angle_max: float = angle_max
        self.default_velocity: float = default_velocity

        # Live state (populated by simulation).
        self._current_angle: float = float(initial_angle)
        self._target_angle: float = float(initial_angle)
        self._velocity: float = default_velocity

        # Per spec: idle when current == target within epsilon.
        self._idle_event = threading.Event()
        self._idle_event.set()  # starts idle since initial == target

        # Reference to shared state (set by Arm after construction).
        self._state: SharedState | None = None
        self._index: int = -1

    # ── Internal wiring (called by Arm) ──────────────────────────────────

    def _wire(self, state: SharedState, index: int) -> None:
        """Connect this joint to the shared-state container."""
        self._state = state
        self._index = index

    # ── Public API ───────────────────────────────────────────────────────

    def set_angle(
        self,
        angle: float,
        velocity: float | None = None,
        blocking: bool = False,
        timeout: float | None = None,
    ) -> None:
        """Command this joint toward a new target angle.

        Parameters
        ----------
        angle : float
            Target angle in degrees.  Must be within [angle_min, angle_max].
        velocity : float | None
            Angular speed in deg/s.  Defaults to the joint's default_velocity.
        blocking : bool
            If True, block until the joint reaches the target.
        timeout : float | None
            Max seconds to block; only meaningful when blocking=True.

        Raises
        ------
        JointLimitError
            If ``angle`` is outside this joint's configured range.
        TimeoutError
            If blocking=True and the timeout elapses before completion.
        """
        if not (self.angle_min <= angle <= self.angle_max):
            raise JointLimitError(
                self.name, angle, self.angle_min, self.angle_max
            )

        # Reset idle event *before* writing target so the simulation
        # thread can't transition to idle between our write and our
        # event clear (the event is read *after* angle comparison in
        # the tick loop, so it's safe regardless, but this ordering is
        # more obviously correct).
        if abs(angle - self._current_angle) > ANGLE_EPSILON_DEG:
            self._idle_event.clear()

        if velocity is not None:
            if velocity <= 0:
                raise InvalidConfigError(
                    f"joint '{self.name}': velocity must be > 0, got {velocity}"
                )
            self._velocity = velocity

        self._target_angle = float(angle)

        # Push to shared state if wired.
        if self._state is not None and self._index >= 0:
            self._state.set_target(self._index, angle)
            self._state.set_moving(
                self._index,
                abs(angle - self._current_angle) > ANGLE_EPSILON_DEG,
            )

        if blocking:
            self.wait(timeout=timeout)

    def wait(self, timeout: float | None = None) -> None:
        """Block until this joint reaches its target angle.

        Uses an efficient ``threading.Event``, not busy-polling.

        Parameters
        ----------
        timeout : float | None
            Max seconds to wait.

        Raises
        ------
        TimeoutError
            If ``timeout`` elapses before the joint reaches its target.
        """
        if not self._idle_event.wait(timeout=timeout):
            raise TimeoutError(
                f"joint '{self.name}' did not reach target "
                f"{self._target_angle} within {timeout}s"
            )

    def stop(self) -> None:
        """Freeze the joint at its current angle.

        Sets target_angle = current_angle, transitioning to idle on
        the next tick.
        """
        self._target_angle = self._current_angle
        self._idle_event.set()
        if self._state is not None and self._index >= 0:
            self._state.set_target(self._index, self._current_angle)
            self._state.set_moving(self._index, False)

    def get_angle(self) -> float:
        """Return the current angle in degrees."""
        return self._current_angle

    def get_target_angle(self) -> float:
        """Return the current target angle in degrees."""
        return self._target_angle

    def is_moving(self) -> bool:
        """Return True if the joint is still travelling toward its target."""
        return not self._idle_event.is_set()

    def get_dh_parameters(self) -> tuple[float, float, float, float]:
        """Return ``(theta, d, a, alpha)`` in degrees/units."""
        return (self._current_angle, self.link_offset, self.link_length, self._dh_alpha)

    def get_current_dh(self) -> tuple[float, float, float, float]:
        """Return DH tuple for the *current* angle (used by kinematics)."""
        return (self._current_angle, self.link_offset, self.link_length, self._dh_alpha)

    # ── Simulation-thread interface (called from simulation.py) ──────────

    def _advance(self, dt: float) -> None:
        """Advance this joint's angle toward its target for one tick.

        Called only from the simulation thread.

        Parameters
        ----------
        dt : float
            Actual wall-clock elapsed seconds since the last tick.
        """
        target = self._target_angle
        current = self._current_angle

        if abs(target - current) <= ANGLE_EPSILON_DEG:
            # Already at target → stay idle.
            if not self._idle_event.is_set():
                self._idle_event.set()
            return

        direction = 1.0 if target > current else -1.0
        max_step = self._velocity * dt
        delta = min(abs(target - current), max_step)
        new_angle = current + direction * delta

        # Clamp to limits (defense in depth — target was already
        # validated, but this guards against floating-point drift).
        new_angle = max(self.angle_min, min(self.angle_max, new_angle))

        self._current_angle = new_angle

        # Push to shared state.
        if self._state is not None and self._index >= 0:
            self._state.set_angle(self._index, new_angle)

        # Check idle transition.
        if abs(target - new_angle) <= ANGLE_EPSILON_DEG:
            self._current_angle = target  # snap exactly to target
            self._idle_event.set()
            if self._state is not None and self._index >= 0:
                self._state.set_angle(self._index, target)
                self._state.set_moving(self._index, False)

    def __repr__(self) -> str:
        moving = "moving" if self.is_moving() else "idle"
        return (
            f"Joint(name='{self.name}', "
            f"angle={self._current_angle:.1f}/{self._target_angle:.1f} target, "
            f"{moving})"
        )
