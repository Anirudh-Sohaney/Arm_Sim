"""Thread-safe shared-state container for the simulation.

Holds the latest angles, motion status, and computed 3D positions.
Both the simulation thread (writer) and the server/control thread
(readers) touch this.  Reads return an immutable snapshot so callers
can't accidentally mutate shared state.
"""

from __future__ import annotations

import threading
import time
from dataclasses import dataclass


@dataclass(frozen=True)
class JointSnapshot:
    """Immutable point-in-time snapshot of one joint's state."""

    name: str
    angle: float
    target_angle: float
    is_moving: bool
    position: tuple[float, float, float]


@dataclass(frozen=True)
class ArmState:
    """Immutable snapshot of the entire arm at one simulation tick."""

    timestamp: float
    joints: tuple[JointSnapshot, ...]
    end_effector_position: tuple[float, float, float]
    front_view: tuple[tuple[float, float], ...]
    top_view: tuple[tuple[float, float], ...]


class SharedState:
    """Thread-safe container for the simulation's live state.

    Only the simulation thread writes; all other threads read via
    :meth:`get_snapshot`.
    """

    def __init__(self) -> None:
        self._lock = threading.Lock()

        # Written under lock by simulation thread.
        self._angles: list[float] = []
        self._targets: list[float] = []
        self._moving: list[bool] = []
        self._positions_3d: list[tuple[float, float, float]] = []
        self._end_effector: tuple[float, float, float] = (0.0, 0.0, 0.0)
        self._front_view: list[tuple[float, float]] = []
        self._top_view: list[tuple[float, float]] = []
        self._tick: int = 0
        self._timestamp: float = 0.0
        self._joint_names: list[str] = []

        # Error tracking.
        self._has_error: bool = False
        self._error_message: str = ""
        self._consecutive_tick_failures: int = 0
        self._max_consecutive_failures: int = 5

    def set_joint_names(self, names: list[str]) -> None:
        """Configure joint names (called once during Arm setup)."""
        with self._lock:
            self._joint_names = list(names)
            n = len(names)
            self._angles = [0.0] * n
            self._targets = [0.0] * n
            self._moving = [False] * n
            self._positions_3d = [(0.0, 0.0, 0.0)] * n

    def update(
        self,
        tick: int,
        angles: list[float],
        targets: list[float],
        moving: list[bool],
        positions_3d: list[tuple[float, float, float]],
        end_effector: tuple[float, float, float],
        front_view: list[tuple[float, float]],
        top_view: list[tuple[float, float]],
    ) -> None:
        """Atomically write a new simulation tick's state."""
        with self._lock:
            self._tick = tick
            self._timestamp = time.time()
            self._angles = list(angles)
            self._targets = list(targets)
            self._moving = list(moving)
            self._positions_3d = list(positions_3d)
            self._end_effector = end_effector
            self._front_view = list(front_view)
            self._top_view = list(top_view)
            self._consecutive_tick_failures = 0

    def record_tick_failure(self, error_msg: str) -> bool:
        """Record a tick failure.  Returns True if the error-count
        threshold is exceeded and the simulation should stop."""
        with self._lock:
            self._consecutive_tick_failures += 1
            if self._consecutive_tick_failures >= self._max_consecutive_failures:
                self._has_error = True
                self._error_message = error_msg
                return True
            return False

    @property
    def has_error(self) -> bool:
        with self._lock:
            return self._has_error

    @property
    def error_message(self) -> str:
        with self._lock:
            return self._error_message

    def get_snapshot(self) -> ArmState:
        """Return an immutable snapshot of the current state."""
        with self._lock:
            joints = tuple(
                JointSnapshot(
                    name=self._joint_names[i],
                    angle=self._angles[i],
                    target_angle=self._targets[i],
                    is_moving=self._moving[i],
                    position=(
                        self._positions_3d[i]
                        if i < len(self._positions_3d)
                        else (0.0, 0.0, 0.0)
                    ),
                )
                for i in range(len(self._joint_names))
            )
            return ArmState(
                timestamp=self._timestamp,
                joints=joints,
                end_effector_position=self._end_effector,
                front_view=tuple(self._front_view),
                top_view=tuple(self._top_view),
            )

    def get_end_effector_position(self) -> tuple[float, float, float]:
        """Thread-safe read of the latest end-effector position."""
        with self._lock:
            return self._end_effector

    def get_joint_positions(self) -> list[tuple[float, float, float]]:
        """Thread-safe read of all joint 3D positions."""
        with self._lock:
            return list(self._positions_3d)

    def get_angles(self) -> list[float]:
        with self._lock:
            return list(self._angles)

    def get_targets(self) -> list[float]:
        with self._lock:
            return list(self._targets)

    def get_moving(self) -> list[bool]:
        with self._lock:
            return list(self._moving)

    def get_tick(self) -> int:
        with self._lock:
            return self._tick

    def set_target(self, index: int, target: float) -> None:
        with self._lock:
            self._targets[index] = target

    def set_angle(self, index: int, angle: float) -> None:
        with self._lock:
            self._angles[index] = angle

    def set_moving(self, index: int, moving: bool) -> None:
        with self._lock:
            self._moving[index] = moving
