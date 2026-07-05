"""Arm class: an ordered chain of Joints + top-level control API.

Owns the full lifecycle: construction validation, start/stop of the
background simulation and server, joint- and arm-level control, and
data access.
"""

from __future__ import annotations

import threading
import time
from typing import TYPE_CHECKING, Literal

from armsim.constants import DEFAULT_TICK_RATE_HZ
from armsim.errors import (
    ArmAlreadyStartedError,
    ArmNotStartedError,
    InvalidConfigError,
    JointLimitError,
)
from armsim.kinematics import (
    forward_kinematics,
    get_end_effector_position,
    project_front,
    project_top,
)
from armsim.validation import validate_non_empty_joints, validate_unique_names

if TYPE_CHECKING:
    from armsim.joint import Joint
    from armsim.state import ArmState


class Arm:
    """An ordered kinematic chain of revolute joints.

    Parameters
    ----------
    joints : list of Joint
        Ordered base → end effector.  Order is semantically significant —
        it defines the DH chain from the fixed origin.
    tick_rate_hz : float
        Simulation update frequency (default 30).
    """

    def __init__(
        self,
        joints: list[Joint],
        tick_rate_hz: float = DEFAULT_TICK_RATE_HZ,
    ) -> None:
        # ── Validate ─────────────────────────────────────────────────
        validate_non_empty_joints(joints)
        validate_unique_names([j.name for j in joints])

        self._joint_list: list[Joint] = list(joints)
        self.joints: dict[str, Joint] = {j.name: j for j in joints}
        self._initial_angles: dict[str, float] = {j.name: j.get_angle() for j in joints}
        self.tick_rate_hz: float = float(tick_rate_hz)

        # Wire each joint to know about the shared state.
        from armsim.state import SharedState

        self._state = SharedState()
        self._state.set_joint_names([j.name for j in self._joint_list])

        for i, joint in enumerate(self._joint_list):
            joint._wire(self._state, i)

        # Lifecycle flags.
        self._started: bool = False
        self._simulation_thread: threading.Thread | None = None
        self._server_thread: threading.Thread | None = None
        self._stop_event = threading.Event()

        # Server reference (populated by start()).
        self._uvicorn_server: object = None  # type: ignore[assignment]

        # Recorder reference.
        self._recorder: object = None  # type: ignore[assignment]

    # ── Lifecycle ────────────────────────────────────────────────────

    def start(
        self,
        mode: Literal["local", "lan"] = "local",
        port: int = 8080,
        tick_rate_hz: float | None = None,
        record_to: str | None = None,
    ) -> None:
        """Start the background simulation loop and web server.

        Parameters
        ----------
        mode : str
            ``"local"`` binds 127.0.0.1 only; ``"lan"`` binds 0.0.0.0.
        port : int
            TCP port for the HTTP/WebSocket server.
        tick_rate_hz : float | None
            Override the tick rate set at construction time.
        record_to : str | None
            If given, enables trajectory logging to the named file.

        Raises
        ------
        ArmAlreadyStartedError
            If called on an already-running Arm.
        OSError
            If the port is already in use.
        """
        if self._started:
            raise ArmAlreadyStartedError()

        if tick_rate_hz is not None:
            self.tick_rate_hz = float(tick_rate_hz)

        # Start recorder if requested.
        if record_to is not None:
            from armsim.recorder import Recorder

            self._recorder = Recorder(record_to)

        self._stop_event.clear()
        self._started = True

        # Launch simulation thread.
        self._simulation_thread = threading.Thread(
            target=self._run_simulation,
            name="armsim-simulation",
            daemon=True,
        )
        self._simulation_thread.start()

        # Launch server thread.
        self._server_thread = threading.Thread(
            target=self._run_server,
            args=(mode, port),
            name="armsim-server",
            daemon=True,
        )
        self._server_thread.start()

    def stop_server(self) -> None:
        """Gracefully shut down the simulation loop and web server.

        Closes WebSocket connections, flushes/closes the recorder,
        and joins background threads before returning.
        """
        self._stop_event.set()

        if self._simulation_thread is not None:
            self._simulation_thread.join(timeout=5.0)

        # Shut down the uvicorn server.
        if hasattr(self, "_uvicorn_server") and self._uvicorn_server is not None:
            try:
                self._uvicorn_server.should_exit = True
            except Exception:
                pass

        if self._server_thread is not None:
            self._server_thread.join(timeout=5.0)

        # Close recorder.
        if self._recorder is not None:
            try:
                self._recorder.close()
            except Exception:
                pass

        self._started = False

    # ── Simulation loop (runs on background thread) ──────────────────

    def _run_simulation(self) -> None:
        """Background tick loop: measure dt, advance motion, compute FK,
        push state."""
        import logging

        from armsim.logging_utils import get_logger

        logger = get_logger("armsim.simulation")
        interval = 1.0 / self.tick_rate_hz
        tick = 0
        last_tick_time = time.perf_counter()

        while not self._stop_event.is_set():
            now = time.perf_counter()
            dt = now - last_tick_time
            last_tick_time = now

            try:
                # Advance every joint's angle.
                for joint in self._joint_list:
                    joint._advance(dt)

                # Recompute FK.
                dh_list = [j.get_current_dh() for j in self._joint_list]
                positions_3d = forward_kinematics(dh_list)
                ee_pos = positions_3d[-1] if positions_3d else (0.0, 0.0, 0.0)

                front_view = project_front(positions_3d)
                top_view = project_top(positions_3d)

                # Write shared state.
                self._state.update(
                    tick=tick,
                    angles=[j.get_angle() for j in self._joint_list],
                    targets=[j.get_target_angle() for j in self._joint_list],
                    moving=[j.is_moving() for j in self._joint_list],
                    positions_3d=positions_3d,
                    end_effector=ee_pos,
                    front_view=front_view,
                    top_view=top_view,
                )

                # Record trajectory if enabled.
                if self._recorder is not None:
                    self._recorder.record(
                        tick,
                        [j.get_angle() for j in self._joint_list],
                        ee_pos,
                    )

                tick += 1

            except Exception as exc:
                logger.error("Tick %d failed: %s", tick, exc, exc_info=True)
                terminal = self._state.record_tick_failure(str(exc))
                if terminal:
                    logger.critical(
                        "Simulation stopped after %d consecutive failures.",
                        self._state._consecutive_tick_failures,
                    )
                    self._stop_event.set()
                    return

            # Sleep for the remainder of the tick interval.
            elapsed = time.perf_counter() - last_tick_time
            remaining = interval - elapsed
            if remaining > 0:
                self._stop_event.wait(timeout=remaining)

    # ── Server (runs on background thread) ───────────────────────────

    def _run_server(self, mode: str, port: int) -> None:
        """Launch the FastAPI + uvicorn server on a background thread."""
        import sys

        from armsim.server import create_app

        host = "127.0.0.1" if mode == "local" else "0.0.0.0"
        app = create_app(self._state, self, mode, port)

        import uvicorn

        config = uvicorn.Config(
            app,
            host=host,
            port=port,
            log_level="info",
            lifespan="off",
        )
        self._uvicorn_server = uvicorn.Server(config)
        try:
            self._uvicorn_server.run()
        except Exception as exc:
            print(f"[armsim-server] ERROR: {exc}", file=sys.stderr, flush=True)

    # ── Control API ──────────────────────────────────────────────────

    def _ensure_started(self, method: str = "") -> None:
        if not self._started:
            raise ArmNotStartedError(method)

    def set_angles(
        self,
        targets: dict[str, float],
        velocity: float | None = None,
        blocking: bool = False,
        timeout: float | None = None,
    ) -> None:
        """Set multiple joints' target angles at once (parallel movement).

        Validates **all** requested angles before applying any
        (all-or-nothing semantics).

        Parameters
        ----------
        targets : dict
            Mapping ``{joint_name: target_angle}``.
        velocity : float | None
            Angular speed override for all joints in this call.
        blocking : bool
            If True, block until all joints reach their targets.
        timeout : float | None
            Max seconds to block.

        Raises
        ------
        JointLimitError
            If any requested angle is out of range.
        KeyError
            If a joint name in ``targets`` is not in this arm.
        """
        # Validate all before applying any (validation happens before
        # the started check so that config errors are surfaced clearly).
        for name, angle in targets.items():
            if name not in self.joints:
                raise KeyError(
                    f"Joint '{name}' not found in arm. "
                    f"Available: {list(self.joints.keys())}"
                )
            j = self.joints[name]
            if not (j.angle_min <= angle <= j.angle_max):
                raise JointLimitError(name, angle, j.angle_min, j.angle_max)

        self._ensure_started("set_angles()")

        # Apply.
        for name, angle in targets.items():
            self.joints[name].set_angle(angle, velocity=velocity, blocking=False)

        if blocking:
            self.wait_until_idle(timeout=timeout)

    def wait_until_idle(self, timeout: float | None = None) -> None:
        """Block until every joint is idle.

        Parameters
        ----------
        timeout : float | None
            Max seconds to wait.

        Raises
        ------
        TimeoutError
            If timeout elapses before all joints reach idle.
        """
        self._ensure_started("wait_until_idle()")
        deadline = time.monotonic() + timeout if timeout is not None else None
        for joint in self._joint_list:
            remaining = None
            if deadline is not None:
                remaining = max(0.0, deadline - time.monotonic())
            joint.wait(timeout=remaining)

    def get_end_effector_position(self) -> tuple[float, float, float]:
        """Return the current (x, y, z) world-space end-effector position.

        Raises
        ------
        ArmNotStartedError
            If called before :meth:`start`.
        """
        self._ensure_started("get_end_effector_position()")
        return self._state.get_end_effector_position()

    def get_joint_positions(self) -> list[tuple[float, float, float]]:
        """Return ordered list of every joint's 3D world-space position.

        Raises
        ------
        ArmNotStartedError
        """
        self._ensure_started("get_joint_positions()")
        return self._state.get_joint_positions()

    def get_state(self) -> ArmState:
        """Return a full immutable snapshot of the current arm state.

        Raises
        ------
        ArmNotStartedError
        """
        self._ensure_started("get_state()")
        return self._state.get_snapshot()

    def reset_to_initial(self) -> None:
        """Set every joint's target back to its originally configured
        initial_angle (non-blocking)."""
        self._ensure_started("reset_to_initial()")
        for name, angle in self._initial_angles.items():
            if name in self.joints:
                self.joints[name].set_angle(angle)

    def __repr__(self) -> str:
        n = len(self._joint_list)
        return f"Arm( joints={n}, started={self._started} )"
