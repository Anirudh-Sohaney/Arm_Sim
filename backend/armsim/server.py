"""HTTP + WebSocket server: serves the frontend and streams simulation state.

Runs on its own thread with its own asyncio event loop.
"""

from __future__ import annotations

import asyncio
import json
from pathlib import Path
from typing import TYPE_CHECKING

from fastapi import FastAPI, WebSocket, WebSocketDisconnect
from fastapi.staticfiles import StaticFiles
from starlette.responses import FileResponse

from armsim.protocol import (
    build_config_message,
    build_error_message,
    build_state_message,
)

if TYPE_CHECKING:
    from armsim.state import SharedState


FRONTEND_DIR = Path(__file__).resolve().parent.parent.parent / "frontend"


class ConnectionManager:
    """Manages all active WebSocket connections for broadcasting."""

    def __init__(self) -> None:
        self._connections: list[WebSocket] = []
        self._lock = asyncio.Lock()

    async def connect(self, ws: WebSocket) -> None:
        await ws.accept()
        async with self._lock:
            self._connections.append(ws)

    async def disconnect(self, ws: WebSocket) -> None:
        async with self._lock:
            if ws in self._connections:
                self._connections.remove(ws)

    async def broadcast(self, message: dict) -> None:
        payload = json.dumps(message)
        async with self._lock:
            dead: list[WebSocket] = []
            for ws in self._connections:
                try:
                    await ws.send_text(payload)
                except Exception:
                    dead.append(ws)
            for ws in dead:
                self._connections.remove(ws)

    async def broadcast_error(self, code: str, msg: str, recoverable: bool) -> None:
        await self.broadcast(build_error_message(code, msg, recoverable))


def create_app(
    state: SharedState,
    arm: object,  # Arm instance
    mode: str,
    port: int,
) -> FastAPI:
    """Create and configure the FastAPI application."""
    app = FastAPI(title="Robotic Arm Simulator", docs_url=None, redoc_url=None)

    manager = ConnectionManager()

    # State streaming loop (runs as a background task, started on
    # first WebSocket connection — NOT via @app.on_event which requires
    # the ASGI lifespan protocol, disabled by lifespan="off").
    async def state_streamer() -> None:
        last_tick = None
        while getattr(app, "should_exit", False) is False:
            tick = state.get_tick()
            if last_tick is None or tick != last_tick:
                last_tick = tick
                snapshot = state.get_snapshot()
                msg = build_state_message(snapshot, tick)
                await manager.broadcast(msg)
            await asyncio.sleep(0.01)  # ~100 Hz poll

    app.should_exit = False
    app._streamer_started = False

    # ── HTTP routes ──────────────────────────────────────────────────

    @app.get("/healthz")
    async def healthz() -> dict:
        return {
            "status": "ok",
            "tick_rate_hz": getattr(arm, "tick_rate_hz", 30),
        }

    @app.get("/")
    async def index() -> FileResponse:
        index_path = FRONTEND_DIR / "index.html"
        if index_path.exists():
            return FileResponse(str(index_path))
        return FileResponse(str(index_path), status_code=404)

    # ── WebSocket ────────────────────────────────────────────────────

    @app.websocket("/ws")
    async def ws_endpoint(ws: WebSocket) -> None:
        await manager.connect(ws)

        # Start the state streamer once (can't use @app.on_event with lifespan="off").
        if not getattr(app, "_streamer_started", False):
            app._streamer_started = True
            asyncio.create_task(state_streamer())

        # Send config message immediately.
        try:
            joint_list = arm.joints  # dict[str, Joint]
            joint_names = list(joint_list.keys())
            link_lengths = [j.link_length for j in joint_list.values()]
            link_offsets = [j.link_offset for j in joint_list.values()]
            angle_mins = [j.angle_min for j in joint_list.values()]
            angle_maxs = [j.angle_max for j in joint_list.values()]
            axes = [j.axis for j in joint_list.values()]
            from armsim.kinematics import total_reach

            config_msg = build_config_message(
                arm_name=getattr(arm, "_arm_name", "demo_arm"),
                tick_rate_hz=getattr(arm, "tick_rate_hz", 30),
                joint_names=joint_names,
                link_lengths=link_lengths,
                link_offsets=link_offsets,
                angle_mins=angle_mins,
                angle_maxs=angle_maxs,
                axes=axes,
                total_reach_val=total_reach(link_lengths),
            )
            await ws.send_text(json.dumps(config_msg))

            # Send current state immediately so the frontend has view arrays.
            current_snapshot = state.get_snapshot()
            current_tick = state.get_tick()
            await ws.send_text(
                json.dumps(build_state_message(current_snapshot, current_tick))
            )
        except Exception as exc:
            import logging
            _log = logging.getLogger("armsim.server")
            _log.warning("Failed to send config/state message: %s", exc)

        # Then the state streamer handles ongoing state messages.
        try:
            while getattr(app, "should_exit", False) is False:
                # Keep the connection open; state streamer broadcasts.
                await asyncio.sleep(0.5)
        except WebSocketDisconnect:
            pass
        finally:
            await manager.disconnect(ws)

    # ── Static files (if frontend exists) ────────────────────────────
    if FRONTEND_DIR.exists():
        app.mount("/css", StaticFiles(directory=str(FRONTEND_DIR / "css")), name="css")
        app.mount("/js", StaticFiles(directory=str(FRONTEND_DIR / "js")), name="js")
        app.mount(
            "/assets", StaticFiles(directory=str(FRONTEND_DIR / "assets")), name="assets"
        )

    return app
