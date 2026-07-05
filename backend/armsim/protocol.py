"""Protocol message builders — single source of truth for JSON message shapes."""

from __future__ import annotations

PROTOCOL_VERSION = 1


def build_config_message(
    arm_name: str,
    tick_rate_hz: float,
    joint_names: list[str],
    link_lengths: list[float],
    link_offsets: list[float],
    angle_mins: list[float],
    angle_maxs: list[float],
    axes: list[str],
    total_reach_val: float,
) -> dict:
    """Build the one-time ``"config"`` message sent on WebSocket connect."""
    joints = [
        {
            "name": joint_names[i],
            "link_length": link_lengths[i],
            "link_offset": link_offsets[i],
            "angle_min": angle_mins[i],
            "angle_max": angle_maxs[i],
            "axis": axes[i],
        }
        for i in range(len(joint_names))
    ]
    return {
        "type": "config",
        "protocol_version": PROTOCOL_VERSION,
        "arm_name": arm_name,
        "tick_rate_hz": tick_rate_hz,
        "joint_count": len(joint_names),
        "total_reach": total_reach_val,
        "joints": joints,
    }


def build_state_message(state, tick: int) -> dict:
    """Build a ``"state"`` message from an :class:`ArmState` snapshot."""
    from armsim.state import ArmState

    joints = [
        {
            "name": j.name,
            "angle": j.angle,
            "target_angle": j.target_angle,
            "is_moving": j.is_moving,
        }
        for j in state.joints
    ]
    positions_3d = [
        {"x": j.position[0], "y": j.position[1], "z": j.position[2]}
        for j in state.joints
    ]
    front_view = [{"x": p[0], "z": p[1]} for p in state.front_view]
    top_view = [{"x": p[0], "y": p[1]} for p in state.top_view]

    return {
        "type": "state",
        "protocol_version": PROTOCOL_VERSION,
        "tick": tick,
        "timestamp": state.timestamp,
        "joints": joints,
        "positions_3d": positions_3d,
        "end_effector": {
            "x": state.end_effector_position[0],
            "y": state.end_effector_position[1],
            "z": state.end_effector_position[2],
        },
        "front_view": front_view,
        "top_view": top_view,
    }


def build_error_message(code: str, message: str, recoverable: bool) -> dict:
    """Build an ``"error"`` message."""
    return {
        "type": "error",
        "protocol_version": PROTOCOL_VERSION,
        "code": code,
        "message": message,
        "recoverable": recoverable,
    }
