"""Config loader: load arm definitions from YAML/JSON files."""

from __future__ import annotations

from pathlib import Path

import yaml

from armsim.constants import SUPPORTED_SCHEMA_VERSION, AXIS_TO_DH_ALPHA
from armsim.errors import InvalidConfigError
from armsim.joint import Joint
from armsim.arm import Arm
from armsim.validation import (
    validate_angle_range,
    validate_axis,
    validate_initial_angle,
    validate_link_length,
    validate_link_offset,
    validate_name,
    validate_non_empty_joints,
    validate_tick_rate,
    validate_unique_names,
    validate_velocity,
)


def load_arm_from_config(path: str) -> Arm:
    """Load a YAML or JSON arm definition file and return a ready-to-use Arm.

    File format auto-detected from extension (``.yaml``/``.yml`` → YAML,
    ``.json`` → JSON).

    Parameters
    ----------
    path : str
        Path to a YAML or JSON config file.

    Returns
    -------
    Arm
        A fully constructed :class:`Arm` ready for ``.start()``.

    Raises
    ------
    InvalidConfigError
        On any validation failure, with a message identifying the
        specific joint and field at fault.
    """
    path_obj = Path(path)
    if not path_obj.exists():
        raise InvalidConfigError(f"Config file not found: {path}")

    suffix = path_obj.suffix.lower()
    if suffix in (".yaml", ".yml"):
        with open(path) as f:
            data = yaml.safe_load(f)
    elif suffix == ".json":
        import json

        with open(path) as f:
            data = json.load(f)
    else:
        raise InvalidConfigError(
            f"Unsupported config file format: '{suffix}'. "
            f"Expected .yaml, .yml, or .json"
        )

    if data is None:
        raise InvalidConfigError("Config file is empty")

    return _parse_config(data)


def _parse_config(data: dict) -> Arm:
    """Parse a loaded config dict into an Arm object."""
    # Schema version.
    schema_version = data.get("schema_version")
    if schema_version is None:
        raise InvalidConfigError(
            "Config missing required field 'schema_version'"
        )
    if schema_version != SUPPORTED_SCHEMA_VERSION:
        raise InvalidConfigError(
            f"Config schema_version {schema_version} is not supported "
            f"by this version of armsim (supports: {SUPPORTED_SCHEMA_VERSION})"
        )

    # Top-level options.
    arm_name = data.get("arm_name")
    tick_rate_hz = data.get("tick_rate_hz", None)
    default_velocity = data.get("default_velocity", None)

    if tick_rate_hz is not None:
        validate_tick_rate(tick_rate_hz)

    if default_velocity is not None and default_velocity <= 0:
        raise InvalidConfigError(
            f"default_velocity must be > 0, got {default_velocity}"
        )

    # Joints.
    raw_joints = data.get("joints", [])
    if not isinstance(raw_joints, list):
        raise InvalidConfigError("'joints' must be a list")
    validate_non_empty_joints(raw_joints)

    # Validate joint names before building.
    joint_names = [j.get("name", "") for j in raw_joints]
    for name in joint_names:
        validate_name(name)
    validate_unique_names(joint_names)

    # Build Joint objects.
    joints: list[Joint] = []
    for jd in raw_joints:
        joint = _build_joint(jd, default_velocity)
        joints.append(joint)

    # Build Arm.
    kwargs = {}
    if tick_rate_hz is not None:
        kwargs["tick_rate_hz"] = tick_rate_hz
    return Arm(joints, **kwargs)


def _build_joint(data: dict, global_default_velocity: float | None) -> Joint:
    """Build a single Joint from a parsed config dict."""
    name = data["name"]
    axis = data.get("axis", "z")
    link_length = data.get("link_length", 0.0)
    link_offset = data.get("link_offset", 0.0)
    dh_alpha = data.get("dh_alpha", None)
    angle_min = data.get("angle_min", -180.0)
    angle_max = data.get("angle_max", 180.0)
    initial_angle = data.get("initial_angle", 0.0)
    velocity = data.get("velocity", None)

    # Validate axis.
    validate_axis(name, axis)

    # Determine final velocity.
    final_velocity = velocity if velocity is not None else (
        global_default_velocity if global_default_velocity is not None else 30.0
    )

    return Joint(
        name=name,
        axis=axis,
        link_length=float(link_length),
        link_offset=float(link_offset),
        dh_alpha=float(dh_alpha) if dh_alpha is not None else None,
        angle_min=float(angle_min),
        angle_max=float(angle_max),
        initial_angle=float(initial_angle),
        default_velocity=float(final_velocity),
    )
