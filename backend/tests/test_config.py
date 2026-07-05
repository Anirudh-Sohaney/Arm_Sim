"""Tests for config loading: valid configs, validation error paths."""

from __future__ import annotations

import os
from pathlib import Path

import pytest
import yaml

from armsim.config import load_arm_from_config
from armsim.errors import (
    InvalidConfigError,
    DuplicateJointNameError,
)


def _write_yaml(path: str, data: dict) -> None:
    with open(path, "w") as f:
        yaml.dump(data, f)


class TestValidConfigs:
    """Loading valid config files should produce correct Arm objects."""

    def test_minimal_config(self, temp_dir):
        """A config with just schema_version and one joint works."""
        path = os.path.join(temp_dir, "minimal.yaml")
        _write_yaml(path, {
            "schema_version": 1,
            "joints": [{"name": "only", "link_length": 10}],
        })
        arm = load_arm_from_config(path)
        assert len(arm.joints) == 1
        assert "only" in arm.joints

    def test_planar_2dof_config(self, temp_dir):
        """2-DOF planar config loads correctly."""
        path = os.path.join(temp_dir, "planar.yaml")
        _write_yaml(path, {
            "schema_version": 1,
            "arm_name": "test_planar",
            "default_velocity": 60,
            "joints": [
                {"name": "j1", "axis": "z", "link_length": 10, "angle_min": -90, "angle_max": 90},
                {"name": "j2", "axis": "z", "link_length": 8, "angle_min": -90, "angle_max": 90, "velocity": 45},
            ],
        })
        arm = load_arm_from_config(path)
        assert arm.joints["j1"].default_velocity == 60.0
        assert arm.joints["j2"].default_velocity == 45.0
        assert arm.joints["j1"].angle_min == -90.0

    def test_tick_rate_hz_in_config(self, temp_dir):
        """Config can specify tick_rate_hz."""
        path = os.path.join(temp_dir, "fast.yaml")
        _write_yaml(path, {
            "schema_version": 1,
            "tick_rate_hz": 60,
            "joints": [{"name": "j", "link_length": 5}],
        })
        arm = load_arm_from_config(path)
        assert arm.tick_rate_hz == 60.0


class TestConfigErrors:
    """Every validation rule should raise the correct exception."""

    def test_missing_schema_version(self, temp_dir):
        path = os.path.join(temp_dir, "bad.yaml")
        _write_yaml(path, {"joints": [{"name": "j"}]})
        with pytest.raises(InvalidConfigError, match="schema_version"):
            load_arm_from_config(path)

    def test_unsupported_schema_version(self, temp_dir):
        path = os.path.join(temp_dir, "bad.yaml")
        _write_yaml(path, {"schema_version": 99, "joints": [{"name": "j"}]})
        with pytest.raises(InvalidConfigError, match="schema_version"):
            load_arm_from_config(path)

    def test_empty_joints_list(self, temp_dir):
        path = os.path.join(temp_dir, "bad.yaml")
        _write_yaml(path, {"schema_version": 1, "joints": []})
        with pytest.raises(InvalidConfigError):
            load_arm_from_config(path)

    def test_duplicate_joint_names(self, temp_dir):
        path = os.path.join(temp_dir, "bad.yaml")
        _write_yaml(path, {
            "schema_version": 1,
            "joints": [
                {"name": "dup", "link_length": 5},
                {"name": "dup", "link_length": 3},
            ],
        })
        with pytest.raises(DuplicateJointNameError):
            load_arm_from_config(path)

    def test_initial_angle_out_of_range(self, temp_dir):
        path = os.path.join(temp_dir, "bad.yaml")
        _write_yaml(path, {
            "schema_version": 1,
            "joints": [
                {"name": "j", "angle_min": -90, "angle_max": 90, "initial_angle": 180},
            ],
        })
        with pytest.raises(InvalidConfigError, match="initial_angle"):
            load_arm_from_config(path)

    def test_negative_link_length(self, temp_dir):
        path = os.path.join(temp_dir, "bad.yaml")
        _write_yaml(path, {
            "schema_version": 1,
            "joints": [{"name": "j", "link_length": -5}],
        })
        with pytest.raises(InvalidConfigError, match="link_length"):
            load_arm_from_config(path)

    def test_invalid_axis_string(self, temp_dir):
        path = os.path.join(temp_dir, "bad.yaml")
        _write_yaml(path, {
            "schema_version": 1,
            "joints": [{"name": "j", "axis": "diagonal"}],
        })
        with pytest.raises(InvalidConfigError, match="axis"):
            load_arm_from_config(path)

    def test_negative_velocity(self, temp_dir):
        path = os.path.join(temp_dir, "bad.yaml")
        _write_yaml(path, {
            "schema_version": 1,
            "default_velocity": -10,
            "joints": [{"name": "j", "link_length": 5}],
        })
        with pytest.raises(InvalidConfigError):
            load_arm_from_config(path)
