"""Tests for the recorder: JSONL format correctness."""

from __future__ import annotations

import json
import os

from armsim.recorder import Recorder


def test_recorder_writes_jsonl_lines(tmp_path):
    """Each record() call appends a JSON line."""
    path = str(tmp_path / "trajectory.jsonl")
    recorder = Recorder(path)
    recorder.record(0, [10.0, 20.0], (5.0, 6.0, 7.0))
    recorder.record(1, [15.0, 25.0], (8.0, 9.0, 10.0))
    recorder.close()

    with open(path) as f:
        lines = f.readlines()
    assert len(lines) == 2
    for line in lines:
        data = json.loads(line)
        assert "tick" in data
        assert "joint_angles" in data
        assert "end_effector" in data
        assert "x" in data["end_effector"]


def test_recorder_flushes_on_close(tmp_path):
    """close() flushes buffered data to disk."""
    path = str(tmp_path / "traj.jsonl")
    recorder = Recorder(path)
    recorder.record(0, [0.0], (0.0, 0.0, 0.0))
    recorder.close()
    assert os.path.getsize(path) > 0
