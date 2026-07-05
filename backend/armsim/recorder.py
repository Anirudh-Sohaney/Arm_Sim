"""Optional trajectory recording to disk (JSONL) for ML training data."""

from __future__ import annotations

import json
import time


class Recorder:
    """Buffered JSONL trajectory writer."""

    def __init__(self, path: str) -> None:
        self._file = open(path, "w")
        self._buffer: list[str] = []
        self._flush_interval = 50  # lines

    def record(
        self,
        tick: int,
        joint_angles: list[float],
        end_effector_position: tuple[float, float, float],
    ) -> None:
        """Append one line per tick."""
        entry = {
            "tick": tick,
            "timestamp": time.time(),
            "joint_angles": joint_angles,
            "end_effector": {
                "x": end_effector_position[0],
                "y": end_effector_position[1],
                "z": end_effector_position[2],
            },
        }
        self._buffer.append(json.dumps(entry) + "\n")
        if len(self._buffer) >= self._flush_interval:
            self._flush()

    def _flush(self) -> None:
        if self._buffer:
            self._file.writelines(self._buffer)
            self._buffer.clear()
            self._file.flush()

    def close(self) -> None:
        self._flush()
        self._file.close()
