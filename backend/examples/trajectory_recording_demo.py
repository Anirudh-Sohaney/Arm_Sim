"""Trajectory recording demo — record joint angles + EE positions to disk.

Produces a JSONL file suitable for ML training without further transformation.
"""

from __future__ import annotations

import armsim
import tempfile
import os

output_path = os.path.join(tempfile.gettempdir(), "arm_trajectory.jsonl")
print(f"Recording trajectory to: {output_path}")

arm = armsim.Arm([
    armsim.Joint(name="j1", axis="z", link_length=10.0, angle_min=-180, angle_max=180),
    armsim.Joint(name="j2", axis="z", link_length=8.0,  angle_min=-150, angle_max=150),
])

arm.start(mode="local", port=8080, record_to=output_path)

try:
    # Sweep both joints through a range of angles.
    for angle in range(0, 91, 15):
        arm.set_angles({"j1": float(angle), "j2": float(angle * 1.5)})
        arm.wait_until_idle()

    print(f"Recorded {os.path.getsize(output_path)} bytes.")
    print(f"First few lines of {output_path}:")
    with open(output_path) as f:
        for i, line in enumerate(f):
            if i >= 3:
                break
            print(f"  {line.rstrip()}")
finally:
    arm.stop_server()
    print("Server stopped.")
