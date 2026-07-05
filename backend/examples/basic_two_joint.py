"""Minimal 2-joint planar arm example — single joint move."""

from __future__ import annotations

import armsim

arm = armsim.Arm([
    armsim.Joint(name="j1", axis="z", link_length=10.0, default_velocity=90.0),
    armsim.Joint(name="j2", axis="z", link_length=8.0, default_velocity=90.0),
])

arm.start(mode="local", port=8080)

try:
    print("Moving j1 to 45 degrees...")
    arm.joints["j1"].set_angle(45.0, blocking=True)

    print("Moving j2 to 90 degrees...")
    arm.joints["j2"].set_angle(90.0, blocking=True)

    ee = arm.get_end_effector_position()
    print(f"End effector position: x={ee[0]:.3f}, y={ee[1]:.3f}, z={ee[2]:.3f}")
finally:
    arm.stop_server()
    print("Server stopped.")
