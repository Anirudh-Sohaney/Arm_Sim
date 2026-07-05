"""Parallel multi-joint movement demo with wait_until_idle."""

from __future__ import annotations

import armsim

arm = armsim.Arm([
    armsim.Joint(name="base",     axis="z",     link_length=5.0,  angle_min=-180, angle_max=180),
    armsim.Joint(name="shoulder", axis="pitch", link_length=15.0, angle_min=-90,  angle_max=90),
    armsim.Joint(name="elbow",    axis="pitch", link_length=10.0, angle_min=0,    angle_max=150),
])

arm.start(mode="local", port=8080)

try:
    # Move all three joints in parallel.
    print("Moving all joints in parallel...")
    arm.set_angles({"base": 45, "shoulder": 30, "elbow": 70})
    arm.wait_until_idle()
    ee = arm.get_end_effector_position()
    print(f"End effector: ({ee[0]:.3f}, {ee[1]:.3f}, {ee[2]:.3f})")

    # Sequential moves.
    print("Sequential: base first, then shoulder, then elbow...")
    arm.joints["base"].set_angle(90, blocking=True)
    arm.joints["shoulder"].set_angle(60, blocking=True)
    arm.joints["elbow"].set_angle(120, blocking=True)
    ee = arm.get_end_effector_position()
    print(f"End effector: ({ee[0]:.3f}, {ee[1]:.3f}, {ee[2]:.3f})")
finally:
    arm.stop_server()
    print("Server stopped.")
