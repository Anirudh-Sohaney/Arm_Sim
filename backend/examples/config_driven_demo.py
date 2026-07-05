"""Load an arm from a YAML config file — no in-code Joint/Arm construction."""

from __future__ import annotations

import armsim

import sys

if len(sys.argv) > 1:
    config_path = sys.argv[1]
else:
    config_path = "examples/configs/planar_2dof.yaml"

print(f"Loading arm from {config_path}...")
arm = armsim.load_arm_from_config(config_path)

print(f"Arm has {len(arm.joints)} joints:")
for name, joint in arm.joints.items():
    print(f"  {name}: link_length={joint.link_length}, "
          f"limits=[{joint.angle_min}, {joint.angle_max}]")

arm.start(mode="local", port=8080)
try:
    # Move all joints to 45 degrees.
    targets = {name: 45.0 for name in arm.joints}
    arm.set_angles(targets)
    arm.wait_until_idle()
    ee = arm.get_end_effector_position()
    print(f"End effector: ({ee[0]:.3f}, {ee[1]:.3f}, {ee[2]:.3f})")
finally:
    arm.stop_server()
