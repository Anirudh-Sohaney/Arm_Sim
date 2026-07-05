"""LAN mode demo — bind to 0.0.0.0 so other devices on the network can view.

Security note: This mode has NO authentication, NO encryption.
Only use on trusted local networks.  See agent_guide/04_COMMUNICATION_PROTOCOL.md §6.
"""

from __future__ import annotations

import armsim

arm = armsim.Arm([
    armsim.Joint(name="base",     axis="z",     link_length=5.0,  angle_min=-180, angle_max=180),
    armsim.Joint(name="shoulder", axis="pitch", link_length=20.0, angle_min=-90,  angle_max=90),
    armsim.Joint(name="elbow",    axis="pitch", link_length=15.0, angle_min=0,    angle_max=150),
])

print("Starting arm in LAN mode on port 8080...")
print("Open http://<this-machine-ip>:8080 on a second device to view.")
arm.start(mode="lan", port=8080)

try:
    # Move joints slowly so the viewer can see motion.
    arm.joints["shoulder"].set_angle(45, velocity=15)
    arm.joints["elbow"].set_angle(90, velocity=15)
    arm.wait_until_idle()

    arm.joints["base"].set_angle(45, velocity=15)
    arm.wait_until_idle()

    print("Demo complete. Press Ctrl+C to stop.")
    import time
    while True:
        time.sleep(1)
except KeyboardInterrupt:
    print("\nShutting down...")
finally:
    arm.stop_server()
