"""armsim — A configurable robotic arm kinematic simulator.

Provides forward-kinematics computation for arbitrary kinematic chains
defined via Denavit–Hartenberg parameters.  Supports live WebSocket
state streaming to a retro 2D frontend viewer.

Typical usage::

    import armsim

    arm = armsim.Arm([
        armsim.Joint(name="base",     axis="z",     link_length=0,  angle_min=-180, angle_max=180),
        armsim.Joint(name="shoulder", axis="pitch", link_length=20, angle_min=-90,  angle_max=90),
        armsim.Joint(name="elbow",    axis="pitch", link_length=15, angle_min=0,    angle_max=150),
    ])
    arm.start(mode="local", port=8080)
    arm.joints["shoulder"].set_angle(45)
    arm.wait_until_idle()
    print(arm.get_end_effector_position())
    arm.stop_server()
"""

from armsim.joint import Joint
from armsim.arm import Arm
from armsim.state import ArmState
from armsim.errors import (
    ArmSimError,
    JointLimitError,
    ArmNotStartedError,
    ArmAlreadyStartedError,
    InvalidConfigError,
    DuplicateJointNameError,
)
from armsim.config import load_arm_from_config
from armsim.logging_utils import set_log_level

__version__ = "0.1.0"

__all__ = [
    "Joint",
    "Arm",
    "ArmState",
    "ArmSimError",
    "JointLimitError",
    "ArmNotStartedError",
    "ArmAlreadyStartedError",
    "InvalidConfigError",
    "DuplicateJointNameError",
    "load_arm_from_config",
    "set_log_level",
]
