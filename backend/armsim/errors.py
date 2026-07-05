"""Custom exception hierarchy for armsim.

All exceptions inherit from :class:`ArmSimError` so calling code can catch
broadly or narrowly as needed.
"""

from __future__ import annotations


class ArmSimError(Exception):
    """Base class for all armsim-specific exceptions."""


class JointLimitError(ArmSimError):
    """Raised when a requested joint angle is outside the configured range."""

    def __init__(
        self,
        joint_name: str,
        requested_angle: float,
        angle_min: float,
        angle_max: float,
    ) -> None:
        self.joint_name = joint_name
        self.requested_angle = requested_angle
        self.angle_min = angle_min
        self.angle_max = angle_max
        super().__init__(
            f"joint '{joint_name}': requested angle {requested_angle} is "
            f"outside allowed range [{angle_min}, {angle_max}]"
        )


class ArmNotStartedError(ArmSimError):
    """Raised when a method requiring live simulation state is called before start()."""

    def __init__(self, method_name: str = "") -> None:
        msg = f"Arm has not been started. Call arm.start() before {method_name}".rstrip(
            " before "
        )
        if not method_name:
            msg = "Arm has not been started. Call arm.start() first."
        super().__init__(msg)


class ArmAlreadyStartedError(ArmSimError):
    """Raised when start() is called on an already-running Arm."""

    def __init__(self) -> None:
        super().__init__(
            "Arm is already running. Call stop_server() before starting again."
        )


class InvalidConfigError(ArmSimError):
    """Raised for malformed configuration (bad angles, missing fields, etc.)."""

    def __init__(self, message: str) -> None:
        super().__init__(message)


class DuplicateJointNameError(ArmSimError):
    """Raised when two joints in the same arm share a name."""

    def __init__(self, name: str) -> None:
        self.name = name
        super().__init__(f"Duplicate joint name: '{name}'. Joint names must be unique.")
