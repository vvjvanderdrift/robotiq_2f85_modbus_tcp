# Robotiq 2F-85 Modbus-TCP Controller - Exceptions

"""
Exception classes for the Robotiq 2F-85/2F-140 Modbus-TCP controller.
"""


class RobotiqException(Exception):
    """Base exception for all Robotiq gripper errors."""
    pass


class ConnectionException(RobotiqException):
    """Exception raised when connection to the gripper fails."""
    pass


class ActivationException(RobotiqException):
    """Exception raised when gripper activation fails."""
    pass


class MotionException(RobotiqException):
    """Exception raised when motion commands fail."""
    pass


class FaultException(RobotiqException):
    """Exception raised when the gripper reports a fault."""
    def __init__(self, fault_code, message=None):
        self.fault_code = fault_code
        super().__init__(message or f"Gripper fault: {fault_code:#04x}")


class TimeoutException(RobotiqException):
    """Exception raised when an operation times out."""
    pass


class InvalidParameterException(RobotiqException):
    """Exception raised when invalid parameters are provided."""
    pass
