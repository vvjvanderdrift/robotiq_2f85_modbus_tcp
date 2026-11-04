# Robotiq 2F-85 Modbus-TCP Controller
# 
# A Python library for controlling Robotiq 2F-85 and 2F-140 grippers via Modbus-TCP.
# This implementation is based on the official Robotiq documentation and reuses
# patterns from the robotiq-ros and crigroup/robotiq repositories.

"""
Robotiq 2F-85/2F-140 Modbus-TCP Controller

This module provides a clean Python interface for controlling Robotiq 2-Finger
adaptive grippers (2F-85 and 2F-140) via Modbus-TCP protocol.

The implementation is based on:
- Robotiq 2F-85 & 2F-140 Instruction Manual (20210623)
- robotiq-ros repository patterns
- crigroup/robotiq repository Modbus-TCP implementation

Features:
- Full register-level control of the gripper
- Position, speed, and force control
- Status monitoring with object detection
- Fault status reporting
- Thread-safe operations

Example usage:
    from robotiq_2f85_modbus_tcp import Robotiq2F85Gripper
    
    # Connect to gripper
    gripper = Robotiq2F85Gripper('192.168.1.100')
    gripper.connect()
    
    # Activate gripper
    gripper.activate()
    
    # Move to position (0-255)
    gripper.goto_position(100, speed=100, force=50)
    
    # Wait for motion to complete
    gripper.wait_for_motion_complete()
    
    # Get current status
    status = gripper.get_status()
    print(f"Position: {status.position}, Current: {status.current}")
    
    # Close gripper
    gripper.close()
    
    # Disconnect
    gripper.disconnect()
"""

from .gripper import Robotiq2F85Gripper, GripperStatus, GripperCommand, ActivationState, ObjectDetection
from .modbus_tcp_transport import ModbusTcpTransport
from .exceptions import (
    RobotiqException,
    ConnectionException,
    ActivationException,
    MotionException,
    FaultException,
    TimeoutException,
    InvalidParameterException,
)

__version__ = "1.0.0"
__all__ = [
    "Robotiq2F85Gripper",
    "GripperStatus",
    "GripperCommand",
    "ActivationState",
    "ObjectDetection",
    "ModbusTcpTransport",
    "RobotiqException",
    "ConnectionException",
    "ActivationException",
    "MotionException",
    "FaultException",
    "TimeoutException",
    "InvalidParameterException",
]
