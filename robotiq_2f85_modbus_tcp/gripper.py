# Robotiq 2F-85 Modbus-TCP Controller - Gripper Class

"""
Main gripper control class for Robotiq 2F-85 and 2F-140 grippers.

This module provides the high-level interface for controlling Robotiq
2-Finger adaptive grippers via Modbus-TCP protocol.
"""

import time
import threading
from enum import Enum, auto
from dataclasses import dataclass, field
from typing import Optional
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


class ActivationState(Enum):
    """Activation state of the gripper."""
    RESET = 0x00
    IN_PROGRESS = 0x01
    RESERVED = 0x02
    COMPLETE = 0x03


class ObjectDetection(Enum):
    """Object detection status."""
    MOVING = 0x00
    DETECTED_WHILE_OPENING = 0x01
    DETECTED_WHILE_CLOSING = 0x02
    AT_REQUESTED_POSITION = 0x03


# Fault status codes (from Robotiq manual)
FAULT_CODES = {
    0x00: "No fault",
    0x05: "Action delayed - activation must be completed first",
    0x07: "Activation bit must be set prior to performing action",
    0x08: "Emergency stop activated",
    0x09: "Protective stop activated",
    0x0A: "Gripper in reset state",
    0x0B: "Safety reverse activated",
    0x0C: "Hardware fault",
    0x0D: "Not used",
    0x0E: "Action not allowed in current state",
    0x0F: "Communication not established",
}


@dataclass
class GripperStatus:
    """
    Current status of the Robotiq gripper.
    
    Attributes:
        g_act: Activation status echo (0 or 1)
        g_gto: Go-to position echo (0 or 1)
        g_sta: Activation state (ActivationState enum)
        g_obj: Object detection status (ObjectDetection enum)
        g_flt: Gripper fault status code
        g_pr: Position request echo (0-255)
        g_po: Current position (0-255)
        g_cu: Current motor current (0-255)
        raw_bytes: Raw status bytes from the gripper
    """
    g_act: int = 0
    g_gto: int = 0
    g_sta: ActivationState = ActivationState.RESET
    g_obj: ObjectDetection = ObjectDetection.MOVING
    g_flt: int = 0
    g_pr: int = 0
    g_po: int = 0
    g_cu: int = 0
    raw_bytes: list = field(default_factory=list)
    
    @property
    def is_activated(self) -> bool:
        """Check if the gripper is activated."""
        return self.g_act == 1
    
    @property
    def is_moving(self) -> bool:
        """Check if the gripper is currently moving."""
        return self.g_gto == 1
    
    @property
    def has_fault(self) -> bool:
        """Check if the gripper has a fault."""
        return self.g_flt != 0
    
    @property
    def fault_message(self) -> str:
        """Get the fault message for the current fault code."""
        return FAULT_CODES.get(self.g_flt, f"Unknown fault: {self.g_flt:#04x}")
    
    @property
    def object_detected(self) -> bool:
        """Check if an object has been detected."""
        return self.g_obj in (
            ObjectDetection.DETECTED_WHILE_OPENING,
            ObjectDetection.DETECTED_WHILE_CLOSING,
        )
    
    @property
    def motion_complete(self) -> bool:
        """Check if motion is complete."""
        return self.g_gto == 0 and self.g_obj == ObjectDetection.AT_REQUESTED_POSITION


@dataclass
class GripperCommand:
    """
    Command to send to the Robotiq gripper.
    
    Attributes:
        r_act: Activation request (0 = deactivate, 1 = activate)
        r_gto: Go-to position request (0 = stop, 1 = go to position)
        r_atr: Automatic release (0 = normal, 1 = emergency release)
        r_ard: Auto-release direction (0 = closing, 1 = opening)
        r_pr: Position request (0-255)
        r_sp: Speed (0-255)
        r_fr: Force (0-255)
    """
    r_act: int = 0
    r_gto: int = 0
    r_atr: int = 0
    r_ard: int = 0
    r_pr: int = 0
    r_sp: int = 0
    r_fr: int = 0
    
    def to_bytes(self) -> list:
        """
        Convert the command to bytes for Modbus transmission.
        
        The command block is organized as follows:
        - Byte 0: ACTION REQUEST (rACT, rGTO, rATR, rARD packed)
        - Byte 1: RESERVED (0)
        - Byte 2: RESERVED (0)
        - Byte 3: POSITION REQUEST (rPR)
        - Byte 4: SPEED (rSP)
        - Byte 5: FORCE (rFR)
        - Bytes 6-15: RESERVED (0)
        
        Returns:
            List of 16 bytes representing the command block
        """
        # Pack action request byte (bits 0, 3, 4, 5)
        # Bit 0: rACT
        # Bit 3: rGTO
        # Bit 4: rATR
        # Bit 5: rARD
        action_byte = (
            (self.r_act & 0x01) | 
            ((self.r_gto & 0x01) << 3) | 
            ((self.r_atr & 0x01) << 4) | 
            ((self.r_ard & 0x01) << 5)
        )
        
        # Build the command block (16 bytes)
        command_bytes = [
            action_byte,
            0,  # Byte 1: RESERVED
            0,  # Byte 2: RESERVED
            self.r_pr & 0xFF,  # Byte 3: POSITION REQUEST
            self.r_sp & 0xFF,  # Byte 4: SPEED
            self.r_fr & 0xFF,  # Byte 5: FORCE
        ]
        
        # Pad with zeros to 16 bytes
        while len(command_bytes) < 16:
            command_bytes.append(0)
        
        return command_bytes


class Robotiq2F85Gripper:
    """
    High-level controller for Robotiq 2F-85 and 2F-140 grippers via Modbus-TCP.
    
    This class provides a convenient interface for controlling Robotiq
    2-Finger adaptive grippers. It handles all the low-level Modbus
    communication and provides methods for common operations.
    
    Attributes:
        transport: The Modbus-TCP transport instance
        host: IP address of the Modbus-TCP server
        port: Modbus-TCP port
        timeout: Operation timeout in seconds
        _last_command: Last command sent to the gripper
        _last_status: Last status received from the gripper
        _lock: Thread lock for synchronized operations
    
    Example:
        >>> gripper = Robotiq2F85Gripper('192.168.1.100')
        >>> gripper.connect()
        >>> gripper.activate()
        >>> gripper.goto_position(100)
        >>> gripper.wait_for_motion_complete()
        >>> status = gripper.get_status()
        >>> print(f"Position: {status.g_po}")
        >>> gripper.close()
        >>> gripper.disconnect()
    """
    
    # Default parameters
    DEFAULT_SPEED = 100
    DEFAULT_FORCE = 50
    DEFAULT_TIMEOUT = 10.0  # seconds
    
    # Position limits (0-255 for 2F-85/2F-140)
    POSITION_MIN = 0
    POSITION_MAX = 255
    SPEED_MIN = 0
    SPEED_MAX = 255
    FORCE_MIN = 0
    FORCE_MAX = 255
    
    def __init__(
        self,
        host: str,
        port: int = 502,
        slave_id: Optional[int] = None,
        timeout: float = DEFAULT_TIMEOUT,
    ):
        """
        Initialize the gripper controller.
        
        Args:
            host: IP address or hostname of the Modbus-TCP server
            port: Modbus-TCP port (default: 502)
            slave_id: Modbus slave ID (default: 9 for Robotiq grippers)
            timeout: Operation timeout in seconds
        """
        self.host = host
        self.port = port
        self.slave_id = slave_id
        self.timeout = timeout
        
        self.transport = ModbusTcpTransport(
            host=host,
            port=port,
            slave_id=slave_id,
            timeout=timeout
        )
        
        self._last_command = GripperCommand()
        self._last_status = GripperStatus()
        self._lock = threading.Lock()
        self._activation_timeout = 5.0  # seconds for activation
    
    def connect(self):
        """
        Connect to the gripper via Modbus-TCP.
        
        Raises:
            ConnectionException: If connection fails
        """
        self.transport.connect()
    
    def disconnect(self):
        """Disconnect from the gripper."""
        self.transport.disconnect()
    
    def is_connected(self) -> bool:
        """Check if connected to the gripper."""
        return self.transport.is_connected()
    
    def _send_command(self, command: GripperCommand):
        """
        Send a command to the gripper.
        
        Args:
            command: The command to send
            
        Raises:
            ConnectionException: If not connected or send fails
        """
        if not self.is_connected():
            raise ConnectionException("Not connected to gripper")
        
        # Validate command parameters
        if not (self.POSITION_MIN <= command.r_pr <= self.POSITION_MAX):
            raise InvalidParameterException(
                f"Position {command.r_pr} out of range [{self.POSITION_MIN}, {self.POSITION_MAX}]"
            )
        if not (self.SPEED_MIN <= command.r_sp <= self.SPEED_MAX):
            raise InvalidParameterException(
                f"Speed {command.r_sp} out of range [{self.SPEED_MIN}, {self.SPEED_MAX}]"
            )
        if not (self.FORCE_MIN <= command.r_fr <= self.FORCE_MAX):
            raise InvalidParameterException(
                f"Force {command.r_fr} out of range [{self.FORCE_MIN}, {self.FORCE_MAX}]"
            )
        
        # Convert command to bytes and send
        command_bytes = command.to_bytes()
        self.transport.write_command_registers(command_bytes)
        
        # Store the last command
        with self._lock:
            self._last_command = command
    
    def _read_status(self, num_bytes: int = 6) -> list:
        """
        Read status from the gripper.
        
        Args:
            num_bytes: Number of bytes to read (default: 6 for basic status)
            
        Returns:
            List of status bytes
            
        Raises:
            ConnectionException: If not connected or read fails
        """
        if not self.is_connected():
            raise ConnectionException("Not connected to gripper")
        
        return self.transport.read_status_registers(num_bytes)
    
    def get_status(self, refresh: bool = True) -> GripperStatus:
        """
        Get the current status of the gripper.
        
        Args:
            refresh: If True, read fresh status from the gripper
            
        Returns:
            GripperStatus object with current status
        """
        if refresh:
            # Read status bytes from gripper
            status_bytes = self._read_status(6)
            
            # Parse status bytes
            status = GripperStatus(raw_bytes=status_bytes)
            
            # Byte 0: GRIPPER STATUS
            # Bits: gOBJ (7-6), Reserved (5), gSTA (4-3), Reserved (2), gGTO (1), gACT (0)
            byte0 = status_bytes[0]
            status.g_act = byte0 & 0x01
            status.g_gto = (byte0 >> 1) & 0x01
            status.g_sta = ActivationState((byte0 >> 3) & 0x03)
            status.g_obj = ObjectDetection((byte0 >> 6) & 0x03)
            
            # Byte 2: FAULT STATUS
            byte2 = status_bytes[2]
            status.g_flt = byte2 & 0x0F  # Low nibble
            
            # Byte 3: POS REQUEST ECHO
            status.g_pr = status_bytes[3]
            
            # Byte 4: POSITION
            status.g_po = status_bytes[4]
            
            # Byte 5: CURRENT
            status.g_cu = status_bytes[5]
            
            # Store the last status
            with self._lock:
                self._last_status = status
        else:
            with self._lock:
                status = self._last_status
        
        return status
    
    def activate(self, timeout: Optional[float] = None):
        """
        Activate the gripper.
        
        This must be called before any other operations. The gripper will
        perform its activation routine (calibration).
        
        Args:
            timeout: Timeout in seconds (default: _activation_timeout)
            
        Raises:
            ActivationException: If activation fails or times out
            ConnectionException: If not connected
        """
        if not self.is_connected():
            raise ConnectionException("Not connected to gripper")
        
        timeout = timeout if timeout is not None else self._activation_timeout
        start_time = time.time()
        
        # Send activation command
        command = GripperCommand(
            r_act=1,  # Activate
            r_gto=0,  # Stop
            r_atr=0,  # Normal
            r_ard=0,  # Closing
            r_pr=0,   # Position
            r_sp=0,   # Speed
            r_fr=0    # Force
        )
        self._send_command(command)
        
        # Wait for activation to complete
        while time.time() - start_time < timeout:
            status = self.get_status()
            
            # Check for faults
            if status.has_fault:
                raise ActivationException(
                    f"Activation failed with fault: {status.fault_message}"
                )
            
            # Check if activation is complete
            if status.g_sta == ActivationState.COMPLETE:
                return
            
            # Small delay between checks
            time.sleep(0.05)
        
        # Timeout
        status = self.get_status()
        raise ActivationException(
            f"Activation timed out after {timeout} seconds. Current state: {status.g_sta}"
        )
    
    def deactivate(self):
        """
        Deactivate (reset) the gripper.
        
        This clears the activation bit and resets the gripper.
        
        Raises:
            ConnectionException: If not connected
        """
        if not self.is_connected():
            raise ConnectionException("Not connected to gripper")
        
        # Send deactivation command
        command = GripperCommand(
            r_act=0,  # Deactivate
            r_gto=0,  # Stop
            r_atr=0,  # Normal
            r_ard=0,  # Closing
            r_pr=0,   # Position
            r_sp=0,   # Speed
            r_fr=0    # Force
        )
        self._send_command(command)
        
        # Wait a bit for the gripper to process the command
        time.sleep(0.1)
    
    def goto_position(
        self,
        position: int,
        speed: Optional[int] = None,
        force: Optional[int] = None,
        wait: bool = False,
        timeout: Optional[float] = None,
    ):
        """
        Move the gripper to a specific position.
        
        Args:
            position: Target position (0-255, where 0 is closed, 255 is open)
            speed: Movement speed (0-255, default: DEFAULT_SPEED)
            force: Gripping force (0-255, default: DEFAULT_FORCE)
            wait: If True, wait for motion to complete
            timeout: Timeout in seconds for waiting (default: timeout)
            
        Raises:
            MotionException: If motion fails
            ConnectionException: If not connected
            InvalidParameterException: If parameters are out of range
        """
        if not self.is_connected():
            raise ConnectionException("Not connected to gripper")
        
        speed = speed if speed is not None else self.DEFAULT_SPEED
        force = force if force is not None else self.DEFAULT_FORCE
        timeout = timeout if timeout is not None else self.timeout
        
        # Send goto command
        command = GripperCommand(
            r_act=1,  # Must stay activated
            r_gto=1,  # Go to position
            r_atr=0,  # Normal
            r_ard=0,  # Closing
            r_pr=position,
            r_sp=speed,
            r_fr=force
        )
        self._send_command(command)
        
        # Wait for motion to complete if requested
        if wait:
            self.wait_for_motion_complete(timeout=timeout)
    
    def open_gripper(
        self,
        speed: Optional[int] = None,
        force: Optional[int] = None,
        wait: bool = False,
        timeout: Optional[float] = None,
    ):
        """
        Open the gripper to its maximum position.
        
        Args:
            speed: Movement speed (0-255, default: DEFAULT_SPEED)
            force: Gripping force (0-255, default: DEFAULT_FORCE)
            wait: If True, wait for motion to complete
            timeout: Timeout in seconds for waiting
            
        Raises:
            MotionException: If motion fails
            ConnectionException: If not connected
        """
        self.goto_position(
            position=self.POSITION_MAX,
            speed=speed,
            force=force,
            wait=wait,
            timeout=timeout
        )
    
    def close_gripper(
        self,
        speed: Optional[int] = None,
        force: Optional[int] = None,
        wait: bool = False,
        timeout: Optional[float] = None,
    ):
        """
        Close the gripper to its minimum position.
        
        Args:
            speed: Movement speed (0-255, default: DEFAULT_SPEED)
            force: Gripping force (0-255, default: DEFAULT_FORCE)
            wait: If True, wait for motion to complete
            timeout: Timeout in seconds for waiting
            
        Raises:
            MotionException: If motion fails
            ConnectionException: If not connected
        """
        self.goto_position(
            position=self.POSITION_MIN,
            speed=speed,
            force=force,
            wait=wait,
            timeout=timeout
        )
    
    def stop(self):
        """
        Stop the gripper motion.
        
        This sends a stop command (rGTO = 0) to halt any current motion.
        
        Raises:
            ConnectionException: If not connected
        """
        if not self.is_connected():
            raise ConnectionException("Not connected to gripper")
        
        command = GripperCommand(
            r_act=1,  # Must stay activated
            r_gto=0,  # Stop
            r_atr=0,  # Normal
            r_ard=0,  # Closing
            r_pr=0,   # Position
            r_sp=0,   # Speed
            r_fr=0    # Force
        )
        self._send_command(command)
    
    def emergency_release(self, direction: str = "opening"):
        """
        Perform an emergency release of the gripper.
        
        This is intended for emergency situations and will override all other
        commands. After an emergency release, the gripper must be reactivated.
        
        Args:
            direction: Direction for auto-release ("opening" or "closing")
            
        Raises:
            ConnectionException: If not connected
        """
        if not self.is_connected():
            raise ConnectionException("Not connected to gripper")
        
        r_ard = 1 if direction.lower() == "opening" else 0
        
        command = GripperCommand(
            r_act=1,  # Must stay activated
            r_gto=0,  # Stop
            r_atr=1,  # Emergency auto-release
            r_ard=r_ard,  # Direction
            r_pr=0,   # Position
            r_sp=0,   # Speed
            r_fr=0    # Force
        )
        self._send_command(command)
    
    def wait_for_motion_complete(self, timeout: Optional[float] = None):
        """
        Wait for the current motion to complete.
        
        Args:
            timeout: Timeout in seconds (default: timeout)
            
        Returns:
            True if motion completed successfully
            
        Raises:
            MotionException: If motion fails or times out
            FaultException: If a fault occurs during motion
        """
        if not self.is_connected():
            raise ConnectionException("Not connected to gripper")
        
        timeout = timeout if timeout is not None else self.timeout
        start_time = time.time()
        
        while time.time() - start_time < timeout:
            status = self.get_status()
            
            # Check for faults
            if status.has_fault:
                raise FaultException(status.g_flt, status.fault_message)
            
            # Check if motion is complete
            if status.motion_complete:
                return True
            
            # Small delay between checks
            time.sleep(0.05)
        
        # Timeout
        status = self.get_status()
        raise MotionException(
            f"Motion timed out after {timeout} seconds. Current state: {status}"
        )
    
    def wait_for_object_detection(self, timeout: Optional[float] = None) -> bool:
        """
        Wait for object detection.
        
        Args:
            timeout: Timeout in seconds (default: timeout)
            
        Returns:
            True if object was detected, False if timeout
            
        Raises:
            FaultException: If a fault occurs
        """
        if not self.is_connected():
            raise ConnectionException("Not connected to gripper")
        
        timeout = timeout if timeout is not None else self.timeout
        start_time = time.time()
        
        while time.time() - start_time < timeout:
            status = self.get_status()
            
            # Check for faults
            if status.has_fault:
                raise FaultException(status.g_flt, status.fault_message)
            
            # Check if object was detected
            if status.object_detected:
                return True
            
            # Check if motion is complete (no object detected)
            if status.motion_complete:
                return False
            
            # Small delay between checks
            time.sleep(0.05)
        
        # Timeout
        return False
    
    def get_position(self) -> int:
        """
        Get the current position of the gripper.
        
        Returns:
            Current position (0-255)
            
        Raises:
            ConnectionException: If not connected
        """
        status = self.get_status()
        return status.g_po
    
    def get_current(self) -> int:
        """
        Get the current motor current.
        
        Returns:
            Current motor current (0-255)
            
        Raises:
            ConnectionException: If not connected
        """
        status = self.get_status()
        return status.g_cu
    
    def get_fault_status(self) -> int:
        """
        Get the current fault status code.
        
        Returns:
            Fault status code
            
        Raises:
            ConnectionException: If not connected
        """
        status = self.get_status()
        return status.g_flt
    
    def clear_fault(self):
        """
        Clear a fault by deactivating and reactivating the gripper.
        
        This is a convenience method that deactivates and then reactivates
        the gripper to clear any fault conditions.
        
        Raises:
            ActivationException: If reactivation fails
            ConnectionException: If not connected
        """
        if not self.is_connected():
            raise ConnectionException("Not connected to gripper")
        
        # Deactivate
        self.deactivate()
        time.sleep(0.1)
        
        # Reactivate
        self.activate()
    
    def __enter__(self):
        """Context manager entry - connect to gripper."""
        self.connect()
        return self
    
    def __exit__(self, exc_type, exc_val, exc_tb):
        """Context manager exit - disconnect from gripper."""
        self.disconnect()
        return False
    
    # Aliases for convenience
    def close(self):
        """Alias for close_gripper."""
        self.close_gripper()
    
    def open(self):
        """Alias for open_gripper."""
        self.open_gripper()
