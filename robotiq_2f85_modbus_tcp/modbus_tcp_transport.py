# Robotiq 2F-85 Modbus-TCP Controller - Modbus TCP Transport

"""
Modbus-TCP transport implementation for Robotiq grippers.

This module provides a low-level Modbus-TCP transport layer that handles
communication with the Robotiq gripper via Modbus-TCP protocol.
"""

import threading
from pymodbus.client import ModbusTcpClient
from pymodbus.exceptions import ModbusException
from .exceptions import ConnectionException


class ModbusTcpTransport:
    """
    Modbus-TCP transport for Robotiq grippers.
    
    This class provides a thread-safe interface for reading and writing
    Modbus registers to a Robotiq gripper via Modbus-TCP.
    
    The gripper uses the following Modbus addressing:
    - Robot Output (Command) Registers: 0x03E8 (1000 decimal) - 16-bit registers
    - Robot Input (Status) Registers: 0x07D0 (2000 decimal) - 16-bit registers
    
    Each 16-bit register contains 2 bytes from the gripper's register map.
    The register map is Little Endian for data bytes.
    
    Attributes:
        client: The Modbus TCP client instance
        lock: Thread lock for synchronized access
        slave_id: Modbus slave ID (default: 9 for Robotiq grippers)
        command_address: Modbus address for command registers (0x03E8 = 1000)
        status_address: Modbus address for status registers (0x07D0 = 2000)
    """
    
    # Modbus addressing constants (from Robotiq manual)
    MODBUS_SLAVE_ID = 0x09  # Default slave ID for Robotiq grippers
    COMMAND_ADDRESS = 0x03E8  # 1000 decimal
    STATUS_ADDRESS = 0x07D0  # 2000 decimal
    
    def __init__(self, host, port=502, slave_id=None, timeout=1.0):
        """
        Initialize the Modbus-TCP transport.
        
        Args:
            host: IP address or hostname of the Modbus-TCP server
            port: Modbus-TCP port (default: 502)
            slave_id: Modbus slave ID (default: 9 for Robotiq grippers)
            timeout: Connection timeout in seconds
        """
        self.host = host
        self.port = port
        self.slave_id = slave_id if slave_id is not None else self.MODBUS_SLAVE_ID
        self.timeout = timeout
        self.client = None
        self.lock = threading.Lock()
    
    def connect(self):
        """
        Connect to the Modbus-TCP server.
        
        Raises:
            ConnectionException: If connection fails
        """
        try:
            self.client = ModbusTcpClient(
                host=self.host,
                port=self.port,
                timeout=self.timeout,
                retries=3,
                retry_on_empty=True
            )
            # Test connection
            if not self.client.connect():
                raise ConnectionException(
                    f"Failed to connect to Modbus-TCP server at {self.host}:{self.port}"
                )
        except ModbusException as e:
            raise ConnectionException(f"Modbus connection error: {e}")
    
    def disconnect(self):
        """Disconnect from the Modbus-TCP server."""
        with self.lock:
            if self.client is not None:
                self.client.close()
                self.client = None
    
    def is_connected(self):
        """Check if the transport is connected."""
        return self.client is not None and self.client.connected
    
    def write_command_registers(self, data_bytes):
        """
        Write command data to the gripper's command registers.
        
        The command block is organized as 16-bit registers (2 bytes each).
        This method takes a list of bytes and packs them into registers.
        
        Args:
            data_bytes: List of bytes to write (must have even length)
            
        Raises:
            ConnectionException: If not connected or write fails
        """
        if not self.is_connected():
            raise ConnectionException("Not connected to Modbus-TCP server")
        
        # Ensure data has even number of bytes (for 16-bit registers)
        if len(data_bytes) % 2 != 0:
            data_bytes = data_bytes + [0]  # Pad with zero
        
        # Pack bytes into 16-bit registers (Big Endian for Modbus)
        registers = []
        for i in range(0, len(data_bytes), 2):
            # Combine two bytes into one register (MSB first)
            register_value = (data_bytes[i] << 8) | data_bytes[i + 1]
            registers.append(register_value)
        
        try:
            with self.lock:
                # Write to command registers starting at COMMAND_ADDRESS
                result = self.client.write_registers(
                    address=self.COMMAND_ADDRESS,
                    values=registers,
                    slave=self.slave_id
                )
                if result.isError():
                    raise ConnectionException(
                        f"Failed to write command registers: {result}"
                    )
        except ModbusException as e:
            raise ConnectionException(f"Modbus write error: {e}")
    
    def read_status_registers(self, num_bytes):
        """
        Read status data from the gripper's status registers.
        
        Args:
            num_bytes: Number of bytes to read (will be rounded up to even)
            
        Returns:
            List of bytes read from the status registers
            
        Raises:
            ConnectionException: If not connected or read fails
        """
        if not self.is_connected():
            raise ConnectionException("Not connected to Modbus-TCP server")
        
        # Calculate number of registers needed (2 bytes per register)
        num_registers = (num_bytes + 1) // 2
        
        try:
            with self.lock:
                # Read from status registers starting at STATUS_ADDRESS
                result = self.client.read_input_registers(
                    address=self.STATUS_ADDRESS,
                    count=num_registers,
                    slave=self.slave_id
                )
                
                if result.isError():
                    raise ConnectionException(
                        f"Failed to read status registers: {result}"
                    )
                
                # Unpack registers into bytes (Little Endian for Robotiq data)
                bytes_data = []
                for register_value in result.registers:
                    # Extract high and low bytes (MSB first in register)
                    high_byte = (register_value >> 8) & 0xFF
                    low_byte = register_value & 0xFF
                    bytes_data.append(high_byte)
                    bytes_data.append(low_byte)
                
                # Return only the requested number of bytes
                return bytes_data[:num_bytes]
        except ModbusException as e:
            raise ConnectionException(f"Modbus read error: {e}")
    
    def read_holding_registers(self, address, count):
        """
        Read holding registers (for debugging/direct access).
        
        Args:
            address: Modbus address to read from
            count: Number of registers to read
            
        Returns:
            List of register values
        """
        if not self.is_connected():
            raise ConnectionException("Not connected to Modbus-TCP server")
        
        try:
            with self.lock:
                result = self.client.read_holding_registers(
                    address=address,
                    count=count,
                    slave=self.slave_id
                )
                if result.isError():
                    raise ConnectionException(
                        f"Failed to read holding registers: {result}"
                    )
                return result.registers
        except ModbusException as e:
            raise ConnectionException(f"Modbus read error: {e}")
    
    def write_holding_registers(self, address, values):
        """
        Write holding registers (for debugging/direct access).
        
        Args:
            address: Modbus address to write to
            values: List of register values to write
        """
        if not self.is_connected():
            raise ConnectionException("Not connected to Modbus-TCP server")
        
        try:
            with self.lock:
                result = self.client.write_registers(
                    address=address,
                    values=values,
                    slave=self.slave_id
                )
                if result.isError():
                    raise ConnectionException(
                        f"Failed to write holding registers: {result}"
                    )
        except ModbusException as e:
            raise ConnectionException(f"Modbus write error: {e}")
