# Robotiq 2F-85 Modbus-TCP Controller - Implementation Summary

## Overview

This document summarizes the implementation of a **Modbus-TCP based controller for the Robotiq 2F-85 gripper** (also compatible with 2F-140). The implementation maximizes reuse of existing repositories and follows best practices from the robotics community.

## Architecture

### Design Philosophy

1. **Maximize Reuse**: Built upon patterns from:
   - [robotiq/robotiq-ros](https://github.com/robotiq/robotiq-ros) - Official ROS driver
   - [crigroup/robotiq](https://github.com/crigroup/robotiq) - Modbus-TCP implementation
   - [robotiq/grippers](https://github.com/robotiq/grippers) - Official SDK

2. **Clean API**: Simple, intuitive interface for common operations
3. **Thread-Safe**: All operations are thread-safe
4. **Comprehensive Error Handling**: Detailed exception hierarchy
5. **Well-Documented**: Complete documentation and examples

### Package Structure

```
robotiq_2f85_modbus_tcp/
├── robotiq_2f85_modbus_tcp/
│   ├── __init__.py           # Package exports
│   ├── gripper.py            # Main gripper class
│   ├── modbus_tcp_transport.py # Modbus-TCP transport
│   ├── exceptions.py         # Custom exceptions
│   └── cli/                  # Command-line tools
│       ├── __init__.py
│       ├── test_connection.py
│       └── demo.py
├── examples/
│   ├── __init__.py
│   └── simple_example.py
├── tests/
│   ├── __init__.py
│   └── test_gripper.py
├── setup.py
├── requirements.txt
├── README.md
└── LICENSE
```

## Key Components

### 1. ModbusTcpTransport (`modbus_tcp_transport.py`)

Low-level Modbus-TCP transport layer using [pymodbus](https://pymodbus.readthedocs.io).

**Features:**
- Thread-safe operations with locking
- Automatic byte packing/unpacking for Robotiq register format
- Support for reading/writing command and status registers
- Configurable slave ID, timeout, and port

**Modbus Addressing:**
- Command Registers: `0x03E8` (1000 decimal)
- Status Registers: `0x07D0` (2000 decimal)
- Slave ID: `0x09` (9 decimal) - default for Robotiq grippers

### 2. Robotiq2F85Gripper (`gripper.py`)

High-level gripper controller with comprehensive functionality.

**Main Features:**
- Connection management (connect/disconnect)
- Activation and deactivation
- Position control (0-255 range)
- Speed and force control (0-255 range)
- Status monitoring
- Fault detection and clearing
- Object detection
- Motion completion detection

**Register Mapping (from Robotiq Manual):**

| Byte | Command (Output) | Status (Input) |
|------|------------------|----------------|
| 0    | ACTION REQUEST   | GRIPPER STATUS |
| 1    | RESERVED         | RESERVED      |
| 2    | RESERVED         | FAULT STATUS  |
| 3    | POSITION REQUEST | POS REQUEST ECHO |
| 4    | SPEED            | POSITION      |
| 5    | FORCE            | CURRENT       |
| 6-15 | RESERVED         | RESERVED      |

**Action Request Byte (Byte 0):**
- Bit 0: `rACT` - Activation request
- Bit 3: `rGTO` - Go-to position
- Bit 4: `rATR` - Automatic release
- Bit 5: `rARD` - Auto-release direction

**Gripper Status Byte (Byte 0):**
- Bit 0: `gACT` - Activation status echo
- Bit 1: `gGTO` - Go-to position echo
- Bits 3-4: `gSTA` - Activation state
- Bits 6-7: `gOBJ` - Object detection

### 3. Data Classes

#### GripperCommand
Represents a command to send to the gripper:
- `r_act`: Activation request (0/1)
- `r_gto`: Go-to position (0/1)
- `r_atr`: Automatic release (0/1)
- `r_ard`: Auto-release direction (0/1)
- `r_pr`: Position request (0-255)
- `r_sp`: Speed (0-255)
- `r_fr`: Force (0-255)

#### GripperStatus
Represents the current status of the gripper:
- `g_act`: Activation status echo
- `g_gto`: Go-to position echo
- `g_sta`: Activation state (ActivationState enum)
- `g_obj`: Object detection (ObjectDetection enum)
- `g_flt`: Fault status code
- `g_pr`: Position request echo
- `g_po`: Current position
- `g_cu`: Current motor current

### 4. Enums

- **ActivationState**: RESET, IN_PROGRESS, RESERVED, COMPLETE
- **ObjectDetection**: MOVING, DETECTED_WHILE_OPENING, DETECTED_WHILE_CLOSING, AT_REQUESTED_POSITION

### 5. Exception Hierarchy

```
RobotiqException (base)
├── ConnectionException
├── ActivationException
├── MotionException
├── FaultException
├── TimeoutException
└── InvalidParameterException
```

## Usage Examples

### Basic Usage

```python
from robotiq_2f85_modbus_tcp import Robotiq2F85Gripper

# Connect
gripper = Robotiq2F85Gripper('192.168.1.100')
gripper.connect()

# Activate
gripper.activate()

# Move to position
gripper.goto_position(100, speed=100, force=50)
gripper.wait_for_motion_complete()

# Get status
status = gripper.get_status()
print(f"Position: {status.g_po}")

# Disconnect
gripper.disconnect()
```

### Context Manager

```python
with Robotiq2F85Gripper('192.168.1.100') as gripper:
    gripper.connect()
    gripper.activate()
    gripper.open_gripper()
    # Automatically disconnects on exit
```

### Advanced Usage

```python
# Custom parameters
gripper = Robotiq2F85Gripper(
    host='192.168.1.100',
    port=502,
    slave_id=9,
    timeout=10.0
)

# Open with custom parameters
gripper.open_gripper(speed=150, force=80, wait=True)

# Check for object detection
status = gripper.get_status()
if status.object_detected:
    print("Object detected!")

# Emergency release (use with caution!)
gripper.emergency_release(direction="opening")

# Clear faults
gripper.clear_fault()
```

## CLI Tools

### Test Connection

```bash
# Basic test
robotiq-test 192.168.1.100

# Verbose output
robotiq-test 192.168.1.100 -v

# Custom port
robotiq-test 192.168.1.100 -p 502 -s 9
```

### Interactive Demo

```bash
# Interactive mode
robotiq-demo 192.168.1.100 -i

# Automated demo
robotiq-demo 192.168.1.100
```

## Implementation Details

### Modbus Protocol

The implementation uses **Modbus-TCP** protocol with the following characteristics:

1. **Transport**: TCP/IP
2. **Port**: 502 (default)
3. **Slave ID**: 9 (default for Robotiq grippers)
4. **Function Codes**:
   - FC 0x03: Read Holding Registers
   - FC 0x04: Read Input Registers
   - FC 0x10: Write Multiple Registers

### Byte Order

- **Modbus Addressing**: Big Endian (MSB first)
- **Data Bytes**: Little Endian (LSB first) - Robotiq-specific

### Register Packing

Each 16-bit Modbus register contains 2 bytes from the gripper's register map:
- Register N = (byte[2N] << 8) | byte[2N+1]

### Communication Flow

1. **Command**: Application → Modbus-TCP → Gripper
   - Pack command bytes into registers
   - Write to command registers (0x03E8)
   
2. **Status**: Gripper → Modbus-TCP → Application
   - Read from status registers (0x07D0)
   - Unpack registers into status bytes

### Thread Safety

All operations are thread-safe through:
- Transport-level locking (ModbusTcpTransport._lock)
- Gripper-level locking (Robotiq2F85Gripper._lock)
- Atomic read-modify-write operations

## Compatibility

### Hardware

- ✅ Robotiq 2F-85
- ✅ Robotiq 2F-140
- ✅ Any Modbus-TCP to RS-485 gateway
- ✅ Robotiq Universal Controller (with Modbus-TCP support)

### Software

- ✅ Python 3.8+
- ✅ pymodbus 3.0.0+
- ✅ Linux, macOS, Windows

## Testing

The implementation includes comprehensive unit tests:

```bash
# Run all tests
python -m pytest tests/ -v

# Run specific test file
python -m pytest tests/test_gripper.py -v
```

### Test Coverage

- ✅ Command byte packing
- ✅ Status byte unpacking
- ✅ Connection management
- ✅ Activation/deactivation
- ✅ Motion control
- ✅ Parameter validation
- ✅ Error handling

## Performance Considerations

1. **Exchange Rate**: Maximum 200 Hz (5 ms minimum delay between commands)
2. **Timeout**: Configurable per operation
3. **Thread Safety**: Minimal overhead from locking
4. **Bulk Operations**: Efficient register reads/writes

## Future Enhancements

1. **Async Support**: Add async/await interface
2. **ROS 2 Integration**: Create ROS 2 wrapper
3. **Additional Protocols**: Support for Modbus RTU (serial)
4. **Advanced Features**: Gripper configuration and calibration
5. **Performance Metrics**: Add timing and throughput monitoring

## References

### Documentation

- [Robotiq 2F-85 & 2F-140 Instruction Manual](https://support.robotiq.com) - 20210623
- [Modbus Protocol Specification](http://www.modbus.org/docs/Modbus_over_serial_line_V1_02.pdf)
- [pymodbus Documentation](https://pymodbus.readthedocs.io)

### Related Repositories

- [robotiq/robotiq-ros](https://github.com/robotiq/robotiq-ros) - Official ROS driver
- [crigroup/robotiq](https://github.com/crigroup/robotiq) - Modbus-TCP implementation
- [robotiq/grippers](https://github.com/robotiq/grippers) - Official SDK

## License

This project is licensed under the **BSD-3-Clause License**. See [LICENSE](LICENSE) for details.

## Contributing

Contributions are welcome! Please feel free to:
- Report bugs
- Suggest features
- Submit pull requests

## Support

For support and questions:
- Check the [documentation](README.md)
- Review the [examples](examples/)
- Consult the [Robotiq manual](https://support.robotiq.com)

## Appendix A: Register Details

### Command Registers (Robot Output)

| Byte | Field | Bits | Description | Range |
|------|-------|------|-------------|-------|
| 0 | ACTION REQUEST | 0 | rACT - Activate | 0/1 |
| 0 | ACTION REQUEST | 3 | rGTO - Go to position | 0/1 |
| 0 | ACTION REQUEST | 4 | rATR - Auto-release | 0/1 |
| 0 | ACTION REQUEST | 5 | rARD - Auto-release direction | 0/1 |
| 3 | POSITION REQUEST | - | rPR - Target position | 0-255 |
| 4 | SPEED | - | rSP - Speed | 0-255 |
| 5 | FORCE | - | rFR - Force | 0-255 |

### Status Registers (Robot Input)

| Byte | Field | Bits | Description | Range |
|------|-------|------|-------------|-------|
| 0 | GRIPPER STATUS | 0 | gACT - Activation status echo | 0/1 |
| 0 | GRIPPER STATUS | 1 | gGTO - Go-to position echo | 0/1 |
| 0 | GRIPPER STATUS | 3-4 | gSTA - Activation state | 0-3 |
| 0 | GRIPPER STATUS | 6-7 | gOBJ - Object detection | 0-3 |
| 2 | FAULT STATUS | 0-3 | gFLT - Gripper fault | 0-15 |
| 2 | FAULT STATUS | 4-7 | kFLT - Controller fault | 0-15 |
| 3 | POS REQUEST ECHO | - | gPR - Position request echo | 0-255 |
| 4 | POSITION | - | gPO - Current position | 0-255 |
| 5 | CURRENT | - | gCU - Current motor current | 0-255 |

### Fault Codes

| Code | Description | Severity |
|------|-------------|----------|
| 0x00 | No fault | None |
| 0x05 | Action delayed - activation must be completed first | Priority |
| 0x07 | Activation bit must be set prior to performing action | Priority |
| 0x08 | Emergency stop activated | Minor |
| 0x09 | Protective stop activated | Minor |
| 0x0A | Gripper in reset state | Minor |
| 0x0B | Safety reverse activated | Minor |
| 0x0C | Hardware fault | Major |
| 0x0E | Action not allowed in current state | Priority |
| 0x0F | Communication not established | Major |

## Appendix B: Modbus-TCP Setup

### Using a Modbus-TCP to RS-485 Gateway

To use Modbus-TCP with a Robotiq gripper that has RS-485 interface:

1. **Hardware Setup**:
   - Connect the gateway to the gripper's RS-485 port
   - Connect the gateway to your network
   - Power on both devices

2. **Configuration**:
   - Set gateway IP address (e.g., 192.168.1.100)
   - Set gateway Modbus-TCP port (default: 502)
   - Set RS-485 baud rate to match gripper (default: 115200)
   - Set RS-485 slave ID to match gripper (default: 9)

3. **Software**:
   ```python
   from robotiq_2f85_modbus_tcp import Robotiq2F85Gripper
   
   # Connect to gateway
   gripper = Robotiq2F85Gripper('192.168.1.100')
   gripper.connect()
   
   # Use as normal
   gripper.activate()
   ```

### Using Robotiq Universal Controller

The Robotiq Universal Controller supports Modbus-TCP directly:

1. **Configuration**:
   - Set controller IP address
   - Enable Modbus-TCP protocol
   - Configure gripper parameters

2. **Software**:
   ```python
   from robotiq_2f85_modbus_tcp import Robotiq2F85Gripper
   
   # Connect to Universal Controller
   gripper = Robotiq2F85Gripper('192.168.1.200')
   gripper.connect()
   
   # Use as normal
   gripper.activate()
   ```

## Conclusion

This implementation provides a **clean, reusable, and well-documented** Modbus-TCP controller for Robotiq 2F-85 and 2F-140 grippers. It maximizes reuse of existing repositories and patterns while providing a simple, intuitive API for robotics applications.
