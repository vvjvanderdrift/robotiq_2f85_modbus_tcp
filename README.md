# Robotiq 2F-85/2F-140 Modbus-TCP Controller

[![Python 3.8+](https://img.shields.io/badge/python-3.8+-blue.svg)](https://www.python.org/downloads/)
[![License: BSD-3](https://img.shields.io/badge/license-BSD--3-blue.svg)](https://opensource.org/licenses/BSD-3-Clause)

A Python library for controlling **Robotiq 2F-85** and **2F-140** adaptive grippers via **Modbus-TCP** protocol.

This implementation is based on:
- Robotiq 2F-85 & 2F-140 Instruction Manual (20210623)
- Official [robotiq-ros](https://github.com/robotiq/robotiq-ros) repository patterns
- [crigroup/robotiq](https://github.com/crigroup/robotiq) Modbus-TCP implementation

## Features

- ✅ Full register-level control of the gripper
- ✅ Position, speed, and force control (0-255 range)
- ✅ Status monitoring with object detection
- ✅ Fault status reporting with descriptive messages
- ✅ Thread-safe operations
- ✅ Context manager support
- ✅ Comprehensive error handling
- ✅ CLI tools for testing and demos

## Installation

### Prerequisites

- Python 3.8 or later
- pip (Python package manager)

### Recommended: Use a Virtual Environment

We strongly recommend using a virtual environment to isolate dependencies:

```bash
# Create a virtual environment
python -m venv robotiq_env

# Activate it
# Linux/macOS:
source robotiq_env/bin/activate
# Windows:
robotiq_env\Scripts\activate

# Your terminal prompt should now show (robotiq_env)
```

### Install from source

```bash
# Clone the repository
git clone https://github.com/vvjvanderdrift/robotiq_2f85_modbus_tcp.git
cd robotiq_2f85_modbus_tcp

# Install the package
pip install .

# Or install in development mode (editable install)
pip install -e .
```

### Install with dependencies

```bash
pip install .[dev]  # Includes development dependencies
```

## Quick Start

### Basic Usage

```python
from robotiq_2f85_modbus_tcp import Robotiq2F85Gripper

# Connect to gripper
gripper = Robotiq2F85Gripper('192.168.1.100')
gripper.connect()

# Activate gripper
gripper.activate()

# Move to position (0-255, where 0 is closed, 255 is open)
gripper.goto_position(100, speed=100, force=50)

# Wait for motion to complete
gripper.wait_for_motion_complete()

# Get current status
status = gripper.get_status()
print(f"Position: {status.g_po}, Current: {status.g_cu}")

# Close gripper
gripper.close()

# Disconnect
gripper.disconnect()
```

### Using Context Manager

```python
from robotiq_2f85_modbus_tcp import Robotiq2F85Gripper

with Robotiq2F85Gripper('192.168.1.100') as gripper:
    gripper.connect()
    gripper.activate()
    gripper.goto_position(100)
    gripper.wait_for_motion_complete()
    # Automatically disconnects when exiting the context
```

### Advanced Usage

```python
from robotiq_2f85_modbus_tcp import Robotiq2F85Gripper

# Connect with custom parameters
gripper = Robotiq2F85Gripper(
    host='192.168.1.100',
    port=502,
    slave_id=9,  # Default for Robotiq grippers
    timeout=10.0
)

# Connect
gripper.connect()

# Activate and handle faults
try:
    gripper.activate()
    
    # Open gripper with custom parameters
    gripper.open_gripper(speed=150, force=80, wait=True)
    
    # Check if object was detected during motion
    status = gripper.get_status()
    if status.object_detected:
        print("Object detected!")
    
    # Close gripper
    gripper.close_gripper(speed=100, force=100, wait=True)
    
    # Emergency release (use with caution!)
    # gripper.emergency_release(direction="opening")
    
except FaultException as e:
    print(f"Fault occurred: {e}")
    gripper.clear_fault()
    
finally:
    gripper.disconnect()
```

## CLI Tools

### Test Connection

```bash
# Test connection to gripper
robotiq-test 192.168.1.100

# With verbose output
robotiq-test 192.168.1.100 -v

# Custom port
robotiq-test 192.168.1.100 -p 502
```

### Interactive Demo

```bash
# Run interactive demo
robotiq-demo 192.168.1.100 -i

# Run automated demo
robotiq-demo 192.168.1.100
```

## API Reference

### Robotiq2F85Gripper

Main class for controlling the gripper.

#### Constructor

```python
Robotiq2F85Gripper(
    host: str,
    port: int = 502,
    slave_id: Optional[int] = None,
    timeout: float = 10.0
)
```

- `host`: IP address of the Modbus-TCP server
- `port`: Modbus-TCP port (default: 502)
- `slave_id`: Modbus slave ID (default: 9 for Robotiq grippers)
- `timeout`: Operation timeout in seconds

#### Connection Methods

- `connect()`: Connect to the gripper
- `disconnect()`: Disconnect from the gripper
- `is_connected()`: Check connection status

#### Activation Methods

- `activate(timeout=None)`: Activate the gripper
- `deactivate()`: Deactivate (reset) the gripper
- `clear_fault()`: Clear faults by deactivating and reactivating

#### Motion Methods

- `goto_position(position, speed=None, force=None, wait=False, timeout=None)`: Move to specific position
- `open_gripper(speed=None, force=None, wait=False, timeout=None)`: Open gripper to maximum
- `close_gripper(speed=None, force=None, wait=False, timeout=None)`: Close gripper to minimum
- `stop()`: Stop current motion
- `emergency_release(direction="opening")`: Emergency release

#### Status Methods

- `get_status(refresh=True)`: Get current status
- `get_position()`: Get current position
- `get_current()`: Get current motor current
- `get_fault_status()`: Get fault status code
- `wait_for_motion_complete(timeout=None)`: Wait for motion to complete
- `wait_for_object_detection(timeout=None)`: Wait for object detection

#### Properties

- `POSITION_MIN`: Minimum position (0)
- `POSITION_MAX`: Maximum position (255)
- `SPEED_MIN`: Minimum speed (0)
- `SPEED_MAX`: Maximum speed (255)
- `FORCE_MIN`: Minimum force (0)
- `FORCE_MAX`: Maximum force (255)

### GripperStatus

Data class containing gripper status information.

#### Attributes

- `g_act`: Activation status echo (0 or 1)
- `g_gto`: Go-to position echo (0 or 1)
- `g_sta`: Activation state (ActivationState enum)
- `g_obj`: Object detection status (ObjectDetection enum)
- `g_flt`: Gripper fault status code
- `g_pr`: Position request echo (0-255)
- `g_po`: Current position (0-255)
- `g_cu`: Current motor current (0-255)
- `raw_bytes`: Raw status bytes from the gripper

#### Properties

- `is_activated`: Check if gripper is activated
- `is_moving`: Check if gripper is moving
- `has_fault`: Check if gripper has a fault
- `fault_message`: Get fault message for current fault code
- `object_detected`: Check if object was detected
- `motion_complete`: Check if motion is complete

### Enums

- `ActivationState`: RESET, IN_PROGRESS, RESERVED, COMPLETE
- `ObjectDetection`: MOVING, DETECTED_WHILE_OPENING, DETECTED_WHILE_CLOSING, AT_REQUESTED_POSITION

## Register Mapping

The controller uses the following register mapping (from Robotiq manual):

### Command Registers (Robot Output)

| Byte | Field | Description |
|------|-------|-------------|
| 0 | ACTION REQUEST | rACT, rGTO, rATR, rARD |
| 1 | RESERVED | - |
| 2 | RESERVED | - |
| 3 | POSITION REQUEST | rPR (0-255) |
| 4 | SPEED | rSP (0-255) |
| 5 | FORCE | rFR (0-255) |
| 6-15 | RESERVED | - |

### Status Registers (Robot Input)

| Byte | Field | Description |
|------|-------|-------------|
| 0 | GRIPPER STATUS | gACT, gGTO, gSTA, gOBJ |
| 1 | RESERVED | - |
| 2 | FAULT STATUS | gFLT, kFLT |
| 3 | POS REQUEST ECHO | gPR |
| 4 | POSITION | gPO (0-255) |
| 5 | CURRENT | gCU (0-255) |
| 6-15 | RESERVED | - |

### Modbus Addressing

- **Command Registers**: 0x03E8 (1000 decimal)
- **Status Registers**: 0x07D0 (2000 decimal)
- **Slave ID**: 0x09 (9 decimal)

## Error Handling

The library provides comprehensive error handling with custom exceptions:

- `RobotiqException`: Base exception for all errors
- `ConnectionException`: Connection-related errors
- `ActivationException`: Activation-related errors
- `MotionException`: Motion-related errors
- `FaultException`: Gripper fault errors
- `TimeoutException`: Operation timeout errors
- `InvalidParameterException`: Invalid parameter errors

Example:

```python
from robotiq_2f85_modbus_tcp import Robotiq2F85Gripper, ConnectionException

try:
    gripper = Robotiq2F85Gripper('192.168.1.100')
    gripper.connect()
except ConnectionException as e:
    print(f"Connection failed: {e}")
```

## Requirements

- Python 3.8+
- pymodbus 3.0.0+

## Compatibility

This library is compatible with:
- Robotiq 2F-85 gripper
- Robotiq 2F-140 gripper
- Any Modbus-TCP to RS-485 gateway (for direct Modbus-TCP connection)
- Robotiq Universal Controller (with Modbus-TCP support)

## Contributing

Contributions are welcome! Please feel free to submit a Pull Request.

## License

This project is licensed under the BSD-3-Clause License - see the [LICENSE](LICENSE) file for details.

## Acknowledgments

- [Robotiq](https://robotiq.com) for the gripper hardware and documentation
- [robotiq-ros](https://github.com/robotiq/robotiq-ros) repository for inspiration
- [crigroup/robotiq](https://github.com/crigroup/robotiq) for Modbus-TCP implementation patterns
- [pymodbus](https://github.com/riptideio/pymodbus) for Modbus communication

## References

- [Robotiq 2F-85 & 2F-140 Instruction Manual](https://support.robotiq.com)
- [Modbus Protocol Specification](http://www.modbus.org/docs/Modbus_over_serial_line_V1_02.pdf)
- [pymodbus Documentation](https://pymodbus.readthedocs.io)
