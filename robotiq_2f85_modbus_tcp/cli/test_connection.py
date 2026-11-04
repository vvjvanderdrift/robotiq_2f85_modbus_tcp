#!/usr/bin/env python3
"""
Test connection to Robotiq gripper via Modbus-TCP.

This script tests the connection to a Robotiq gripper and displays basic status.
"""

import argparse
import sys
from robotiq_2f85_modbus_tcp import Robotiq2F85Gripper, ConnectionException


def test_connection(host, port=502, slave_id=None, verbose=False):
    """
    Test connection to a Robotiq gripper.
    
    Args:
        host: IP address of the Modbus-TCP server
        port: Modbus-TCP port
        slave_id: Modbus slave ID
        verbose: Enable verbose output
    
    Returns:
        True if connection successful, False otherwise
    """
    try:
        gripper = Robotiq2F85Gripper(host=host, port=port, slave_id=slave_id)
        
        if verbose:
            print(f"Connecting to {host}:{port}...")
        
        gripper.connect()
        
        if verbose:
            print("Connection successful!")
        
        # Try to read status
        try:
            status = gripper.get_status()
            if verbose:
                print("Status read successful!")
                print(f"  Activated: {status.is_activated}")
                print(f"  Moving: {status.is_moving}")
                print(f"  Position: {status.g_po}")
                print(f"  Current: {status.g_cu}")
                print(f"  Fault: {status.has_fault} ({status.fault_message})")
            gripper.disconnect()
            return True
        except Exception as e:
            if verbose:
                print(f"Status read failed: {e}")
            gripper.disconnect()
            return False
            
    except ConnectionException as e:
        if verbose:
            print(f"Connection failed: {e}")
        return False
    except Exception as e:
        if verbose:
            print(f"Unexpected error: {e}")
        return False


def main():
    """Main entry point for the test connection command."""
    parser = argparse.ArgumentParser(
        description="Test connection to Robotiq gripper via Modbus-TCP"
    )
    parser.add_argument(
        "host",
        help="IP address of the Modbus-TCP server"
    )
    parser.add_argument(
        "-p", "--port",
        type=int,
        default=502,
        help="Modbus-TCP port (default: 502)"
    )
    parser.add_argument(
        "-s", "--slave-id",
        type=int,
        default=None,
        help="Modbus slave ID (default: 9)"
    )
    parser.add_argument(
        "-v", "--verbose",
        action="store_true",
        help="Enable verbose output"
    )
    parser.add_argument(
        "-q", "--quiet",
        action="store_true",
        help="Quiet mode (only return exit code)"
    )
    
    args = parser.parse_args()
    
    success = test_connection(
        host=args.host,
        port=args.port,
        slave_id=args.slave_id,
        verbose=not args.quiet and args.verbose
    )
    
    if not args.quiet:
        if success:
            print("Connection test: PASSED")
        else:
            print("Connection test: FAILED")
    
    sys.exit(0 if success else 1)


if __name__ == "__main__":
    main()
