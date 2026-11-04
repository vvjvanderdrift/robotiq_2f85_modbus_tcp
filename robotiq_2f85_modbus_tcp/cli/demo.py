#!/usr/bin/env python3
"""
Demo script for Robotiq 2F-85/2F-140 gripper via Modbus-TCP.

This script demonstrates basic gripper operations including activation,
movement, and status monitoring.
"""

import argparse
import time
import sys
from robotiq_2f85_modbus_tcp import (
    Robotiq2F85Gripper,
    ConnectionException,
    ActivationException,
    MotionException,
    FaultException,
)


def demo(host, port=502, slave_id=None, interactive=False):
    """
    Run a demo of Robotiq gripper operations.
    
    Args:
        host: IP address of the Modbus-TCP server
        port: Modbus-TCP port
        slave_id: Modbus slave ID
        interactive: Enable interactive mode
    """
    print(f"Connecting to Robotiq gripper at {host}:{port}...")
    
    try:
        gripper = Robotiq2F85Gripper(host=host, port=port, slave_id=slave_id)
        gripper.connect()
        print("Connected!")
        
        # Check initial status
        status = gripper.get_status()
        print(f"\nInitial status:")
        print(f"  Activated: {status.is_activated}")
        print(f"  Position: {status.g_po}")
        print(f"  Fault: {status.has_fault} ({status.fault_message})")
        
        if status.has_fault:
            print("\nClearing fault...")
            gripper.clear_fault()
            print("Fault cleared!")
        
        # Activate gripper
        if not status.is_activated:
            print("\nActivating gripper...")
            gripper.activate()
            print("Gripper activated!")
        
        # Get status after activation
        status = gripper.get_status()
        print(f"\nStatus after activation:")
        print(f"  Activation state: {status.g_sta}")
        print(f"  Position: {status.g_po}")
        
        if interactive:
            print("\n" + "="*50)
            print("Interactive Mode")
            print("="*50)
            print("Commands:")
            print("  open [speed] [force]  - Open gripper")
            print("  close [speed] [force] - Close gripper")
            print("  goto <pos> [speed] [force] - Go to position (0-255)")
            print("  stop - Stop motion")
            print("  status - Show status")
            print("  activate - Activate gripper")
            print("  deactivate - Deactivate gripper")
            print("  quit - Quit")
            print("="*50)
            
            while True:
                try:
                    cmd = input("\n> ").strip().lower()
                    
                    if not cmd:
                        continue
                    
                    if cmd == "quit" or cmd == "exit" or cmd == "q":
                        print("Goodbye!")
                        break
                    
                    elif cmd == "status":
                        status = gripper.get_status()
                        print(f"\nStatus:")
                        print(f"  Activated: {status.is_activated}")
                        print(f"  Moving: {status.is_moving}")
                        print(f"  Position: {status.g_po}")
                        print(f"  Current: {status.g_cu}")
                        print(f"  Fault: {status.has_fault} ({status.fault_message})")
                        print(f"  Object detected: {status.object_detected}")
                    
                    elif cmd == "activate":
                        print("Activating...")
                        gripper.activate()
                        print("Activated!")
                    
                    elif cmd == "deactivate":
                        print("Deactivating...")
                        gripper.deactivate()
                        print("Deactivated!")
                    
                    elif cmd == "stop":
                        print("Stopping...")
                        gripper.stop()
                        print("Stopped!")
                    
                    elif cmd.startswith("open"):
                        parts = cmd.split()
                        speed = int(parts[1]) if len(parts) > 1 else None
                        force = int(parts[2]) if len(parts) > 2 else None
                        print(f"Opening gripper (speed={speed}, force={force})...")
                        gripper.open_gripper(speed=speed, force=force, wait=True)
                        print("Gripper opened!")
                    
                    elif cmd.startswith("close"):
                        parts = cmd.split()
                        speed = int(parts[1]) if len(parts) > 1 else None
                        force = int(parts[2]) if len(parts) > 2 else None
                        print(f"Closing gripper (speed={speed}, force={force})...")
                        gripper.close_gripper(speed=speed, force=force, wait=True)
                        print("Gripper closed!")
                    
                    elif cmd.startswith("goto"):
                        parts = cmd.split()
                        position = int(parts[1])
                        speed = int(parts[2]) if len(parts) > 2 else None
                        force = int(parts[3]) if len(parts) > 3 else None
                        print(f"Moving to position {position} (speed={speed}, force={force})...")
                        gripper.goto_position(position, speed=speed, force=force, wait=True)
                        print("Motion complete!")
                    
                    else:
                        print("Unknown command. Type 'quit' to exit.")
                        
                except (ValueError, IndexError) as e:
                    print(f"Invalid command: {e}")
                except (MotionException, FaultException) as e:
                    print(f"Error: {e}")
                    
        else:
            # Run automated demo
            print("\n" + "="*50)
            print("Automated Demo")
            print("="*50)
            
            # Open gripper
            print("\n1. Opening gripper...")
            gripper.open_gripper(speed=100, force=50, wait=True)
            status = gripper.get_status()
            print(f"   Position: {status.g_po}")
            time.sleep(1)
            
            # Move to intermediate position
            print("\n2. Moving to position 150...")
            gripper.goto_position(150, speed=100, force=50, wait=True)
            status = gripper.get_status()
            print(f"   Position: {status.g_po}")
            time.sleep(1)
            
            # Move to another position
            print("\n3. Moving to position 50...")
            gripper.goto_position(50, speed=100, force=50, wait=True)
            status = gripper.get_status()
            print(f"   Position: {status.g_po}")
            time.sleep(1)
            
            # Close gripper
            print("\n4. Closing gripper...")
            gripper.close_gripper(speed=100, force=100, wait=True)
            status = gripper.get_status()
            print(f"   Position: {status.g_po}")
            time.sleep(1)
            
            print("\n" + "="*50)
            print("Demo complete!")
            print("="*50)
        
        gripper.disconnect()
        print("\nDisconnected!")
        return 0
        
    except ConnectionException as e:
        print(f"Connection error: {e}")
        return 1
    except ActivationException as e:
        print(f"Activation error: {e}")
        return 1
    except FaultException as e:
        print(f"Fault error: {e}")
        return 1
    except KeyboardInterrupt:
        print("\nInterrupted by user!")
        return 0
    except Exception as e:
        print(f"Unexpected error: {e}")
        import traceback
        traceback.print_exc()
        return 1


def main():
    """Main entry point for the demo command."""
    parser = argparse.ArgumentParser(
        description="Demo for Robotiq 2F-85/2F-140 gripper via Modbus-TCP"
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
        "-i", "--interactive",
        action="store_true",
        help="Enable interactive mode"
    )
    
    args = parser.parse_args()
    
    sys.exit(demo(
        host=args.host,
        port=args.port,
        slave_id=args.slave_id,
        interactive=args.interactive
    ))


if __name__ == "__main__":
    main()
