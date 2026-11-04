#!/usr/bin/env python3
"""
Simple example of using the Robotiq 2F-85/2F-140 Modbus-TCP controller.

This example demonstrates basic gripper operations:
1. Connection
2. Activation
3. Movement to different positions
4. Status monitoring
5. Disconnection
"""

from robotiq_2f85_modbus_tcp import Robotiq2F85Gripper


def main():
    # Replace with your gripper's IP address
    GRIPPER_IP = "192.168.1.11"
    
    print("Robotiq 2F-85/2F-140 Modbus-TCP Example")
    print("=" * 50)
    
    # Create gripper instance
    print(f"\nConnecting to gripper at {GRIPPER_IP}...")
    gripper = Robotiq2F85Gripper(GRIPPER_IP)
    
    try:
        # Connect to gripper
        gripper.connect()
        print("Connected!")
        
        # Check initial status
        status = gripper.get_status()
        print(f"\nInitial status:")
        print(f"  Activated: {status.is_activated}")
        print(f"  Position: {status.g_po}")
        print(f"  Fault: {status.has_fault} ({status.fault_message})")
        
        # Clear any faults
        if status.has_fault:
            print("\nClearing fault...")
            gripper.clear_fault()
            print("Fault cleared!")
        
        # Activate gripper
        if not status.is_activated:
            print("\nActivating gripper...")
            gripper.activate()
            print("Gripper activated!")
        
        # Open gripper
        print("\nOpening gripper...")
        gripper.open_gripper(speed=100, force=50, wait=True)
        status = gripper.get_status()
        print(f"  Position: {status.g_po}")
        
        # Move to intermediate position
        print("\nMoving to position 150...")
        gripper.goto_position(150, speed=100, force=50, wait=True)
        status = gripper.get_status()
        print(f"  Position: {status.g_po}")
        
        # Move to another position
        print("\nMoving to position 50...")
        gripper.goto_position(50, speed=100, force=50, wait=True)
        status = gripper.get_status()
        print(f"  Position: {status.g_po}")
        
        # Close gripper
        print("\nClosing gripper...")
        gripper.close_gripper(speed=100, force=100, wait=True)
        status = gripper.get_status()
        print(f"  Position: {status.g_po}")
        
        # Final status
        print("\nFinal status:")
        print(f"  Activated: {status.is_activated}")
        print(f"  Position: {status.g_po}")
        print(f"  Current: {status.g_cu}")
        print(f"  Fault: {status.has_fault}")
        
        print("\n" + "=" * 50)
        print("Example completed successfully!")
        
    except Exception as e:
        print(f"\nError: {e}")
        import traceback
        traceback.print_exc()
        return 1
    
    finally:
        # Disconnect
        gripper.disconnect()
        print("\nDisconnected!")
    
    return 0


if __name__ == "__main__":
    import sys
    sys.exit(main())
