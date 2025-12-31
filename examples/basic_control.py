"""
Example: Basic Robot Control

This example demonstrates basic robot control operations.
"""

import sys
import os

# Add parent directory to path
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

from software.control import RobotController


def main():
    """Run basic robot control example."""
    
    # Configure serial port (adjust for your system)
    # Linux: typically /dev/ttyUSB0 or /dev/ttyACM0
    # Windows: typically COM3, COM4, etc.
    # macOS: typically /dev/cu.usbserial-* or /dev/cu.usbmodem*
    
    port = '/dev/ttyUSB0'  # Change this to your port
    
    print("dogArm Basic Control Example")
    print("=" * 50)
    
    # Create controller with context manager
    with RobotController(port) as robot:
        # Home the robot
        print("\n1. Homing robot...")
        if not robot.home():
            print("Failed to home robot. Exiting.")
            return
        
        # Set speed
        print("\n2. Setting speed...")
        robot.set_speed(500)
        
        # Move to some positions
        print("\n3. Moving to test positions...")
        
        positions = [
            (100, 200, 10),
            (150, 200, 10),
            (150, 250, 10),
            (100, 250, 10),
        ]
        
        for i, (x, y, z) in enumerate(positions, 1):
            print(f"   Moving to position {i}: ({x}, {y}, {z})")
            robot.move_to(x, y, z)
        
        # Test pen up/down
        print("\n4. Testing pen control...")
        robot.pen_down()
        print("   Pen down")
        robot.move_to(125, 225, 5)
        robot.pen_up()
        print("   Pen up")
        
        # Get current status
        print("\n5. Getting status...")
        status = robot.get_status()
        if status:
            print(f"   Status: {status}")
        
        # Get current position
        print("\n6. Getting position...")
        position = robot.get_position()
        if position:
            print(f"   Position: X={position[0]:.2f}, Y={position[1]:.2f}, Z={position[2]:.2f}")
        
        print("\n7. Returning to home position...")
        robot.move_to(100, 200, 10)
        robot.pen_up()
    
    print("\nExample complete!")


if __name__ == '__main__':
    main()
