"""
Example: Write a Simple Character

This example demonstrates writing a simple Chinese character.
"""

import sys
import os

# Add parent directory to path
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

from software.control import RobotController
from software.calligraphy import CalligraphyPlanner


def write_character(robot: RobotController, planner: CalligraphyPlanner, character: str):
    """
    Write a Chinese character using the robot.
    
    Args:
        robot: Robot controller instance
        planner: Calligraphy planner instance
        character: Character to write
    """
    print(f"\nGenerating path for character: {character}")
    
    # Generate path for character
    path = planner.create_simple_character(character)
    
    if not path:
        print(f"No path generated for character: {character}")
        return
    
    # Interpolate path for smooth motion
    path = planner.interpolate_path(path, step_size=5.0)
    
    print(f"Path contains {len(path)} points")
    
    # Get path bounds
    bounds = planner.get_path_bounds(path)
    print(f"Path bounds: X=[{bounds[0]:.1f}, {bounds[1]:.1f}], Y=[{bounds[2]:.1f}, {bounds[3]:.1f}]")
    
    # Execute path
    print("Executing path...")
    for i, point in enumerate(path):
        # Control pen
        if point.pen_down:
            robot.pen_down()
        else:
            robot.pen_up()
        
        # Move to point
        robot.move_to(point.x, point.y, 5.0 if point.pen_down else 10.0)
        
        # Progress indicator
        if (i + 1) % 10 == 0:
            print(f"  Progress: {i+1}/{len(path)} points")
    
    # Pen up at end
    robot.pen_up()
    print("Character complete!")


def main():
    """Run calligraphy example."""
    
    # Configure serial port (adjust for your system)
    port = '/dev/ttyUSB0'  # Change this to your port
    
    print("dogArm Calligraphy Example")
    print("=" * 50)
    
    # Create planner
    planner = CalligraphyPlanner(workspace_width=200, workspace_height=200)
    
    # Connect to robot
    with RobotController(port) as robot:
        # Home the robot
        print("\n1. Homing robot...")
        if not robot.home():
            print("Failed to home robot. Exiting.")
            return
        
        # Set speed for calligraphy
        print("\n2. Setting calligraphy speed...")
        robot.set_speed(300)  # Slower speed for better quality
        
        # Write characters
        characters = ["一", "十", "木"]
        
        for char in characters:
            print(f"\n{'=' * 50}")
            write_character(robot, planner, char)
            print(f"{'=' * 50}")
            
            # Move to next position (offset for multiple characters)
            # In a real application, you would calculate proper spacing
        
        # Return to home position
        print("\n3. Returning to safe position...")
        robot.move_to(100, 200, 10)
        robot.pen_up()
    
    print("\nCalligraphy example complete!")


if __name__ == '__main__':
    main()
