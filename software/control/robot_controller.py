"""
Robot Control Module

Provides high-level control interface for the dogArm robot.
"""

import serial
import time
from typing import Tuple, Optional


class RobotController:
    """
    High-level controller for dogArm robot.
    
    Handles communication with the firmware and provides
    convenient methods for robot control.
    """
    
    def __init__(self, port: str, baudrate: int = 115200, timeout: float = 1.0):
        """
        Initialize the robot controller.
        
        Args:
            port: Serial port name (e.g., '/dev/ttyUSB0' or 'COM3')
            baudrate: Serial communication baudrate
            timeout: Serial read timeout in seconds
        """
        self.port = port
        self.baudrate = baudrate
        self.timeout = timeout
        self.serial = None
        self.is_connected = False
        self.is_homed = False
        
    def connect(self) -> bool:
        """
        Connect to the robot.
        
        Returns:
            True if connection successful, False otherwise
        """
        try:
            self.serial = serial.Serial(
                port=self.port,
                baudrate=self.baudrate,
                timeout=self.timeout
            )
            time.sleep(2)  # Wait for Arduino to reset
            self.is_connected = True
            print(f"Connected to dogArm on {self.port}")
            return True
        except serial.SerialException as e:
            print(f"Failed to connect: {e}")
            return False
    
    def disconnect(self):
        """Disconnect from the robot."""
        if self.serial and self.serial.is_open:
            self.serial.close()
            self.is_connected = False
            print("Disconnected from dogArm")
    
    def send_command(self, command: str) -> Optional[str]:
        """
        Send a command to the robot and wait for response.
        
        Args:
            command: Command string to send
            
        Returns:
            Response string or None if no response
        """
        if not self.is_connected:
            print("Error: Not connected to robot")
            return None
        
        try:
            # Send command
            self.serial.write((command + '\n').encode())
            
            # Wait for response
            response = self.serial.readline().decode().strip()
            return response
        except Exception as e:
            print(f"Communication error: {e}")
            return None
    
    def home(self) -> bool:
        """
        Home the robot.
        
        Returns:
            True if homing successful
        """
        print("Homing robot...")
        response = self.send_command("HOME")
        
        if response and "OK" in response:
            self.is_homed = True
            print("Homing complete")
            return True
        else:
            print(f"Homing failed: {response}")
            return False
    
    def move_to(self, x: float, y: float, z: float) -> bool:
        """
        Move to specified Cartesian coordinates.
        
        Args:
            x: X coordinate in mm
            y: Y coordinate in mm
            z: Z coordinate in mm
            
        Returns:
            True if move command accepted
        """
        if not self.is_homed:
            print("Error: Robot not homed. Call home() first.")
            return False
        
        command = f"MOVE:{x},{y},{z}"
        response = self.send_command(command)
        
        if response and "OK" in response:
            # Wait for movement to complete
            while True:
                status = self.get_status()
                if status and not status.get('moving', False):
                    break
                time.sleep(0.1)
            return True
        else:
            print(f"Move failed: {response}")
            return False
    
    def pen_up(self) -> bool:
        """Raise the pen/brush."""
        response = self.send_command("PEN:UP")
        return response and "OK" in response
    
    def pen_down(self) -> bool:
        """Lower the pen/brush."""
        response = self.send_command("PEN:DOWN")
        return response and "OK" in response
    
    def set_speed(self, speed: float) -> bool:
        """
        Set movement speed.
        
        Args:
            speed: Speed in steps per second
            
        Returns:
            True if speed set successfully
        """
        command = f"SPEED:{speed}"
        response = self.send_command(command)
        return response and "OK" in response
    
    def get_status(self) -> Optional[dict]:
        """
        Get current robot status.
        
        Returns:
            Dictionary with status information or None
        """
        response = self.send_command("STATUS")
        
        if response and "STATUS:" in response:
            # Parse status response
            status_str = response.split("STATUS:")[1]
            status = {}
            for item in status_str.split(','):
                key, value = item.split(':')
                status[key.lower()] = (value == '1')
            return status
        return None
    
    def get_position(self) -> Optional[Tuple[float, float, float]]:
        """
        Get current robot position.
        
        Returns:
            Tuple of (x, y, z) coordinates or None
        """
        response = self.send_command("POS")
        
        if response and "POS:" in response:
            pos_str = response.split("POS:")[1]
            coords = [float(c) for c in pos_str.split(',')]
            return tuple(coords)
        return None
    
    def emergency_stop(self):
        """Emergency stop - immediately halt all motion."""
        self.send_command("STOP")
        print("Emergency stop activated")
    
    def __enter__(self):
        """Context manager entry."""
        self.connect()
        return self
    
    def __exit__(self, exc_type, exc_val, exc_tb):
        """Context manager exit."""
        self.disconnect()
