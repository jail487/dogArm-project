"""
Inverse Kinematics Solver

Provides inverse kinematics calculations for the double parallel linkage mechanism.
"""

import numpy as np
from typing import Tuple, Optional


class InverseKinematics:
    """
    Inverse kinematics solver for dogArm robot.
    
    Calculates joint angles required to reach a target position
    in Cartesian space.
    """
    
    def __init__(self, link1_length: float = 200.0, link2_length: float = 200.0,
                 base_width: float = 100.0):
        """
        Initialize the kinematics solver.
        
        Args:
            link1_length: Length of first link in mm
            link2_length: Length of second link in mm
            base_width: Distance between parallel linkage bases in mm
        """
        self.l1 = link1_length
        self.l2 = link2_length
        self.base_width = base_width
        
        # Workspace limits
        self.min_x = -300.0
        self.max_x = 300.0
        self.min_y = 50.0
        self.max_y = 400.0
    
    def solve(self, x: float, y: float) -> Optional[Tuple[float, float]]:
        """
        Solve inverse kinematics for target position.
        
        Args:
            x: Target X coordinate in mm
            y: Target Y coordinate in mm
            
        Returns:
            Tuple of (theta1, theta2) joint angles in degrees, or None if unreachable
        """
        # Check workspace limits
        if not self.is_reachable(x, y):
            return None
        
        # Calculate distance to target
        distance = np.sqrt(x**2 + y**2)
        
        # Check if within reachable distance
        max_reach = self.l1 + self.l2
        min_reach = abs(self.l1 - self.l2)
        
        if distance > max_reach or distance < min_reach:
            return None
        
        # Use law of cosines to find elbow angle
        cos_elbow = (self.l1**2 + self.l2**2 - distance**2) / (2 * self.l1 * self.l2)
        
        # Check validity
        if cos_elbow < -1.0 or cos_elbow > 1.0:
            return None
        
        elbow_angle = np.arccos(cos_elbow)
        
        # Calculate base angle
        base_angle = np.arctan2(y, x)
        angle2 = np.arcsin((self.l2 * np.sin(elbow_angle)) / distance)
        
        # Convert to degrees
        theta1 = np.degrees(base_angle - angle2)
        theta2 = np.degrees(base_angle + angle2)
        
        return (theta1, theta2)
    
    def forward_kinematics(self, theta1: float, theta2: float) -> Tuple[float, float]:
        """
        Calculate forward kinematics.
        
        Args:
            theta1: First joint angle in degrees
            theta2: Second joint angle in degrees
            
        Returns:
            Tuple of (x, y) position in mm
        """
        # Convert to radians
        theta1_rad = np.radians(theta1)
        theta2_rad = np.radians(theta2)
        
        # For a double parallel linkage mechanism
        # This is a simplified implementation - adjust based on your exact mechanism geometry
        # For a simple 2-link configuration as a starting point:
        x = self.l1 * np.cos(theta1_rad) + self.l2 * np.cos(theta2_rad)
        y = self.l1 * np.sin(theta1_rad) + self.l2 * np.sin(theta2_rad)
        
        # NOTE: For true parallel linkage, modify these equations based on your specific geometry
        
        return (x, y)
    
    def is_reachable(self, x: float, y: float) -> bool:
        """
        Check if position is within workspace.
        
        Args:
            x: X coordinate in mm
            y: Y coordinate in mm
            
        Returns:
            True if position is reachable
        """
        # Check workspace bounds
        if x < self.min_x or x > self.max_x:
            return False
        if y < self.min_y or y > self.max_y:
            return False
        
        # Check reachable distance
        distance = np.sqrt(x**2 + y**2)
        max_reach = self.l1 + self.l2
        min_reach = abs(self.l1 - self.l2)
        
        return min_reach <= distance <= max_reach
    
    def get_workspace_boundary(self, num_points: int = 100) -> np.ndarray:
        """
        Calculate workspace boundary points.
        
        Args:
            num_points: Number of points to calculate
            
        Returns:
            Array of (x, y) boundary points
        """
        # Outer boundary (maximum reach)
        max_reach = self.l1 + self.l2
        angles = np.linspace(0, 2 * np.pi, num_points)
        outer_x = max_reach * np.cos(angles)
        outer_y = max_reach * np.sin(angles)
        
        # Inner boundary (minimum reach)
        min_reach = abs(self.l1 - self.l2)
        inner_x = min_reach * np.cos(angles)
        inner_y = min_reach * np.sin(angles)
        
        # Combine boundaries
        boundary = np.column_stack([
            np.concatenate([outer_x, inner_x]),
            np.concatenate([outer_y, inner_y])
        ])
        
        return boundary
