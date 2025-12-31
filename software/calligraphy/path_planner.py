"""
Calligraphy Path Planner

Converts Chinese characters to robot trajectories suitable for calligraphy writing.
"""

import numpy as np
from typing import List, Tuple
import json
import os


class PathPoint:
    """Represents a point in a calligraphy path."""
    
    def __init__(self, x: float, y: float, pen_down: bool = True):
        self.x = x
        self.y = y
        self.pen_down = pen_down
    
    def __repr__(self):
        return f"PathPoint(x={self.x}, y={self.y}, pen={'down' if self.pen_down else 'up'})"


class CalligraphyPlanner:
    """
    Plans paths for writing Chinese calligraphy.
    
    Converts character stroke data to robot-executable trajectories.
    """
    
    def __init__(self, workspace_width: float = 200.0, workspace_height: float = 200.0):
        """
        Initialize the calligraphy planner.
        
        Args:
            workspace_width: Width of writing workspace in mm
            workspace_height: Height of writing workspace in mm
        """
        self.workspace_width = workspace_width
        self.workspace_height = workspace_height
        self.current_path = []
    
    def load_character(self, character_file: str) -> bool:
        """
        Load character stroke data from file.
        
        Args:
            character_file: Path to character data file (JSON format)
            
        Returns:
            True if loaded successfully
        """
        try:
            with open(character_file, 'r', encoding='utf-8') as f:
                data = json.load(f)
            
            self.character_data = data
            return True
        except Exception as e:
            print(f"Error loading character: {e}")
            return False
    
    def plan_path(self, strokes: List[List[Tuple[float, float]]], 
                  scale: float = 1.0,
                  offset_x: float = 0.0,
                  offset_y: float = 200.0) -> List[PathPoint]:
        """
        Plan a path from stroke data.
        
        Args:
            strokes: List of strokes, each stroke is a list of (x, y) points
            scale: Scaling factor for the character
            offset_x: X offset for positioning
            offset_y: Y offset for positioning
            
        Returns:
            List of PathPoint objects
        """
        path = []
        
        for stroke in strokes:
            if len(stroke) == 0:
                continue
            
            # Pen up and move to start of stroke
            start_x, start_y = stroke[0]
            scaled_x = start_x * scale + offset_x
            scaled_y = start_y * scale + offset_y
            path.append(PathPoint(scaled_x, scaled_y, pen_down=False))
            
            # Pen down for stroke
            for x, y in stroke:
                scaled_x = x * scale + offset_x
                scaled_y = y * scale + offset_y
                path.append(PathPoint(scaled_x, scaled_y, pen_down=True))
            
            # Pen up at end of stroke
            path.append(PathPoint(scaled_x, scaled_y, pen_down=False))
        
        self.current_path = path
        return path
    
    def create_simple_character(self, character: str) -> List[PathPoint]:
        """
        Create a simple path for basic characters.
        
        This is a simplified implementation. In production, you would
        load actual stroke data from a database.
        
        Args:
            character: Chinese character to write
            
        Returns:
            List of PathPoint objects
        """
        # Example stroke data for "一" (one - horizontal line)
        if character == "一":
            strokes = [
                [(50, 100), (150, 100)]
            ]
        # Example stroke data for "十" (ten - cross)
        elif character == "十":
            strokes = [
                [(100, 50), (100, 150)],  # Vertical stroke
                [(50, 100), (150, 100)]   # Horizontal stroke
            ]
        # Example stroke data for "木" (tree)
        elif character == "木":
            strokes = [
                [(100, 50), (100, 150)],   # Vertical stroke
                [(50, 100), (150, 100)],   # Horizontal stroke
                [(60, 120), (90, 150)],    # Left branch
                [(140, 120), (110, 150)]   # Right branch
            ]
        else:
            print(f"Character '{character}' not implemented")
            return []
        
        return self.plan_path(strokes, scale=1.0)
    
    def interpolate_path(self, path: List[PathPoint], 
                        step_size: float = 5.0) -> List[PathPoint]:
        """
        Interpolate path to ensure smooth motion.
        
        Args:
            path: Original path
            step_size: Maximum distance between points in mm
            
        Returns:
            Interpolated path
        """
        interpolated = []
        
        for i in range(len(path) - 1):
            current = path[i]
            next_point = path[i + 1]
            
            interpolated.append(current)
            
            if current.pen_down and next_point.pen_down:
                # Calculate distance
                dx = next_point.x - current.x
                dy = next_point.y - current.y
                distance = np.sqrt(dx**2 + dy**2)
                
                # Interpolate if distance is large
                if distance > step_size:
                    num_steps = int(np.ceil(distance / step_size))
                    for j in range(1, num_steps):
                        t = j / num_steps
                        interp_x = current.x + t * dx
                        interp_y = current.y + t * dy
                        interpolated.append(PathPoint(interp_x, interp_y, pen_down=True))
        
        # Add last point
        interpolated.append(path[-1])
        
        return interpolated
    
    def get_path_bounds(self, path: List[PathPoint]) -> Tuple[float, float, float, float]:
        """
        Get bounding box of path.
        
        Args:
            path: Path to analyze
            
        Returns:
            Tuple of (min_x, max_x, min_y, max_y)
        """
        if not path:
            return (0, 0, 0, 0)
        
        x_coords = [p.x for p in path]
        y_coords = [p.y for p in path]
        
        return (min(x_coords), max(x_coords), min(y_coords), max(y_coords))
    
    def save_path(self, path: List[PathPoint], filename: str):
        """
        Save path to file.
        
        Args:
            path: Path to save
            filename: Output filename
        """
        data = {
            'points': [
                {'x': p.x, 'y': p.y, 'pen_down': p.pen_down}
                for p in path
            ]
        }
        
        with open(filename, 'w') as f:
            json.dump(data, f, indent=2)
