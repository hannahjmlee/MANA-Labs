from src.utils.distance_metrics import EuclideanDistance
from src.utils.math_operations import GetSlopeIntercept
from math import floor, ceil
import numpy as np

class RayCasting: 
    # Constructor: Initializes the RayCasting instance with a grid and a proximity limit.
    def __init__(self, grid, limit=2.05): 
        self.grid = grid               # Grid representing the environment
        self.limit = max(limit, 2.05)  # Minimum limit for proximity checks

    # Method: Checks if two positions are within line of sight, considering proximity and strictness.
    def InRange(self, posA, posB, limit=None, strict=False): 
        if limit is None:  # Use the instance's limit if no limit is provided
            return self.CalculateLineOfSight(posA, posB, self.limit, strict)
        return self.CalculateLineOfSight(posA, posB, limit, strict)

    # Method: Calculates the line of sight grid from a given point, considering proximity limit.
    def CalculatePointLineOfSight(self, pos, limit): 
        x1, y1 = pos
        rows, cols = self.grid.shape

        los_grid = np.full((rows, cols), False)  # Initialize a grid for line of sight
        for x2 in range(cols): 
            for y2 in range(rows): 
                if x1 == x2 and y1 == y2: 
                    los_grid[(y1, x1)] = True  # The point itself is always in line of sight
                    continue
                if not self.grid[(y2, x2)]: 
                    continue
                if EuclideanDistance(pos, (x2, y2)) <= limit: 
                    los = self.CalculateLineOfSight(pos, (x2, y2), limit)
                    los_grid[(y2, x2)] = los  # Mark cells within line of sight
        return los_grid

    # Method: Determines if there is a clear line of sight between two positions, considering obstacles.
    def CalculateLineOfSight(self, posA, posB, limit, strict=False): 
        if EuclideanDistance(posA, posB) > limit: 
            return False  # Return False if distance exceeds limit
        
        if not strict and EuclideanDistance(posA, posB) <= 2.05: 
            return True  # Automatically return True for very close positions
        
        x1, y1 = posA
        x2, y2 = posB

        check_x, check_y = True, True
        if y1 == y2:  # Horizontal line
            check_x, check_y = True, False
        elif x1 == x2:  # Vertical line
            check_x, check_y = False, True

        slope, intercept = GetSlopeIntercept(posA, posB)

        min_x, max_x = min(posA[0], posB[0]), max(posA[0], posB[0])
        min_y, max_y = min(posA[1], posB[1]), max(posA[1], posB[1])

        check_cells = set()

        if check_x: 
            for grid_x in range(min_x, max_x): 
                x = grid_x + 0.5
                if slope == 0: 
                    y = min_y
                else: 
                    y = slope * x + intercept

                if abs(round(y * 2) / 2 - y) < 0.1: 
                    check_cells.add((floor(x), floor(y))) 
                    check_cells.add((ceil(x), floor(y))) 
                    check_cells.add((floor(x), ceil(y)))
                    check_cells.add((ceil(x), ceil(y))) 

                else: 
                    y = round(y)
                    check_cells.add((floor(x), y))  # Add left cell
                    check_cells.add((ceil(x), y))  # Add right cell 

        if check_y: 
            for grid_y in range(min_y, max_y): 
                y = grid_y + 0.5
                if slope is None: 
                    x = min_x
                else: 
                    x = (y - intercept) / slope

                if abs(round(x * 2) / 2 - x) < 0.1:
                    check_cells.add((floor(x), floor(y))) 
                    check_cells.add((ceil(x), floor(y))) 
                    check_cells.add((floor(x), ceil(y)))
                    check_cells.add((ceil(x), ceil(y)))                
                else: 
                    x = round(x)
                    check_cells.add((x, floor(y)))  # Add bottom cell
                    check_cells.add((x, ceil(y)))  # Add top cell

        rows, cols = self.grid.shape
        for cell in check_cells: 
            if cell[1] >= rows or cell[1] < 0: 
                continue
            elif cell[0] >= cols or cell[0] < 0:
                continue
            if not self.grid[(cell[1], cell[0])]: 
                return False  # Return False if any cell in line of sight is obstructed

        return True  # Return True if line of sight is clear
