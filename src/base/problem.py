import numpy as np

class Problem:
    # Constructor: Initializes the Problem instance with a boolean grid and an optional map name.
    def __init__(self, bool_grid, map_name = ""):
        self.grid = bool_grid                           # Boolean grid representing the environment
        self.map_name = map_name                        # Optional name for the map
        self.num_cols, self.num_rows = bool_grid.shape  # Grid dimensions (columns, rows)
        self.queries = []                               # List to store start-goal queries

    # Method: Loads a list of scenarios from provided lines, parsing start and goal positions.
    def LoadScenario(self, lines):
        self.queries = []  # Clear existing queries before loading new ones
        for line in lines:
            split_line = line.split()  
            start = (int(split_line[4]), int(split_line[5]))  # Parse start position (x, y)
            goal = (int(split_line[6]), int(split_line[7]))  # Parse goal position (x, y)
            self.queries.append((start, goal))  
