import numpy as np


def GetLines(filename, skip_lines):
    lines = []
    with open (filename, "r") as f:
        count = 0

        for line in f:
            count += 1
            if count <= skip_lines:
                continue
            lines.append(line.strip())
    return lines


def CreateBooleanGrid(lines):
    obst_chars = "T@O" # False
    free_chars = ".G"  # True

    grid = []
    for line in lines:
        bool_line = []
        for char in line:
            if char in obst_chars:
                bool_line.append(False)
            else:
                bool_line.append(True)
        grid.append(bool_line)

    return np.array(grid)
