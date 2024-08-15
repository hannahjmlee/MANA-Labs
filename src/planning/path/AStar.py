import numpy as np
from heapq import heappop, heappush, heapify
from src.utils.distance_metrics import ManhattanDistance

# A* for a grid-world that assumes a 4-neighbor movement model
class AStarSearch():
    # Constructor: Initializes the AStarSearch instance with a grid and optional debug mode.
    def __init__(self, grid, debug=True):
        self.neighbors = dict()         # Dictionary to store neighbors of each grid cell
        self.InitializeNeighbors(grid)  # Initialize neighbors based on the grid
        self.debug = debug              # Debug mode flag

    class Node:
        # Constructor: Initializes a Node with a vertex, g-cost, and h-cost.
        def __init__(self, vertex, g, h):
            self.vertex = vertex    # Grid position of the node
            self.g = g              # Cost from start to the current node
            self.h = h              # Heuristic cost estimate from the current node to the goal

        # Hash function for Node, based on its vertex and g-cost.
        def __hash__(self):
            return hash((self.vertex[0], self.vertex[1], self.g))

        # Less than operator: Defines priority in the priority queue based on f-cost (g + h).
        def __lt__(self, other):
            if self.g + self.h == other.g + other.h:
                return self.g > other.g
            return self.g + self.h < other.g + other.h

        # Greater than operator: Inverse of the less than operator.
        def __gt__(self, other):
            if self.g + self.h == other.g + other.h:
                return self.g < other.g
            return self.g + self.h > other.g + other.h

        # Equality operator: Two nodes are equal if they have the same vertex and g-cost.
        def __eq__(self, other):
            return self.vertex == other.vertex and self.g == other.g

    # Method: Plans a path from start to goal, considering constraints and end time.
    def PlanPath(self, task, constraints=set(), end_time = 0):
        start, goal = task
        if self.debug: 
            print("Solving task: start-", start, " goal-", goal, " endtime-", end_time)

        parent = dict()  # Dictionary to track the parent of each node
        seen = set()  # Set to track visited nodes

        current = self.Node(start, 0, 0)  # Start node with g=0 and h=0
        pq = [current]  # Priority queue initialized with the start node
        heapify(pq)  # Convert list to a heap

        while len(pq) > 0:
            current = heappop(pq)  # Get the node with the lowest f-cost
            if current in seen:
                continue
            seen.add(current)

            if current.vertex == goal and current.g >= end_time:
                break

            # check validity of each neighbor 
            for neighbor in self.neighbors[current.vertex]:
                vertex_constr = (neighbor, current.g + 1)
                edge_constr1 = ((current.vertex, neighbor), (current.g, current.g + 1))
                edge_constr2 = ((neighbor, current.vertex), (current.g, current.g + 1))

                if vertex_constr in constraints or edge_constr1 in constraints or edge_constr2 in constraints:
                    continue
                
                # update heuristic cost
                new_h = self.Heuristic(neighbor, goal)
                new_node = self.Node(neighbor, current.g + 1, new_h)

                heappush(pq, new_node)
                parent[new_node] = current

        if current.vertex != goal or current.g < end_time:
            if self.debug: 
                print("Path was not found")
            return False, None

        path = [current.vertex]  # Reconstruct the path from goal to start
        while current.vertex != start or current.g != 0:
            current = parent[current]
            path.append(current.vertex)
        path.reverse()

        if self.debug: 
            print("Path found:", path)

        return True, path

    # Method: Calculates the heuristic (Manhattan distance) between two positions.
    def Heuristic(self, posA, posB):
        return ManhattanDistance(posA, posB)

    # Method: Initializes neighbors for each grid cell based on the grid structure.
    def InitializeNeighbors(self, grid):
        max_y, max_x = grid.shape

        for x in range(max_x):
            for y in range(max_y):
                if not grid[(y, x)]:
                    continue

                neighbors = [(x-1, y), (x+1, y), (x, y-1), (x, y+1)]
                valid_neighbors = set()
                for neighbor in neighbors:
                    if neighbor[0] < 0 or neighbor[0] >= max_x:
                        continue
                    if neighbor[1] < 0 or neighbor[1] >= max_y:
                        continue
                    if not grid[(neighbor[1], neighbor[0])]:
                        continue
                    valid_neighbors.add(neighbor)
                valid_neighbors.add((x, y))
                self.neighbors[(x, y)] = valid_neighbors

    # Method: Post-processes the path if any additional processing is needed (currently a placeholder).
    def PostProcess(self, path): 
        return path
