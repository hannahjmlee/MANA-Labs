from heapq import heappush, heappop, heapify
from copy import deepcopy
import time

class CTNode:
    # Constructor: Initializes the CTNode with default values for constraints, cost, paths, and depth.
    def __init__(self):
        self.constraints = dict()  # Dictionary to hold constraints
        self.cost = 0              # Cost associated with the node
        self.paths = dict()        # Dictionary to hold paths
        self.depth = 0             # Depth of the node in the search tree

    # Method: Copies the parent node's properties into the current node, with an incremented depth.
    def CopyParent(self, other):
        self.constraints = deepcopy(other.constraints)  # Copy constraints from parent
        self.cost = other.cost                          # Copy cost from parent
        self.paths = deepcopy(other.paths)              # Copy paths from parent
        self.depth = other.depth + 1                    # Increment depth from parent

    # Method: Defines the less than comparison for heap operations based on cost.
    def __lt__(self, other):
        return self.cost < other.cost  # Compare nodes by cost

    # Method: Counts the total number of constraints in the node.
    def NumConstraints(self):
        count = 0
        for k, v in self.constraints.items():
            count += len(v)  # Sum the lengths of all constraint lists
        return count

# Function: Solves the constraint tree problem using a heap-based approach.
# Returns a tuple indicating whether a solution was found and the final node.
def CTSolve(root, initialize, validate, split):
    ct = initialize(root)  # Initialize the constraint tree
    heapify(ct)            # Convert the tree into a heap

    while len(ct) > 0:
        node = heappop(ct)  # Pop the node with the lowest cost
        constraints = validate(node)  # Validate the node

        if constraints is None:  # If no constraints, the solution is found
            return True, node

        children = split(node, constraints)  # Split the node into children
        for child in children:
            heappush(ct, child)  # Push children into the heap

    return False, None  # Return failure if no solution is found
