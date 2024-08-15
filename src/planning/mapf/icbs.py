from src.planning.mapf.utils.ct import *
from src.detection.collision_detection import GridCollisionCheck
from timeout_decorator import timeout

# Improved Conflict Based Search
class ICBS:
    # Constructor: Initializes the ICBS instance with a low-level planner and optional debug mode.
    def __init__(self, low_level, debug=True):
        self.LowLevel = low_level  # Low-level planning method
        self.debug = debug  # Debug mode flag
        self.path_cache = [None, None]  # Cache for storing paths during conflict resolution
        self.bypass_node = None  # Node used for bypassing conflicts

    # Method: Resets the ICBS instance, clearing tasks.
    def Reset(self): 
        self.tasks = []  # Clear the list of tasks

    # Method: Solves the ICBS problem with the given tasks and an optional root node.
    def Solve(self, tasks, root=[]):
        self.Reset()  # Reset the instance before solving
        self.tasks = tasks  # Assign tasks
        success, node = CTSolve(root, self.Initialize, self.Validate, self.Split)
        return success, node.cost, node.paths  # Return success status, cost, and paths

    # Method: Initializes the root node with initial paths and constraints.
    def Initialize(self, root = []):
        if len(root) == 0:
            root_node = CTNode()
            for agent_num in range(len(self.tasks)):
                success, root_node.paths[agent_num] = self.LowLevel(self.tasks[agent_num])
                if not success:
                    return False, None
                root_node.constraints[agent_num] = set()  # Initialize constraints for each agent

            root_node.cost = self.Cost(root_node.paths)  # Compute cost of the root node
            root.append(root_node)

        return root

    # Method: Computes the total cost of a given set of paths.
    def Cost(self, paths):
        cost = 0
        for _, path in paths.items():
            cost += len(path) - 1  # Sum the length of all paths minus one
        return cost

    # Method: Validates a node by checking for collisions between paths and determining the best constraints.
    def Validate(self, node: CTNode):
        max_time = max(len(node.paths[r]) for r in node.paths)

        if self.debug: 
            for agent, path in node.paths.items(): 
                print("Robot", agent, path)
            print()

        constraints = []
        for t in range(max_time - 1):
            for robotA in range(len(self.tasks)):
                pathA = node.paths[robotA]
                posA0 = pathA[min(t, len(pathA) - 1)]
                posA1 = pathA[min(t + 1, len(pathA) - 1)]

                for robotB in range(robotA + 1, len(self.tasks)):
                    pathB = node.paths[robotB]
                    posB0 = pathB[min(t, len(pathB) - 1)]
                    posB1 = pathB[min(t + 1, len(pathB) -1)]

                    collision, positions, times = GridCollisionCheck(posA0, posA1, posB0, posB1, t, t+1)

                    if collision:
                        constraintA = (robotA, (positions, times))
                        constraintB = (robotB, (positions, times))
                        if self.debug: 
                            print("Collision found: ")
                            print("\t", constraintA)
                            print("\t", constraintB)

                        constraints.append((constraintA, constraintB))

        if len(constraints) == 0: 
            return None

        cost_map = dict()
        for robot in range(len(self.tasks)): 
            cost_map[robot] = len(node.paths[robot])
        
        constraint_type = -1
        best_constraints = [None, None]
        num_conflicts = len(constraints)
        self.bypass_node = None
        self.path_cache = [None, None]

        # find best constraint based on cardinality and bypass
        for constraint_set in constraints: 
            robotA, constraint = constraint_set[0]
            robotB, _ = constraint_set[1]

            constraintsA = deepcopy(node.constraints[robotA])
            constraintsA.add(constraint)
            constraintsB = deepcopy(node.constraints[robotB])
            constraintsB.add(constraint)

            min_endA = self.GetMinEndTime(constraintsA)
            min_endB = self.GetMinEndTime(constraintsB)

            successA, pathA = self.LowLevel(self.tasks[robotA], constraintsA, min_endA)
            successB, pathB = self.LowLevel(self.tasks[robotB], constraintsB, min_endB)
            
            if not successA or not successB: 
                continue

            # find cardinality of constraints
            cardinality = 0
            if len(pathA) > cost_map[robotA]: 
                cardinality += 1
            if len(pathB) > cost_map[robotB]: 
                cardinality += 1
            
            # find if node can be bypassed
            if cardinality <= 1: 
                if len(pathA) == cost_map[robotA]: 
                    success, child = self.Bypass(node, robotA, pathA, num_conflicts)
                    if success: 
                        self.bypass_node = child
                        constraint_type = -1
                if len(pathB) == cost_map[robotB]: 
                    success, child = self.Bypass(node, robotB, pathB, num_conflicts)
                    if success: 
                        self.bypass_node = child
                        constraint_type = -1

            # save the highest cardinality constraint set
            if cardinality > constraint_type: 
                best_constraints[0] = constraint_set[0]
                best_constraints[1] = constraint_set[1]

                self.path_cache[0] = pathA
                self.path_cache[1] = pathB

                constraint_type = cardinality

            if self.bypass_node is not None or cardinality == 2: 
                break
            

        if constraint_type != -1: 
            return (best_constraints[0], best_constraints[1])

        return None
    
    # Method: Attempts to bypass conflicts by updating the path of a given robot.
    def Bypass(self, parent_node: CTNode, robot, path, num_conflicts): 
        child = CTNode()
        child.CopyParent(parent_node)
        child.paths[robot] = path
        success = False

        if self.CountConflicts(child.paths) < num_conflicts: 
            success = True

        return success, child

    # Method: Counts the number of conflicts in a given set of paths.
    def CountConflicts(self, path_map): 
        count = 0
        max_time = max(len(path_map[r]) for r in path_map)

        for t in range(max_time - 1):
            for robotA in range(len(self.tasks)):
                pathA = path_map[robotA]
                posA0 = pathA[min(t, len(pathA) - 1)]
                posA1 = pathA[min(t + 1, len(pathA) - 1)]

                for robotB in range(robotA + 1, len(self.tasks)):
                    pathB = path_map[robotB]
                    posB0 = pathB[min(t, len(pathB) - 1)]
                    posB1 = pathB[min(t + 1, len(pathB) -1)]

                    collision, _, _ = GridCollisionCheck(posA0, posA1, posB0, posB1, t, t+1)

                    if collision:
                        count += 1

        return count

    # Method: Splits a node into child nodes by applying constraints and generating new paths.
    def Split(self, node:CTNode, constraints):
        if self.bypass_node is not None: 
            return [self.bypass_node]

        robotA, constraintA = constraints[0]
        robotB, constraintB = constraints[1]

        children = []

        # create first child
        childA = CTNode()
        childA.CopyParent(node)
        if constraintA in childA.constraints[robotA]:
            raise ValueError("Duplicate constraint added to child A")

        childA.constraints[robotA].add(constraintA)

        # update path of first child from path cache
        if self.path_cache[0] is not None: 
            childA.paths[robotA] = self.path_cache[0]
            childA.cost = self.Cost(childA.paths)
            children.append(childA)

            if self.debug: 
                print("Child node (1) created")

        # create second child
        childB = CTNode()
        childB.CopyParent(node)
        if constraintB in childB.constraints[robotB]:
            raise ValueError("Duplicate constraint added to child B")
        childB.constraints[robotB].add(constraintB)

        # update path of second child from path cache
        if self.path_cache[1] is not None: 
            childB.paths[robotB] = self.path_cache[1]
            childB.cost = self.Cost(childB.paths)
            children.append(childB)

            if self.debug: 
                print("Child node (2) created")

        return children

    # Method: Determines the minimum end time based on a set of constraints.
    def GetMinEndTime(self, constraints):
        min_time = 0
        for constraint in constraints:
            _, time = constraint
            if isinstance(time, tuple):
                min_time = max(min_time, time[0])
            else:
                min_time = max(min_time, time)

        return min_time
