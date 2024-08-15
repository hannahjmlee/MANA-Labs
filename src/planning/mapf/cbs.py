from src.planning.mapf.utils.ct import *
from src.detection.collision_detection import GridCollisionCheck
from timeout_decorator import timeout

class CBS:
    # Constructor: Initializes the CBS instance with a low-level planner and optional debug mode.
    def __init__(self, low_level_planner, debug=True):
        self.LowLevel = low_level_planner.PlanPath  # Low-level planning method
        self.PostProcess = low_level_planner.PostProcess  # Post-processing method for paths
        self.debug = debug  # Debug mode flag

    # Method: Resets the CBS instance, clearing the tasks.
    def Reset(self):
        self.tasks = []  # Clear the list of tasks

    # Method: Solves the CBS problem with a given set of tasks and an optional root node.
    @timeout(6000)
    def Solve(self, tasks, root=[]):
        self.Reset()  # Reset the instance before solving
        self.tasks = tasks  # Assign tasks
        success, node = CTSolve(root, self.Initialize, self.Validate, self.Split)
        if not success:
            return False, None, None
        solution = dict()
        for k, v in node.paths.items():
            solution[k] = self.PostProcess(v)  # Post-process each path

        return success, node.cost, solution  # Return success status, cost, and solution

    # Method: Initializes the root node of the CBS tree with initial paths and constraints.
    def Initialize(self, root=[]):
        if len(root) == 0:
            root_node = CTNode() # create an node that holds optimal path for each agent
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

    # Method: Validates a node by checking for collisions between paths.
    def Validate(self, node: CTNode):
        max_time = max(len(node.paths[r]) for r in node.paths)

        if self.debug:
            for agent, path in node.paths.items():
                print("Robot", agent, path)
            print()

        for t in range(max_time - 1): # iterate through each timestep 
            for robotA in range(len(self.tasks)): # pairwise compare each agent
                pathA = node.paths[robotA]
                posA0 = pathA[min(t, len(pathA) - 1)]
                posA1 = pathA[min(t + 1, len(pathA) - 1)]

                for robotB in range(robotA + 1, len(self.tasks)):
                    pathB = node.paths[robotB]
                    posB0 = pathB[min(t, len(pathB) - 1)]
                    posB1 = pathB[min(t + 1, len(pathB) -1)]

                    # check for collision
                    collision, positions, times = GridCollisionCheck(posA0, posA1, posB0, posB1, t, t+1)

                    if collision:
                        # create and return pair of constraints
                        constraintA = (robotA, (positions, times))
                        constraintB = (robotB, (positions, times))
                        if self.debug:
                            print("Collision found: ")
                            print("\t", constraintA)
                            print("\t", constraintB)
                        return (constraintA, constraintB)  # Return constraints causing the collision

        return None  # Return None if no collisions are found

    # Method: Splits a node into child nodes by applying constraints and generating new paths.
    def Split(self, node: CTNode, constraints):
        robotA, constraintA = constraints[0]
        robotB, constraintB = constraints[1]

        children = []

        # create child node
        childA = CTNode()
        childA.CopyParent(node)
        if constraintA in childA.constraints[robotA]:
            raise ValueError("Duplicate constraint added to child A")

        childA.constraints[robotA].add(constraintA)
        
        # replan child node path 
        min_end = self.GetMinEndTime(childA.constraints[robotA])
        success, childA.paths[robotA] = self.LowLevel(self.tasks[robotA], childA.constraints[robotA], min_end)
        if success:
            childA.cost = self.Cost(childA.paths)
            children.append(childA)

            if self.debug:
                print("Child node (1) created")

        # create second child node
        childB = CTNode()
        childB.CopyParent(node)
        if constraintB in childB.constraints[robotB]:
            raise ValueError("Duplicate constraint added to child B")
        childB.constraints[robotB].add(constraintB)

        # replan second child node path
        min_end = self.GetMinEndTime(childB.constraints[robotB])
        success, childB.paths[robotB] = self.LowLevel(self.tasks[robotB], childB.constraints[robotB], min_end)
        if success:
            childB.cost = self.Cost(childB.paths)
            children.append(childB)

            if self.debug:
                print("Child node (2) created")

        return children  # Return the list of child nodes

    # Method: Determines the minimum end time based on a set of constraints.
    def GetMinEndTime(self, constraints):
        min_time = 0
        for constraint in constraints:
            _, time = constraint
            if isinstance(time, tuple):
                min_time = max(min_time, time[0])
            else:
                min_time = max(min_time, time)

        return min_time  # Return the minimum end time
