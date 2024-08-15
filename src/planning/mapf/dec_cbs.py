from src.planning.mapf.utils.ct import *
from src.detection.collision_detection import GridCollisionCheck
from timeout_decorator import timeout

# Decentralized CBS is for use with PRISM
class DecentralizedCBS:
    # Constructor: Initializes the DecentralizedCBS instance with a low-level planner and optional debug mode.
    def __init__(self, low_level, debug=True):
        self.LowLevel = low_level  # Low-level planning method
        self.debug = debug         # Debug mode flag
        self.root = None           # Root node for the CBS tree

    # Method: Solves the decentralized CBS problem with the given tasks and agents.
    def Solve(self, tasks, agents, static_agents=[], root=[]):
        self.tasks = tasks  # Assign tasks
        self.agents = agents  # Assign agents
        self.static_agents = static_agents  # Assign static agents
        self.all_agents = agents + static_agents  # Combine all agents
        self.is_static = [False for _ in self.agents] + [True for _ in self.static_agents]  # Static flags for agents

        return CTSolve(root, self.Initialize, self.Validate, self.Split)  # Solve using CTSolve

    # Method: Initializes the root node with initial paths and constraints.
    def Initialize(self, root = []):
        if len(root) == 0:
            root_node = CTNode()

            for agent_num in self.agents:
                success, root_node.paths[agent_num] = self.SolveTask(self.tasks[agent_num])
                if not success:
                    return False, None
                root_node.constraints[agent_num] = set()  # Initialize constraints for each agent

            root_node.cost = self.Cost(root_node.paths)  # Compute cost of the root node
            root.append(root_node)
        else:
            for node in root:
                node.cost = self.Cost(node.paths)  # Recompute cost for existing nodes

        self.root = root[0]  # Set the first root node
        return root

    # Method: Solves a single task with given constraints and end time.
    def SolveTask(self, task, constraints=set(), end_time=0):
        extracted_constraints = set()
        for constraint in constraints:
            extracted_constraints.add(constraint[1])  # Extract relevant constraints

        return self.LowLevel(task, extracted_constraints, end_time)  # Call low-level planner

    # Method: Computes the sum-of-costs cost of a given set of paths.
    def Cost(self, paths):
        cost = 0
        for _, path in paths.items():
            cost += len(path) - 1 
        return cost

    # Method: Validates a node by checking for collisions between paths.
    def Validate(self, node: CTNode):
        max_time = max(len(node.paths[r]) for r in node.paths)

        if self.debug:
            print("\n")
            for agent, path in node.paths.items():
                print("Robot", agent, path)
                print("\tConstraints:", node.constraints[agent])

        for t in range(max_time - 1): # iterate through each timestep
            for indexA in range(len(self.all_agents)): # pariwise comparison of agents for collisions
                robotA = self.all_agents[indexA]
                pathA = node.paths[robotA]
                posA0 = pathA[min(t, len(pathA) - 1)]
                posA1 = pathA[min(t + 1, len(pathA) - 1)]

                if self.is_static[indexA] and t >= len(pathA):
                    continue

                for indexB in range(indexA + 1, len(self.all_agents)):
                    if self.is_static[indexA] and self.is_static[indexB]:
                        continue  # Skip comparisons if both agents are static

                    robotB = self.all_agents[indexB]
                    pathB = node.paths[robotB]
                    posB0 = pathB[min(t, len(pathB) - 1)]
                    posB1 = pathB[min(t + 1, len(pathB) -1)]

                    # if second agent is static and at its goal, continue
                    if self.is_static[indexB] and t >= len(pathB):
                        continue
                    
                    # check for collisions 
                    collision, positions, times = GridCollisionCheck(posA0, posA1, posB0, posB1, t, t+1)

                    if collision:
                        constraintA = (robotA, (robotB, (positions, times)))
                        constraintB = (robotB, (robotA, (positions, times)))

                        # If an agent is static, do not add constraints to it.
                        if self.is_static[indexA]:
                            constraintA = (None, (None, (None, None)))
                        if self.is_static[indexB]:
                            constraintB = (None, (None, (None, None)))

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

        # create first child
        if robotA is not None:
            childA = CTNode()
            childA.CopyParent(node)

            # check if duplicate constraint was added
            for constraint in childA.constraints[robotA]:
                if constraintA == constraint[1]:
                    raise ValueError("Duplicate constraint added to child A")

            childA.constraints[robotA].add(constraintA)

            min_end = self.GetMinEndTime(childA.constraints[robotA])
            success, childA.paths[robotA] = self.SolveTask(self.tasks[robotA], childA.constraints[robotA], min_end)
            if success:
                childA.cost = self.Cost(childA.paths)
                children.append(childA)

                if self.debug:
                    print("Child node (1) created")
                    print("\tAdding constraint to", robotA, "robot:", constraintA)
                    print("\tNew path for Robot", robotA, ":", childA.paths[robotA])

        # create second child
        if robotB is not None:
            childB = CTNode()
            childB.CopyParent(node)

            # check if duplicate constraint was added
            for constraint in childB.constraints[robotB]:
                if constraintB == constraint[1]:
                    raise ValueError("Duplicate constraint added to child B")
            childB.constraints[robotB].add(constraintB)

            min_end = self.GetMinEndTime(childB.constraints[robotB])
            success, childB.paths[robotB] = self.SolveTask(self.tasks[robotB], childB.constraints[robotB], min_end)
            if success:
                childB.cost = self.Cost(childB.paths)
                children.append(childB)

                if self.debug:
                    print("Child node (2) created")
                    print("\tAdding constraint to", robotB, "robot:", constraintB)
                    print("\tNew path for Robot", robotB, ":", childB.paths[robotB])

        return children  # Return the list of child nodes

    # Method: Determines the minimum end time based on a set of constraints.
    def GetMinEndTime(self, constraints):
        min_time = 0
        for constraint in constraints:
            _, time = constraint[1]
            if isinstance(time, tuple):
                min_time = max(min_time, time[0])
            else:
                min_time = max(min_time, time)

        return min_time  # Return the minimum end time
