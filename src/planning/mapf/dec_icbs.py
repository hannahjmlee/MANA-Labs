from src.planning.mapf.utils.ct import *
from src.detection.collision_detection import GridCollisionCheck
from timeout_decorator import timeout

# Decentralized ICBS for use with PRISM
class DecentralizedICBS:
    # Constructor: Initializes the DecentralizedICBS instance with a low-level planner and optional debug mode.
    def __init__(self, low_level, debug=True):
        self.LowLevel = low_level       # Low-level planning method
        self.debug = debug              # Debug mode flag
        self.path_cache = [None, None]  # Cache for storing paths during conflict resolution
        self.bypass_node = None         # Node used for bypassing conflicts

    # Method: Resets the DecentralizedICBS instance, clearing cached paths and bypass node.
    def Reset(self):
        self.path_cache = [None, None]  # Reset path cache
        self.bypass_node = None         # Reset bypass node

    # Method: Solves the decentralized ICBS problem with the given tasks and agents.
    @timeout(600)
    def Solve(self, tasks, agents, static_agents=[], root=[]):
        self.Reset()  # Reset the instance before solving
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

        return root

    # Method: Solves a single task with given constraints and end time.
    def SolveTask(self, task, constraints=set(), end_time=0):
        extracted_constraints = set()
        for constraint in constraints:
            extracted_constraints.add(constraint[1])  # Extract relevant constraints

        return self.LowLevel(task, extracted_constraints, end_time)  # Call low-level planner

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
        # iterate through each timestep
        for t in range(max_time - 1):
            for indexA in range(len(self.all_agents)): # pairwise comparison of all agents
                robotA = self.all_agents[indexA]
                pathA = node.paths[robotA]
                posA0 = pathA[min(t, len(pathA) - 1)]
                posA1 = pathA[min(t + 1, len(pathA) - 1)]

                if self.is_static[indexA] and t >= len(pathA):
                    continue

                for indexB in range(indexA + 1, len(self.all_agents)):
                    if self.is_static[indexA] and self.is_static[indexB]:
                        continue  # Skip comparisons between two static paths

                    robotB = self.all_agents[indexB]
                    pathB = node.paths[robotB]
                    posB0 = pathB[min(t, len(pathB) - 1)]
                    posB1 = pathB[min(t + 1, len(pathB) -1)]

                    if self.is_static[indexB] and t >= len(pathB):
                        continue

                    # check for collisions
                    collision, positions, times = GridCollisionCheck(posA0, posA1, posB0, posB1, t, t+1)

                    if collision:
                        constraintA = (robotA, (robotB, (positions, times)))
                        constraintB = (robotB, (robotA, (positions, times)))

                        # If an agent is static, do not add constraints to it.
                        if self.is_static[indexA]:
                            constraintA = (None, (None, (None, None)))  # robot, (parent, position)
                        if self.is_static[indexB]:
                            constraintB = (None, (None, (None, None)))

                        if self.debug:
                            print("Collision found: ")
                            print("\t", constraintA)
                            print("\t", constraintB)

                        # store all constraints
                        constraints.append((constraintA, constraintB))

        if len(constraints) == 0:
            return None

        cost_map = dict()
        for robot, path in node.paths.items():
            cost_map[robot] = len(path)

        constraint_type = -1
        best_constraints = [None, None]
        self.path_cache = [None, None]
        num_conflicts = len(constraints)
        self.bypass_node = None

        # iterate through constraints and find best constraint to apply
        for constraint_set in constraints:
            robotA, cA = constraint_set[0]
            robotB, cB = constraint_set[1]

            successA, successB = True, True
            pathA, pathB = None, None
            if cA[0] is not None:
                constraintsA = deepcopy(node.constraints[robotA])
                constraintsA.add(cA)

                min_endA = self.GetMinEndTime(constraintsA)
                successA, pathA = self.SolveTask(self.tasks[robotA], constraintsA, min_endA)
            if cB[0] is not None:
                constraintsB = deepcopy(node.constraints[robotB])
                constraintsB.add(cB)

                min_endB = self.GetMinEndTime(constraintsB)
                successB, pathB = self.SolveTask(self.tasks[robotB], constraintsB, min_endB)

            if not successA or not successB:
                continue

            # check for cardinal conflict
            cardinality = 0
            if pathA is not None and len(pathA) > cost_map[robotA]:
                cardinality += 1
            if pathB is not None and len(pathB) > cost_map[robotB]:
                cardinality += 1

            # check for bypass node
            if cardinality <= 1:
                if pathA is not None and len(pathA) == cost_map[robotA]:
                    success, child = self.Bypass(node, robotA, pathA, num_conflicts)
                    if success:
                        self.bypass_node = child
                        constraint_type = -1
                if pathB is not None and len(pathB) == cost_map[robotB]:
                    success, child = self.Bypass(node, robotB, pathB, num_conflicts)
                    if success:
                        self.bypass_node = child
                        constraint_type = -1

            # save constraints with best cardinality
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
            for indexA in range(len(self.all_agents)):
                robotA = self.all_agents[indexA]
                pathA = path_map[robotA]
                posA0 = pathA[min(t, len(pathA) - 1)]
                posA1 = pathA[min(t + 1, len(pathA) - 1)]

                for indexB in range(indexA + 1, len(self.all_agents)):
                    if self.is_static[indexA] and self.is_static[indexB]:
                        continue  # Skip comparisons between static paths

                    robotB = self.all_agents[indexB]
                    pathB = path_map[robotB]
                    posB0 = pathB[min(t, len(pathB) - 1)]
                    posB1 = pathB[min(t + 1, len(pathB) -1)]

                    collision, _, _ = GridCollisionCheck(posA0, posA1, posB0, posB1, t, t+1)

                    if collision:
                        count += 1

        return count

