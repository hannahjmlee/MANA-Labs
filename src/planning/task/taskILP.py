from copy import deepcopy
from gurobipy import Model, GRB

# Solve the task assignment problem using a task flow graph - this is not fast, but it is optimal. 
class TaskAssignmentILP():
    # Constructor: Initializes the TaskAssignmentILP with a low-level planner and optional debug mode.
    def __init__(self, low_level, debug=False):
        self.time = 0  # Current time in the simulation
        self.low_level = low_level  # Low-level planning method
        self.debug = debug  # Debug mode flag
        self.num_completed = 0  # Counter for the number of completed tasks

    # Method: Initializes the task allocator with robot IDs and tasks.
    def InitializeAllocator(self, robot_ids, tasks):
        self.robots = deepcopy(robot_ids)                                  # Deep copy of robot IDs
        self.tasks = deepcopy(tasks)                                       # Deep copy of tasks (tuple of (start, goal))
        self.task_dict = {task[1]: idx for idx, task in enumerate(tasks)}  # Map each goal to its task index
        self.completed_tasks = [False for _ in self.tasks]                 # Track completion status of tasks
        self.allocated_tasks = set()                                       # Set of allocated tasks
        self.allocation = {idx: [] for idx in self.robots}                 # Dictionary to store task allocations

        self.num_tasks = len(tasks)                                        # Total number of tasks
        self.num_robots = len(self.robots)                                 # Total number of robots
        self.available_robots = len(self.robots)                           # Number of available robots

        self.InitializeNetwork()                                           # Initialize the network with task connections
        self.BuildGraph()                                                  # Build the graph for task allocation

    # Method: Initializes the network with task-to-task travel distances.
    def InitializeNetwork(self):
        # Holds task-to-task travel distances
        self.edge_costs = dict()

        for i in range(self.num_tasks):
            for j in range(self.num_tasks):
                if i == j:
                    continue
                _, goal = self.tasks[i]
                start, _ = self.tasks[j]
                _, path = self.low_level((goal, start))

                idx_i = -1 * (i + 1)
                idx_j = -1 * (j + 1)
                self.edge_costs[(idx_i, idx_j)] = len(path)

        # Populate task vertices
        self.vertex_costs = dict()
        for id, task in enumerate(self.tasks):
            _, path = self.low_level(task)
            self.vertex_costs[-1 * (id + 1)] = len(path)

        for robot in self.robots:
            self.vertex_costs[robot + 1] = 0

    # Method: Builds the graph for the task allocation problem.
    def BuildGraph(self):
        self.source = 0  # Source vertex in the graph
        self.sink = 3 * len(self.robots) + 2 * (len(self.tasks))  # Sink vertex in the graph

        self.incoming_edges = dict()  # Dictionary to store incoming edges for each vertex
        self.outgoing_edges = dict()  # Dictionary to store outgoing edges for each vertex

        self.outgoing_edges[self.source] = []
        self.incoming_edges[self.sink] = []

        self.vertices = []  # List to store all vertices, including source and sink
        self.edges = []  # List to store all edges, including source and sink connections
        for id in self.robots:
            idx = id + 1
            self.vertices.append(idx)

            self.edges.append((self.source, idx))
            self.outgoing_edges[self.source].append((self.source, idx))

            self.edges.append((idx, self.sink))
            self.incoming_edges[self.sink].append((idx, self.sink))

            self.incoming_edges[idx] = [(self.source, idx)]
            self.outgoing_edges[idx] = [(idx, self.sink)]

        for i in range(len(self.tasks)):
            idx = -1 * (i + 1)
            self.vertices.append(idx)

            self.edges.append((idx, self.sink))
            self.incoming_edges[self.sink].append((idx, self.sink))

            self.incoming_edges[idx] = []
            self.outgoing_edges[idx] = [(idx, self.sink)]

        # Task-to-task connections
        for i in range(len(self.tasks)):
            idx_i = -1 * (i + 1)
            for j in range(i + 1, len(self.tasks)):
                idx_j = -1 * (j + 1)

                edgeA = (idx_i, idx_j)
                edgeB = (idx_j, idx_i)

                self.edges.append(edgeA)
                self.edges.append(edgeB)

                self.outgoing_edges[idx_i].append(edgeA)
                self.incoming_edges[idx_i].append(edgeB)

                self.outgoing_edges[idx_j].append(edgeB)
                self.incoming_edges[idx_j].append(edgeA)

        # Robot-to-task connections
        for i in self.robots:
            idx_i = i + 1
            for j in range(len(self.tasks)):
                idx_j = -1 * (j + 1)

                edge = (idx_i, idx_j)
                self.edges.append(edge)

                self.outgoing_edges[idx_i].append(edge)
                self.incoming_edges[idx_j].append(edge)

        self.vertices.append(self.source)
        self.vertices.append(self.sink)

        self.vertex_costs[self.source] = 0
        self.vertex_costs[self.sink] = 0

    # Method: Marks a task as completed and updates the vertex costs.
    def TaskCompleted(self, task):
        if task not in self.task_dict:
            return
        idx = self.task_dict[task]
        if not self.completed_tasks[idx]:
            self.num_completed += 1  # Increment the count of completed tasks
        self.completed_tasks[idx] = True  # Mark the task as completed
        self.vertex_costs[-1 * (idx + 1)] = 0  # Set the vertex cost to 0 for completed tasks

    # Method: Updates the system state with the current time, agents, and untasked agents.
    def UpdateSystem(self, current_time, agents, untasked_agents):
        self.time = current_time  # Update the current time
        self.allocated_tasks.clear()  # Clear allocated tasks

        if self.num_completed == self.num_tasks:
            self.allocation = dict()
            for id in self.robots:
                self.allocation[id] = []
            return

        for agent in agents.values():
            # Default cost to 0 - captures transitioning agents
            self.vertex_costs[agent.id + 1] = 0
            start = agent.GetCurrentPosition(self.time)

            # Update cost if agent is completing a true task
            if agent.OnTrueTask():
                start = agent.goal
                if agent not in untasked_agents:
                    self.vertex_costs[agent.id + 1] = agent.GetPathDuration(self.time)
                    self.allocated_tasks.add(-1 * (self.task_dict[agent.goal] + 1))

            # Calculate robot-to-task edge costs
            for idx, task in enumerate(self.tasks):
                if self.completed_tasks[idx]:
                    continue
                goal, _ = task
                _, path = self.low_level((start, goal))
                self.edge_costs[(agent.id + 1, -1 * (idx + 1))] = len(path)

            # Source-to-robot edges
            self.edge_costs[(self.source, agent.id + 1)] = self.time

    # Method: Extracts the indices of completed tasks.
    def ExtractCompleted(self):
        completed = []
        for i, is_complete in enumerate(self.completed_tasks):
            if is_complete:
                completed.append(-1 * (i + 1))
        return completed

    # Method: Solves the task allocation problem using Integer Linear Programming (ILP).
    def Solve(self):
        if self.num_completed == self.num_tasks:
            self.allocation = dict()
            for id in self.robots:
                self.allocation[id] = []
            return True

        completed_tasks = self.ExtractCompleted()

        m = Model("TaskAllocation")
        if not self.debug:
            m.setParam('OutputFlag', 0)

        # Flow variable for each edge
        f = {e: m.addVar(vtype=GRB.BINARY, name=f"flow_{e}", lb=0) for e in self.edges}

        # Arrival time for each vertex
        t = {v: m.addVar(vtype=GRB.INTEGER, name=f"time_{v}", lb=0) for v in self.vertices}

        # Capacity constraint on each edge
        for e in self.edges:
            m.addConstr(f[e] <= 1, name=f"cap_{e}")

        # Flow conservation at source
        for r in self.robots:
            v_id = r + 1
            m.addConstr(f[(self.source, v_id)] == 1, f"outflow_{v_id}")

        # Flow conservation at sink
        m.addConstr(sum(f[e] for e in self.incoming_edges[self.sink]) == self.available_robots, "flow_conservation_sink")

        # Flow conservation at all other vertices
        for v in self.vertices:
            if v == self.source or v == self.sink:
                continue
            flow_value = 1

            if v in completed_tasks or v in self.allocated_tasks:
                flow_value = 0

            m.addConstr(sum(f[e] for e in self.incoming_edges[v]) == flow_value, f"flow_conservation_in_{v}")
            m.addConstr(sum(f[e] for e in self.outgoing_edges[v]) == flow_value, f"flow_conservation_out_{v}")

        # Arrival time for each vertex
        for v in self.vertices:
            if v == self.source or v == self.sink:
                continue
            if v in completed_tasks or v in self.allocated_tasks:
                continue

            m.addConstr(t[v] == sum((t[u] + self.vertex_costs[u] + self.edge_costs.get((u, v), 0)) * f[(u, v)]
                                    for (u, v) in self.incoming_edges[v]), f"arrival_time_{v}")

        # Arrival time for sink
        for e in self.incoming_edges[self.sink]:
            u, _ = e
            m.addConstr(t[self.sink] >= (t[u] + self.vertex_costs[u] + self.edge_costs.get(e, 0)) * f[e], f"makespan_sink")

        m.setObjective(t[self.sink], GRB.MINIMIZE)
        m.Params.Cuts = 3
        m.Params.NonConvex = 2

        time_limit = 10
        for _ in range(10):
            # Optimize model
            m.Params.TimeLimit = time_limit
            m.optimize()

            if m.status != GRB.OPTIMAL and m.status != GRB.TIME_LIMIT:
                for c in m.getConstrs():
                    if c.IISConstr:
                        print("\t", c.constrName)

                print("No solution found.")
                if self.debug:
                    m.computeIIS()

                return False

            if self.debug:
                if m.status == GRB.TIME_LIMIT:
                    print("Suboptimal solution returned")
                else:
                    print("Optimal solution returned")
                print(self.allocation)

            try:
                for start_edge in self.outgoing_edges[self.source]:
                    if f[start_edge].X == 0:
                        continue

                    u, v = start_edge
                    true_robot_id = v - 1

                    allocation = []

                    while v != self.sink:
                        for e in self.outgoing_edges[v]:
                            if f[e].X > 0:
                                u, v = e
                                if v != self.sink:
                                    allocation.append(self.tasks[(-1 * v) - 1])
                                break

                    self.allocation[true_robot_id] = allocation
                return True
            except:
                time_limit += 5
        return False

    # Method: Returns the number of completed tasks.
    def NumCompleted(self):
        return sum(self.completed_tasks)

    # Method: Returns the task assigned to a given agent.
    def GetTask(self, agent_id):
        if len(self.allocation[agent_id]) == 0:
            return None
        new_task = self.allocation[agent_id][0]
        return new_task

    # Method: Removes and returns the task assigned to a given agent.
    def PopTask(self, agent_id):
        if len(self.allocation[agent_id]) == 0:
            return
        task = self.allocation[agent_id].pop(0)

    # Method: Returns all allocated tasks as a set.
    def GetAllAllocationTasks(self):
        current_tasks = set()
        for tasks in self.allocation.values():
            for task in tasks:
                current_tasks.add(task[1])
        return current_tasks

    # Method: Finds the original task corresponding to a given goal.
    def FindTrueTask(self, goal):
        idx = self.task_dict[goal]
        return self.tasks[idx]
