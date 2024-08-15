from src.planning.mapf.dec_cbs import *
from src.planning.mapf.utils.ct import CTNode
from src.planning.task.utils.prism_datastructures import *

from copy import deepcopy
from timeout_decorator import timeout
import time

class PRISM:  # Pathfinding with Rapid Information Sharing using Motion Constraints
    # Constructor: Initializes the PRISM instance with a low-level planner, network connector, task allocator, and optional debug mode.
    def __init__(self, low_level, network_connector, task_allocator, debug=True):
        self.name = "prism"
        
        self.LowLevel = low_level  # Low-level planning method
        self.debug = debug  # Debug mode flag
        self.CBS = DecentralizedCBS(self.LowLevel, self.debug)  # Centralized CBS solver
        self.InRange = network_connector  # Network connector for checking proximity
        self.task_allocator = task_allocator  # Task allocator

        self.track_paths = False  # Flag for tracking paths
        self.allocation_max = 1  # Maximum allocation rounds
        self.Reset()

    # Method: Resets the PRISM instance to its initial state.
    def Reset(self):
        self.current_time = 0           # Current time in the simulation
        self.task_allocation_time = 0   # Time spent on task allocation

        self.agents = dict()            # Dictionary to store agents
        self.completed_agents = dict()  # Dictionary to track completed agents
        self.agent_ids = []             # List of agent IDs

        self.cc = []                    # List of connected components
        self.cc_cache = []              # Cache of previous connected components

        self.path_solutions = dict()    # Dictionary to store path solutions
        self.task_tracker = dict()      # Dictionary to track tasks
        self.path_tracker = dict()      # Dictionary to track paths
        self.completed_tracker = []     # List to track completed tasks

        self.allocation_count = -1      # Counter for task allocation rounds

        self.total_tasks = 0            # Total number of tasks
        self.completed_goals = set()    # Set of completed goals

    # Method: Solves the task allocation and pathfinding problem for a given set of tasks and agents.
    @timeout(1800)
    def Solve(self, tasks, num_agents):
        self.Initialize(num_agents, tasks)

        if self.debug:
            print("Starting Solve...")

        while self.task_allocator.num_completed < self.total_tasks or sum(self.completed_agents.values()) < len(self.agents):
            if self.debug:
                print(f"\n\n..........................................................................................")
                print(f"Current Time: {self.current_time}.........................................................")
            needs_task = []
            received_task = []

            for agent_id in self.agent_ids:
                self.Step(self.agents[agent_id], needs_task, received_task)

            not_tasked = []
            all_tasks = []
            if len(needs_task) > 0 or sum(self.completed_agents.values()) == len(self.agents):
                allocated_tasks = self.AllocateTasks(needs_task)
                if not allocated_tasks:
                    return False, None, None

                all_tasks = self.task_allocator.GetAllAllocationTasks()
                for agent_id, agent in self.agents.items():
                    task = self.task_allocator.GetTask(agent_id)
                    if task is None:
                        if not agent.at_rest:
                            not_tasked.append(agent_id)
                        continue

                    assign_task = False
                    if agent.at_rest:
                        # Agent is at rest and can take on a new task
                        assign_task = True
                    elif agent_id in needs_task:
                        # Agent has requested a task
                        assign_task = True
                    elif not agent.OnTrueTask() and agent.GetTrueTask()[1] != task[1]:
                        # Agent is traveling to a different task and can be reassigned
                        if agent.GetTrueTask()[1] in all_tasks:
                            # If it exists in the task stack, it has been reassigned
                            assign_task = True

                    if assign_task:
                        self.task_allocator.PopTask(agent_id)
                        agent.SetTask(task, self.current_time, False)
                        _, agent.path = self.LowLevel(agent.tasks[agent.current_task])
                        received_task.append(agent_id)

                        if self.debug:
                            print(f"\tAgent {agent_id} received new task: {task}")

                for agent_id in not_tasked:
                    agent = self.agents[agent_id]
                    if agent_id in needs_task:
                        # Agent requested a task but did not receive one
                        agent.SetTask((agent.goal, agent.goal), self.current_time, True)
                        agent.path = [agent.goal]
                        if self.debug:
                            print(f"\tAgent {agent_id} did not receive a new task")
                    elif not agent.OnTrueTask() and agent.GetTrueTask()[1] in all_tasks:
                        # Agent was traveling but its task was reassigned to a different agent
                        agent.SetTask((agent.previous_goal, agent.previous_goal), self.current_time, True)
                        if len(agent.tasks) > 1:
                            _, agent.path = self.LowLevel(agent.tasks[agent.current_task])
                        else:
                            agent.path = [agent.goal]

                        if self.debug:
                            print(f"\tAgent {agent_id} did not receive a new task")

            self.DetermineCompletedAgents()

            self.cc_cache = deepcopy(self.cc)
            self.cc = self.FindConnectedComponents()

            # Synchronize agents who have received a new task or those that were not given a task
            sync_agents = set(received_task + not_tasked)
            replan_cc = self.Synchronize(sync_agents)

            for i, replan in enumerate(replan_cc):
                if not replan:
                    continue

                success = self.Replan(self.cc[i])
                if not success:
                    return False, None, None

            self.current_time += 1
            for agent_id in self.completed_agents:
                if self.completed_agents[agent_id]:
                    if self.debug:
                        print(f"\tAgent {agent_id} incrementing start time")
                    self.agents[agent_id].start_time += 1

            if self.track_paths:
                self.completed_tracker.append(deepcopy(self.completed_tracker[self.current_time - 1]))

        return True, self.path_solutions, self.task_allocation_time

    # Method: Initializes the agents and task allocator with the given number of agents and tasks.
    def Initialize(self, num_agents, tasks):
        self.Reset()
        num_agents = min(num_agents, len(tasks))

        # Initialize data structures with agents
        for i in range(num_agents):
            self.path_solutions[i] = []
            self.completed_agents[i] = False
            self.agent_ids.append(i)

            self.task_tracker[i] = []

            if self.track_paths:
                self.path_tracker[i] = []

        if self.track_paths:
            self.completed_tracker = [set()]

        self.total_tasks = len(tasks)

        if self.debug:
            print("Initializing Task Allocator...")

        # Initialize task allocator
        start_time = time.time()
        self.task_allocator.InitializeAllocator(self.agent_ids, tasks)
        end_time = time.time()
        self.task_allocation_time += end_time - start_time

        if self.debug:
            print("Done initializing task allocator...")

        # Initialize agents with naive optimal paths
        for i in self.agent_ids:
            task = tasks[i]
            _, path = self.LowLevel(task)
            self.agents[i] = PRISMAgent(i, task, path)

        if self.debug:
            print("Done initializing agents...")

    # Method: Executes a step in the simulation for a given agent.
    def Step(self, agent, needs_task, received_task):
        position = agent.GetCurrentPosition(self.current_time)
        self.path_solutions[agent.id].append(position)

        if self.debug:
            print(f"\tStepping for Agent {agent.id}... {position}, goal: {agent.GetTrueTask()[1]}")

        if self.completed_agents[agent.id]:
            if self.track_paths:
                self.task_tracker[agent.id].append((False, agent.goal))
                self.path_tracker[agent.id].append([agent.goal])
            return

        if self.track_paths:
            true_task = self.task_allocator.FindTrueTask(agent.GetTrueTask()[1])
            if agent.OnTrueTask():
                self.task_tracker[agent.id].append((False, true_task[0], true_task[1]))
            else:
                self.task_tracker[agent.id].append((True, true_task[0], true_task[1]))

            subpath = deepcopy(agent.GetCurrentPath(self.current_time))
            self.path_tracker[agent.id].append(subpath)

        # Agent has completed its task
        if position == agent.goal and agent.OnLastPosition(self.current_time):
            agent.constraints.clear()
            agent.info_packets.clear()

            # Agent has another task to complete
            if len(agent.tasks) > 1 and agent.current_task == 0:
                if agent.at_rest:
                    agent.current_task = 0
                    agent.tasks = [(agent.goal, agent.goal)]
                    agent.start_time = self.current_time
                    agent.path = [agent.goal]

                    if self.debug:
                        print(f"\t\tAgent {agent.id} has reached its safe resting position.")
                else:
                    agent.current_task += 1
                    task = agent.tasks[agent.current_task]
                    agent.goal = task[1]

                    agent.start_time = self.current_time
                    _, agent.path = self.LowLevel(task)
                    received_task.append(agent.id)

                    if self.debug:
                        print(f"\t\tAgent {agent.id} has moved on to its next task.")
            else:  # Agent is done with all tasks
                agent.previous_goal = agent.goal

                if self.track_paths:
                    true_task = self.task_allocator.FindTrueTask(agent.goal)
                    self.completed_tracker[self.current_time].add(true_task)

                if not agent.at_rest:
                    self.task_allocator.TaskCompleted(agent.goal)
                    agent.previous_goal = agent.goal
                    needs_task.append(agent.id)

                    if self.debug:
                        print(f"\t\tAgent {agent.id} has completed its task {agent.goal} and is requesting a new task.")
                        print(f"\t\tCompleted {self.task_allocator.completed_tasks}")
        else:
            remove_indices = []
            for i, packet in enumerate(agent.info_packets):
                if packet.flush_time >= self.current_time:
                    remove_indices.append(i)

            if self.debug and len(agent.info_packets) > 0:
                print(f"\t\tAgent {agent.id} is flushing {len(remove_indices)}/{len(agent.info_packets)} packets.")

            for idx in sorted(remove_indices, reverse=True):
                agent.info_packets.pop(idx)

        return

    # Method: Determines which agents have completed their tasks and are at rest.
    def DetermineCompletedAgents(self):
        self.completed_agents = dict()
        for agent_id, agent in self.agents.items():
            if agent.at_rest and agent.OnLastPosition(self.current_time):
                self.completed_agents[agent_id] = True
            else:
                self.completed_agents[agent_id] = False

        if self.debug:
            print("Completed agents:")
            for agent_id in self.agents:
                print(f"\tAgent {agent_id}:", self.completed_agents[agent_id])
        return

    # Method: Finds connected components of agents based on proximity.
    def FindConnectedComponents(self):
        cc = []

        for i in range(len(self.agent_ids)):
            id_i = self.agent_ids[i]
            agent_i = self.agents[id_i]
            pos_i = agent_i.GetCurrentPosition(self.current_time)

            # Find all agents that agent i is connected to
            connected_agents = [id_i]
            for j in range(i + 1, len(self.agent_ids)):
                id_j = self.agent_ids[j]
                agent_j = self.agents[id_j]
                pos_j = agent_j.GetCurrentPosition(self.current_time)

                if self.InRange(pos_i, pos_j):
                    connected_agents.append(id_j)

            # Connect and merge connected components
            add_cc = set()
            for agent_id in connected_agents:
                added = False
                # Track which components each of the agents agent i is connected to
                for component_idx, component in enumerate(cc):
                    if agent_id in component:
                        add_cc.add(component_idx)
                        added = True
                        break

                # If the agent it's connected to is not in a component, create a new one
                if not added:
                    cc.append({agent_id})
                    add_cc.add(len(cc) - 1)

            if len(add_cc) == 0:
                # Agent i is not connected to any agents. Create a new component
                cc.append({id_i})
            elif len(add_cc) == 1:
                # Agent i is only connected to one component
                cc[add_cc.pop()].add(id_i)
            else:
                # Agent i is connected to multiple components, and they need to be merged
                merged_cc = set()
                add_cc = list(add_cc)
                add_cc.sort(reverse=True)
                # Delete the components that are being merged together
                for cc_id in add_cc:
                    merged_cc = merged_cc.union(cc[cc_id])
                    del cc[cc_id]
                cc.append(merged_cc)

        cc.sort()
        return cc

    # Method: Synchronizes agents by determining which connected components need to be replanned.
    def Synchronize(self, sync_agents):
        replan = [False for _ in self.cc]

        # Determine which components should be replanned based on newly tasked and untasked agents
        for agent_id in sync_agents:
            for i, component in enumerate(self.cc):
                if agent_id in component:
                    if len(component) > 1:
                        replan[i] = True
                    break

        for i in range(len(self.agent_ids)):
            id_i = self.agent_ids[i]

            for j in range(i + 1, len(self.agent_ids)):
                id_j = self.agent_ids[j]

                # Were agents i and j in the same component in the previous iteration?
                previous_same = False
                for cc in self.cc_cache:
                    if id_i in cc and id_j in cc:
                        previous_same = True
                        break

                # Are agents i and j in the same component right now?
                current_same = False
                current_cc_idx = -1
                for cc_idx, cc in enumerate(self.cc):
                    if id_i in cc and id_j in cc:
                        current_same = True
                        current_cc_idx = cc_idx
                        break

                if current_same == previous_same:
                    # If agents remain in the same component or different components
                    # don't do anything
                    continue
                elif not previous_same and current_same:
                    # Agents were in different components but now they're in the same component
                    if len(self.cc[current_cc_idx]) > 1:
                        replan[current_cc_idx] = True
                elif previous_same and not current_same:
                    # If the agents were in the same component but now they're not, exchange info packets
                    # If either agent is done with its tasks, then don't exchange
                    if self.completed_agents[id_i] or self.completed_agents[id_j]:
                        continue

                    agent_i = self.agents[id_i]
                    agent_j = self.agents[id_j]
                    flush_time = self.CalculateFlushTime(agent_i, agent_j)

                    # Agents do not influence each other, so don't make an info packet
                    if flush_time == 0:
                        continue

                    # Create and share info packets
                    flush_time = flush_time + self.current_time
                    i_to_j = InfoPacket(agent_i, self.current_time, flush_time, agent_i.GetPathDuration(self.current_time))
                    j_to_i = InfoPacket(agent_j, self.current_time, flush_time, agent_j.GetPathDuration(self.current_time))

                    agent_i.info_packets.append(j_to_i)
                    agent_j.info_packets.append(i_to_j)

        if self.debug:
            print("Current connected components...")
            for i, to_replan in enumerate(replan):
                if len(self.cc[i]) > 1:
                    print(f"\tComponent: {self.cc[i]}")
                    print("\t\tReplan:", to_replan)
            print("\tIndividual Components: ", end="")
            for i, to_replan in enumerate(replan):
                if len(self.cc[i]) == 1:
                    print(f"{self.cc[i]}", end=" ")
            print()
        return replan

    # Method: Replans the paths for agents in a given connected component.
    def Replan(self, cc):
        if len(cc) == 1:
            return True

        root_node = CTNode()
        tasks = dict()
        packets = dict()

        for agent_id in cc:
            agent = self.agents[agent_id]

            # Populate root node
            if self.completed_agents[agent_id]:
                root_node.paths[agent_id] = [agent.goal]
                root_node.constraints[agent_id] = set()
                tasks[agent.id] = (agent.goal, agent.goal)
            else:
                root_node.paths[agent_id] = deepcopy(agent.GetCurrentPath(self.current_time))
                root_node.constraints[agent_id] = deepcopy(agent.GetConstraints(self.current_time))
                tasks[agent.id] = (agent.GetCurrentPosition(self.current_time), agent.goal)

            # Remove packets describing agents present in the connected component
            flush_packets = []
            for i, packet in enumerate(agent.info_packets):
                if packet.id in cc:
                    flush_packets.append(i)
                    continue

                # Save only new or more recent packets
                if packet.id not in packets:
                    packets[packet.id] = packet
                elif packet.received_time > packets[packet.id].received_time:
                    packets[packet.id] = packet

            flush_packets.sort(reverse=True)
            for idx in flush_packets:
                del agent.info_packets[idx]

        # Reconstruct info packet paths and populate root node
        static_agents = []
        for id, packet in packets.items():
            static_agents.append(id)
            _, full_path = self.LowLevel(packet.task, packet.constraints, packet.end_time)
            path = full_path[packet.GetCurrentPathIndex(self.current_time):]

            root_node.paths[id] = deepcopy(path)
            root_node.constraints[id] = deepcopy(packet.GetConstraints(self.current_time))

            tasks[id] = (path[0], path[-1])

        if self.debug:
            print("Solving...")
            print(f"\tActive Agents: {cc}")
            print(f"\tStatic Agents: {static_agents}")
            print(f"\tTasks:")
            for a_id, t in tasks.items():
                print(f"\t\tAgent {a_id}: {t}, {self.completed_agents[a_id]}")
                print(f"\t\tPath: {root_node.paths[a_id]}")

        debug_starts = set()
        debug_goals = set()
        for task in tasks.values():
            debug_starts.add(task[0])
            debug_goals.add(task[1])

        if len(debug_starts) != len(tasks):
            raise ValueError("Duplicate starts")
        if len(debug_goals) != len(tasks):
            raise ValueError("Duplicate goals")

        try:
            success, solution_node = self.CBS.Solve(tasks, list(cc), static_agents, [root_node])
        except:
            print("CBS Timed out, unable to solve.")
            return False

        if not success:
            return False

        if self.debug:
            print(f"\tSolution for: {cc}")
            for a_id, path in solution_node.paths.items():
                print(f"\t\tPath: {path}")

        for agent_id in cc:
            agent = self.agents[agent_id]
            new_path = solution_node.paths[agent_id]
            if self.completed_agents[agent_id]:
                if len(new_path) > 1:
                    self.completed_agents[agent_id] = False
                    if len(new_path) == 0:
                        raise ValueError("New path has 0 length")
            agent.path = deepcopy(new_path)
            agent.constraints = deepcopy(solution_node.constraints[agent_id])
            agent.tasks[agent.current_task] = tasks[agent_id]
            agent.start_time = self.current_time

        return success


    # Method: Sets whether the path tracker should be activated.
    def SetPathTracker(self, activate):
        self.track_paths = activate

    # Method: Sets the maximum number of allocation rounds.
    def SetAllocationMax(self, count):
        self.allocation_max = max(count, 1)

    # Method: Allocates tasks to untasked agents.
    def AllocateTasks(self, untasked_agents=[]):
        self.allocation_count = (self.allocation_count + 1) % self.allocation_max
        if sum(self.completed_agents.values()) == len(self.agents):
            self.allocation_count = 0

        if self.allocation_count != 0:
            return True

        if self.debug:
            print("Allocating tasks...")

        start_time = time.time()
        self.task_allocator.UpdateSystem(self.current_time, self.agents, untasked_agents)
        solved = self.task_allocator.Solve()
        end_time = time.time()
        self.task_allocation_time += end_time - start_time

        if not solved:
            return False

        if self.debug:
            for agent_id, tasks in self.task_allocator.allocation.items():
                print(f"\tAgent {agent_id}: {tasks}")

        return True

    # Method: Calculates the flush time for exchanging info packets between two agents.
    def CalculateFlushTime(self, agent_i: PRISMAgent, agent_j: PRISMAgent):
        flush = 0
        # calculate flush time from last aplied constraint
        for full_constraint in agent_i.constraints:
            agent_num, constraint = full_constraint
            if agent_num == agent_j.id:
                _, times = constraint
                if isinstance(times, tuple):
                    flush = max(flush, times[1])
                else:
                    flush = max(flush, times)

        for full_constraint in agent_j.constraints:
            agent_num, constraint = full_constraint
            if agent_num == agent_i.id:
                _, times = constraint
                if isinstance(times, tuple):
                    flush = max(flush, times[1])
                else:
                    flush = max(flush, times)

        return flush

    # Method: Prints the key-value pairs of a dictionary in a formatted manner.
    def PrintDictInfo(self, parse_dict, key_label, value_label):
        output_string = ""
        for k, v in parse_dict.items():
            output_string += value_label + " for "
            output_string += key_label + " " + str(k) + ": "
            output_string += str(v) + "\n"
        print(output_string)

    # Method: Returns the path tracker, task tracker, and completed tracker.
    def GetTrackers(self):
        if self.path_tracker is False:
            print("You did not activate the path tracker")
            return None, None

        return self.path_tracker, self.task_tracker, self.completed_tracker