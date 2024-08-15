from copy import deepcopy

class EmptyAllocator():
    # Constructor: Initializes the EmptyAllocator with an optional debug mode.
    def __init__(self, debug=False):
        self.debug = debug        # Debug mode flag
        self.num_completed = 0    # Number of completed tasks
        self.allocation = dict()  # Dictionary to store task allocations

    # Method: Initializes the allocator with robot IDs and tasks.
    def InitializeAllocator(self, robot_ids, tasks):
        self.tasks = deepcopy(tasks)  # Deep copy of the tasks (tuple of (start, goal))
        self.task_dict = {task[1]:idx for idx, task in enumerate(tasks)}  # Map each goal to its task index
        self.completed_tasks = [False for _ in self.tasks]  # Track completion status of tasks
        self.num_tasks = len(tasks)  # Total number of tasks

    # Method: Marks a task as completed if it exists in the task dictionary.
    def TaskCompleted(self, task):
        if task not in self.task_dict:
            return
        idx = self.task_dict[task]
        if not self.completed_tasks[idx]:
            self.num_completed += 1  # Increment the count of completed tasks
        self.completed_tasks[idx] = True  # Mark the task as completed

    # Method: Updates the system's state at each time step (currently a placeholder).
    def UpdateSystem(self, current_time, agents, untasked_agents):
        return

    # Method: Returns True, indicating that the solving process is complete.
    def Solve(self):
        return True

    # Method: Returns the task assigned to a given agent (currently returns None).
    def GetTask(self, agent_id):
        return None

    # Method: Removes and returns the task assigned to a given agent (currently a placeholder).
    def PopTask(self, agent_id):
        return

    # Method: Returns all allocated tasks as a set (currently returns an empty set).
    def GetAllAllocationTasks(self):
        return set()

    # Method: Finds the original task corresponding to a given goal.
    def FindTrueTask(self, goal):
        idx = self.task_dict[goal]  # Get the index of the task associated with the goal
        return self.tasks[idx]  # Return the original task (start, goal)
