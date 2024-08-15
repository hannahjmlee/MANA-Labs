class PRISMAgent: 
    # Constructor: Initializes the PRISMAgent with an ID, an optional task, and an optional path.
    def __init__(self, id, task=None, path=None):
        self.id = id               # Unique identifier for the agent
        self.start_time = 0        # Start time of the current task

        self.current_task = 0      # Index of the current task
        self.tasks = [task]        # List of tasks assigned to the agent
        self.goal = task[1]        # Goal position for the current task
        self.path = path           # Path the agent will follow
        self.previous_goal = None  # Previous goal position

        self.info_packets = []     # List of info packets associated with the agent
        self.constraints = set()   # Set of constraints applied to the agent
        self.at_rest = False       # Flag indicating if the agent is at rest

    # Method: Sets a new task for the agent, clearing previous tasks and constraints.
    def SetTask(self, task, time, at_rest=False): 
        self.tasks.clear()  
        self.constraints.clear() 
        self.current_task = 0 

        current_position = self.GetCurrentPosition(time)  # Get current position based on time
        if current_position != task[0]: 
            self.tasks.append((current_position, task[0]))  # Add transition to the new task start
        self.tasks.append(task)  # Add the new task
        self.start_time = time  # Set start time
        self.goal = self.tasks[self.current_task][1]  # Update goal position

        self.at_rest = at_rest  # Update rest status

    # Method: Returns the duration of the current path.
    def GetPathDuration(self, time):         
        subpath = self.GetCurrentPath(time)  # Get the subpath based on current time
        return max(len(subpath) - 1, 1)  # Return the duration of the path

    # Method: Returns the current position of the agent based on time.
    def GetCurrentPosition(self, time):
        return self.path[self.GetCurrentPathIndex(time)]
    
    # Method: Returns the current path of the agent based on time.
    def GetCurrentPath(self, time): 
        return self.path[self.GetCurrentPathIndex(time):]
    
    # Method: Returns the index of the current path position based on time.
    def GetCurrentPathIndex(self, time):
        idx = time - self.start_time
        if idx >= len(self.path) and self.at_rest: 
            return -1
        return time - self.start_time
    
    # Method: Returns the final task assigned to the agent.
    def GetTrueTask(self): 
        return self.tasks[-1]
    
    # Method: Checks if the agent is currently on its final task.
    def OnTrueTask(self): 
        if len(self.tasks) > 1: 
            if self.current_task == 0: 
                return False
            return True
        return True    

    # Method: Checks if the agent has reached the last position on its path.
    def OnLastPosition(self, time): 
        if self.at_rest: 
            if len(self.tasks) == 1 and len(self.path) == 1: 
                return True
        
        index = self.GetCurrentPathIndex(time)

        if index == len(self.path) - 1:
            return True
        return False        

    # Method: Returns the constraints applicable to the agent at a given time.
    def GetConstraints(self, time): 
        offset = time - self.start_time 
        if offset == 0: 
            return self.constraints
        
        constraints = set() 
        for full_constraint in self.constraints: 
            giver, constraint = full_constraint
            position, time = constraint
            
            new_time = None
            if type(time) is tuple: 
                t0 = time[0] - offset
                t1 = time[1] - offset 

                if t0 < 0 or t1 < 0: 
                    continue

                new_time = (t0, t1)
            else: 
                new_time = time - offset
                
            constraints.add((giver, (position, new_time)))

        return constraints

class InfoPacket: 
    # Constructor: Initializes an InfoPacket with data from a PRISMAgent and timing details.
    def __init__(self, agent: PRISMAgent, received_time, flush_time, end_time): 
        self.id = agent.id                                                 # ID of the agent that generated the packet
        self.constraints = agent.GetConstraints(received_time)             # Constraints at the time of receipt
        self.task = (agent.GetCurrentPosition(received_time), agent.goal)  # Task at the time of receipt

        self.received_time = received_time                                 # Time when the packet was received
        self.flush_time = flush_time                                       # Time when the packet should be flushed
        self.end_time = end_time                                           # End time for the packet's validity

    # Method: Returns the index of the current path position based on time.
    def GetCurrentPathIndex(self, time): 
        return time - self.received_time
    
    # Method: Returns the constraints applicable at a given time, adjusted for the offset from received time.
    def GetConstraints(self, time): 
        offset = time - self.received_time
        if offset == 0: 
            return self.constraints
        
        constraints = set()
        for full_constraint in self.constraints: 
            giver, constraint = full_constraint
            position, time = constraint
            new_time = None
            if type(time) is tuple: 
                t0 = time[0] - offset
                t1 = time[1] - offset 

                if t0 < 0 or t1 < 0: 
                    continue

                new_time = (t0, t1)
            else: 
                new_time = time - offset
                
            constraints.add((giver, (position, new_time)))
        return constraints
