from src.planning.path.AStar import *
from src.planning.mapf.cbs import *
from src.planning.mapf.icbs import *
from src.base.problem import *

from src.utils.display import *
from src.utils.centralized_display import *
import time
import timeout_decorator

class MAPF:
    # Constructor: Initializes the MAPF instance with the problem, algorithm, and settings for debugging and animation.
    def __init__(self, problem, algorithm, low_level="AStar", debug=True, animate=True):
        self.problem = problem  # The problem instance to be solved
        self.debug = debug  # Debug mode flag
        self.animate = animate  # Flag to indicate whether to animate the solution

        self.tasks = []  # List of tasks

        # Initialize the low-level path planner based on the specified algorithm
        if low_level == "AStar": 
            self.path_planner = AStarSearch(self.problem.grid, self.debug)

        # Initialize the MAPF algorithm (CBS or ICBS)
        if algorithm == "cbs":
            self.algorithm = CBS(self.path_planner, self.debug)
        elif algorithm == "icbs":
            self.algorithm = ICBS(self.path_planner.PlanPath, self.debug)

    # Method: Solves the MAPF problem for the specified number of agents.
    def Solve(self, num_agents):
        self.tasks = self.problem.queries[:num_agents]

        if self.debug:
            for i, task in enumerate(self.tasks):
                print("Task", i, ":", task)
            print()

        try:
            start_time = time.time()
            success, solution_cost, solution = self.algorithm.Solve(self.tasks, [])
            end_time = time.time()
        except timeout_decorator.TimeoutError:
            print("Function Timed out after 60 seconds")
            return False, None, None, None
    
        if not success: 
            return False, None, None, None
        
        # Calculate solution cost based on the sum-of-costs
        solution_cost = sum([len(x) for x in solution.values()])
        
        # Animate the solution if the animate flag is set
        if self.animate:        
            print("Solved")
            frame_rate, intermediates = GetAnimationParams(self.problem.num_rows)
            bounds, obstacles = PreprocessMap(self.problem.grid)
            t, states = PreprocessAnimation(solution, intermediates)
            save_file = "tests/videos/centralized/" + self.problem.map_name + "_a" + str(num_agents) + ".mp4"

            Animate(t, states, self.tasks, bounds, obstacles, frame_rate, save_file)
        return success, solution_cost, solution, end_time - start_time

    # Method: Creates a map of tasks where each agent's path is associated with its goal.
    def CreateSingleTaskMap(self, num_agents, tasks, solution):
        task_map = dict()
        for agent, path in solution.items():
            _, goal = tasks[agent]
            task_map[agent] = [goal for _ in range(len(path))]
        return task_map
