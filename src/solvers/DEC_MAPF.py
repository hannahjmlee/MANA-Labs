from src.base.problem import *

from src.planning.path.AStar import *
from src.planning.mapf.dec_cbs import *
from src.planning.mapf.dec_icbs import *
from src.planning.task.empty_allocator import *
from src.planning.task.prism import *
from src.planning.task.taskILP import *

from src.utils.display import *
import src.utils.decentralized_display as dec_display

from src.detection.proximity import *
from src.detection.raycasting import *

import time
import timeout_decorator

class DEC_MAPF:
    # Constructor: Initializes the DEC_MAPF instance with various planning and detection strategies.
    def __init__(self, problem:Problem, dec_mapf="prism", allocator="taskILP", mapf_algorithm='cbs', low_level="AStar", cost_metric='soc', detection_method='proximity', limit=10, debug=True, animate=True):
        self.problem = problem  # The problem instance to be solved
        self.cost_metric = cost_metric  # Cost metric ('soc' or 'makespan')

        self.tasks = []  # List of tasks

        self.animate = animate  # Flag to indicate whether to animate the solution
        self.debug = debug  # Debug mode flag

        # Initialize path planner based on the chosen low-level planner
        if low_level == "AStar":
            self.path_planner = AStarSearch(self.problem.grid, self.debug)
        else:
            self.path_planner = AStarSearch(self.problem.grid, self.debug)

        # Initialize multi-agent planner based on the chosen MAPF algorithm
        if mapf_algorithm == 'cbs':
            self.multi_agent_planner = DecentralizedCBS(self.path_planner.PlanPath, self.debug)
        elif mapf_algorithm == 'icbs':
            self.multi_agent_planner = DecentralizedICBS(self.path_planner.PlanPath, self.debug)
        else:
            self.multi_agent_planner = DecentralizedCBS(self.path_planner.PlanPath, self.debug)

        # Initialize detection method based on the chosen detection strategy
        if detection_method == 'proximity':
            proximity = Proximity(limit)
            self.detection = proximity.InRange
        elif detection_method == 'raycasting':
            raycaster = RayCasting(self.problem.grid, limit)
            self.detection = raycaster.InRange
        else:
            self.detection = None

        # Initialize task allocator based on the chosen allocator
        if allocator == "taskILP":
            self.allocator = TaskAssignmentILP(self.path_planner.PlanPath, self.debug)
        elif allocator == "empty":
            self.allocator = EmptyAllocator(self.debug)
        else:
            self.allocator = None

        # Initialize the decentralized MAPF algorithm based on the chosen approach
        if dec_mapf == "prism":
            self.algorithm = PRISM(self.path_planner.PlanPath, self.detection, self.allocator, self.debug)
            self.algorithm.SetPathTracker(self.animate)
            if allocator == "taskILP":
                self.algorithm.SetAllocationMax(15)
        else:
            self.algorithm = None

    # Method: Solves the DEC-MAPF problem with the given number of tasks and agents.
    def Solve(self, num_tasks, num_agents):
        self.tasks = self.problem.queries[:num_tasks]

        if self.debug:
            for i, task in enumerate(self.tasks):
                print("Task", i, ":", task)
            print()

        try:
            start_time = time.time()
            success, solution, task_allocation_time = self.algorithm.Solve(self.tasks, num_agents)
            end_time = time.time()

            if success == False:
                return False, None, None, None, None

            for _, path in solution.items():
                while True:
                    if path[-1] == path[-2]:
                        path.pop(-1)
                    else:
                        break
        except timeout_decorator.TimeoutError:
            print("Function Timed out after 30 seconds")
            return False, None, None, None, None

        # Calculate solution cost based on the chosen cost metric
        solution_cost = 0
        if self.cost_metric == 'soc':
            solution_cost = sum([len(x) for x in solution.values()])
        elif self.cost_metric == 'makespan':
            solution_cost = max([len(x) for x in solution.values()])

        # Animate the solution if the animate flag is set
        if self.animate:
            print("Solved")
            frame_rate, intermediates = GetAnimationParams(self.problem.num_rows)
            bounds, obstacles = PreprocessMap(self.problem.grid)

            if self.algorithm.name == "prism":
                path_tracker, task_tracker, completed_tracker = self.algorithm.GetTrackers()
                t, states, goals, completed = dec_display.PreprocessAnimation(path_tracker, task_tracker, completed_tracker, intermediates)

                save_file = "tests/videos/decentralized/" + self.problem.map_name + "_t" + str(num_tasks) + "_a" + str(num_agents) + ".mp4"
                dec_display.Animate(t, states, goals, bounds, obstacles, self.tasks, completed, frame_rate, save_file)


        # Return the solution details based on the algorithm used
        if self.algorithm.name == "prism":
            return success, solution_cost, solution, end_time - start_time - task_allocation_time, task_allocation_time
