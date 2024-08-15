from src.utils.parse import *
from src.base.problem import *
from src.solvers.MAPF import *
from src.solvers.DEC_MAPF import *
from src.utils.display import *

from tests.builder import *

# Function: Runs a centralized single-task MAPF problem on a specified map with given parameters.
def RunCentralizedSingleTask(map_name, scenario, num_agents, algorithm, low_level, debug, animate):
    print("Solving Centralized Problem on:", map_name)

    problem = BuildProblem(map_name, scenario)
    solver = MAPF(problem, algorithm, low_level, debug, animate)
    solved, cost, solution, time = solver.Solve(num_agents)

    print("cost:", cost)
    print("time:", time)
    print()

    return solved, cost, solution, time
