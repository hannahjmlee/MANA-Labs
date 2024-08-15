from src.utils.parse import *
from src.base.problem import *
from src.solvers.MAPF import *
from src.solvers.DEC_MAPF import *
from src.utils.display import *
from tests.builder import *

from src.planning.path.AStar import *
from src.planning.mapf.dec_cbs import *
from src.planning.mapf.dec_icbs import *
from src.planning.mapf.utils.ct import *

# Function: Solves a decentralized single-task MAPF problem with given parameters on a specified map.
def RunDecentralizedSingleTask(map_name, scenario, num_tasks, num_agents, dec_mapf, allocator, mapf, low_level, cost, detection, limit_multi, debug=False, animate=False):
    print("Solving Decentralized Problem on:", map_name)

    problem = BuildProblem(map_name, scenario)
    if limit_multi < 1:
        env_size = min(problem.num_rows, problem.num_cols)
        limit = limit_multi * env_size
    else:
        limit = limit_multi

    limit = max(limit, 2.5)

    solver = DEC_MAPF(problem, dec_mapf, allocator, mapf, low_level, cost, detection, limit, debug, animate)
    solved, cost, solution, time, task_time = solver.Solve(num_tasks, num_agents)

    print("cost:", cost)
    print("time:", time)
    print()

    return solved, cost, solution, time, task_time

# Function: Tests decentralized MAPF solver on a single-task with given parameters, iterating up to 5 times to find a solution.
def TestDecentralizedSingleTask(map_name, scenario, num_tasks, num_agents, dec_mapf, allocator, mapf, low_level, cost_metric, detection):
    limit_multiplier = 0.05

    problem = BuildProblem(map_name)
    problem.LoadScenario(ExtractScenario(map_name, scenario))

    env_size = min(problem.num_rows, problem.num_cols)
    limit = max(limit_multiplier * env_size, 2.5)

    solved = False
    for _ in range(5):
        try:
            solver = DEC_MAPF(problem, dec_mapf, allocator, mapf, low_level, cost_metric, detection, limit, False, False)
            solved, cost, solution, time, task_time = solver.Solve(num_tasks, num_agents)

            if solved:
                makespan = max([len(x) for x in solution.values()])
                print(f"{map_name},{scenario},{detection},{num_agents},{num_tasks},{cost},{makespan},{time},{task_time}")
                break
        except:
            continue

    if not solved:
        print(f"{map_name},{scenario},{detection},{num_agents},{num_tasks},NaN,NaN,NaN,NaN")

    return solved
