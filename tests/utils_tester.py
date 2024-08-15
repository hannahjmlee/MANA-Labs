from src.detection.raycasting import *
from src.utils.utils_display import *
from tests.builder import *

# Function: Runs line-of-sight tests on a specified map and scenario range, visualizing the results.
def RunLineOfSightTests(map_name, limit_multi, output_name, scenario_range = [1], display_range = False):
    problem = BuildProblem(map_name, scenario_range[0])
    limit = round(max(limit_multi * min(problem.num_rows, problem.num_cols), 2.5))
    
    positions = []
    los_grids = []
    for scenario in scenario_range: 
        problem.LoadScenario(ExtractScenario(map_name, scenario))
        start_position = problem.queries[2][0]

        raycaster = RayCasting(problem.grid)
        start_los = raycaster.CalculatePointLineOfSight(start_position, limit)

        positions.append(start_position)
        los_grids.append(start_los)

    if not display_range: 
        scenario_range = []
    DisplayLineOfSight(positions, los_grids, problem.grid, "tests/figures/utils_tests/"+output_name, scenario_range)

# Function: Runs proximity tests on a specified map and scenario range, visualizing the results.
def RunProximityTests(map_name, limit_multi, output_name, scenario_range = [1], display_range = False):
    problem = BuildProblem(map_name, scenario_range[0])
    limit = round(max(limit_multi * min(problem.num_rows, problem.num_cols), 2.5))

    positions = []
    for scenario in scenario_range: 
        problem.LoadScenario(ExtractScenario(map_name, scenario))
        start_position = problem.queries[2][0]
        positions.append(start_position)

    if not display_range: 
        scenario_range = []
    DisplayProximity(positions, problem.grid, limit, "tests/figures/utils_tests/"+output_name, scenario_range)

# Function: Visualizes the environment of a specified map.
def VisualizeEnvironment(map_name): 
    problem = BuildProblem(map_name, 1)

    output_name = map_name + "_env.png"
    DisplayEnvironment(problem.grid, map_name, "tests/figures/envs/" + output_name)
