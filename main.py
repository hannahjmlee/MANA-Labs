from tests.dec_mapf_tester import *
from tests.mapf_tester import *
from tests.prism_tester import *
from tests.utils_tester import *

def main():
    map_groupings = {"Empty": ["empty-8-8", "empty-16-16", "empty-32-32", "empty-48-48"],
                 "Random": ["random-32-32-10", "random-32-32-20", "random-64-64-10", "random-64-64-20"],
                 "Rooms":  ["room-32-32-4", "room-64-64-8", "room-64-64-16"],
                 "Maze": ["maze-32-32-2", "maze-32-32-4", "maze-128-128-2", "maze-128-128-10"],
                 "Cities": ["Berlin_1_256", "Boston_0_256", "Paris_1_256"],
                 "Games-Small": ["ht_chantry", "ht_mansion_n", "lak303d", "lt_gallowstemplar_n", "den312d", "ost003d"],
                 "Games-Large": ["brc202d", "den520d", "w_woundedcoast"]}

    map_name = "empty-32-32"
    scenario = 6
    num_tasks = 25
    num_agents = 25

    mapf_algorithm = "cbs"
    low_level = "AStar"
    cost_metric = 'soc'

    dec_mapf_algorithm = "prism"

    allocator = "taskILP"
    # allocator = "empty"

    # detection_method = 'raycasting'
    detection_method='proximity'
    limit = 0.05

    # debug = True
    debug = False

    # animate = True
    animate = False

    # MAPF Testing Examples ------------------------------------------------------------

    RunDecentralizedSingleTask(map_name, scenario, num_tasks, num_agents, dec_mapf_algorithm,
                            allocator, mapf_algorithm, low_level, cost_metric, detection_method,
                            limit, debug, animate)
    
    RunCentralizedSingleTask(map_name, scenario, num_agents, mapf_algorithm, low_level, debug, animate)


    # Utils Testing Examples ------------------------------------------------------------

    #scenario_range = [1, 2, 3, 4, 6, 7, 8, 10, 11, 12, 13, 15, 18, 19, 20]
    #display_nums = False
    #map_name = "random-64-64-20"
    #RunLineOfSightTests(map_name, 0.1, map_name + "_los", scenario_range, display_nums)
    #RunProximityTests(map_name, 0.1, map_name + "_prox", scenario_range, display_nums)

    # map_names = ["empty-48-48", "random-64-64-20", "room-64-64-16", "room-64-64-8", "den312d"]
    # for map_name in map_names:
    #     VisualizeEnvironment(map_name)

if __name__=="__main__":
    main()