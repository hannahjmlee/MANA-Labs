import os
import sys
import csv

from src.utils.parse import *
from src.base.problem import *

def BuildProblem(map_name, scenario=None): 
    map_file = map_name + ".map"
    map_path = os.path.join(os.getcwd(), "benchmarks", "maps")
    map_path = os.path.join(map_path, map_file)

    map_lines = GetLines(map_path, 4)
    bool_grid = CreateBooleanGrid(map_lines)


    problem = Problem(bool_grid, map_name)
    
    if scenario != None: 
        scen_lines = ExtractScenario(map_name, scenario)
        problem.LoadScenario(scen_lines)
        
    return problem


def ExtractScenario(map_name, scenario): 
    scen_file = map_name + "-random-" + str(scenario) + ".scen"
    scen_path = os.path.join(os.getcwd(), "benchmarks", "scenarios")
    scen_path = os.path.join(scen_path, scen_file)

    scen_lines = GetLines(scen_path, 1)
    return scen_lines
