import glob
import time
from pathlib import Path
from cbs import CBSSolver
from independent import IndependentSolver
from prioritized import PrioritizedPlanningSolver
from distributed import DistributedPlanningSolver # Placeholder for Distributed Planning
from visualize import Animation
from single_agent_planner import get_sum_of_cost
import numpy as np


def import_mapf_instance(filename, agent_number):
    """
    Imports mapf instance from instances folder. Expects input as a .txt file in the following format:
        Line1: #rows #columns (number of rows and columns)
        Line2-X: Grid of @ and . symbols with format #rows * #columns. The @ indicates an obstacle, whereas . indicates free cell.
        Line X: #agents (number of agents)
        Line X+1: xCoordStart yCoordStart xCoordGoal yCoordGoal (xy coordinate start and goal for Agent 1)
        Line X+2: xCoordStart yCoordStart xCoordGoal yCoordGoal (xy coordinate start and goal for Agent 2)
        Line X+n: xCoordStart yCoordStart xCoordGoal yCoordGoal (xy coordinate start and goal for Agent n)
        
    Example:
        4 7             # grid with 4 rows and 7 columns
        @ @ @ @ @ @ @   # example row with obstacle in every column
        @ . . . . . @   # example row with 5 free cells in the middle
        @ @ @ . @ @ @
        @ @ @ @ @ @ @
        2               # 2 agents in this experiment
        1 1 1 5         # agent 1 starts at (1,1) and has (1,5) as goal
        1 2 1 4         # agent 2 starts at (1,2) and has (1,4) as goal
    """
    f = Path(filename)
    if not f.is_file():
        raise BaseException(filename + " does not exist.")
    f = open(filename, 'r')
    # first line: #rows #columns
    line = f.readline()
    rows, columns = [int(x) for x in line.split(' ')]
    rows = int(rows)
    columns = int(columns)
    # rows lines with the map
    my_map = []
    for r in range(rows):
        line = f.readline()
        my_map.append([])
        for cell in line:
            if cell == '@':
                my_map[-1].append(True)
            elif cell == '.':
                my_map[-1].append(False)
    # override number of agents with that provided as input
    line = f.readline()
    num_agents = agent_number
    # agents lines with the start/goal positions
    starts = []
    goals = []
    for a in range(num_agents):
        line = f.readline()
        sx, sy, gx, gy = [int(x) for x in line.split(' ')]
        starts.append((sx, sy))
        goals.append((gx, gy))
    f.close()
    return my_map, starts, goals


def run(instance, solver, agent_number):

    my_map, starts, goals = import_mapf_instance(instance, agent_number)

    if solver == "CBS":
        solver = CBSSolver(my_map, starts, goals)
    elif solver == "Independent":
        solver = IndependentSolver(my_map, starts, goals)
    elif solver == "Prioritized":
        solver = PrioritizedPlanningSolver(my_map, starts, goals)
    elif solver == "Distributed":  # Wrapper of distributed planning solver class
        solver = DistributedPlanningSolver(my_map, starts, goals) 

    t0 = time.time()
    paths = solver.find_solution()
    Dt = time.time() - t0

    cost = get_sum_of_cost(paths)

    return [cost, Dt]

def analysis_loop(instances, solvers, agent_numbers):
    
    for instance in instances:
        for solver in solvers:
            for agent_number in agent_numbers:
                #Outputs
                costs = []
                dts = []
                # Coefficient of variation only for cost
                c_v = []
                ready_to_check = 0

                while True:
                    # Do run and append to outputs
                    cost,dt = run(instance, solver, agent_number)
                    costs.append(cost)
                    dts.append(dt)
                    # Caclulate CVs
                    cv_c = np.std(np.array(costs))/np.mean(np.array(costs))
                    c_v.append(cv_c)
                    # Check if you can stop
                    ready_to_check +=1 # Generate at least a few data points to start from

                    if ready_to_check >= 20:
                        # If ratio of standard dev of all vs the last 10 is smaller than a threshold
                        std_c = np.std(np.array(c_v[-10:]))
                        std_all = np.std(np.array(c_v))
                        ratio = std_c/std_all

                        if ratio < 0.2:
                            break 
                # Record relevant data about this instance
                












        

    

print(run("test_maps/map1_1.txt", "Distributed", 3))