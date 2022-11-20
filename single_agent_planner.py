import heapq
import copy
import random

def move(loc, dir):
    directions = [(0, 0), (0, -1), (1, 0), (0, 1), (-1, 0)]
    return loc[0] + directions[dir][0], loc[1] + directions[dir][1]



def get_sum_of_cost(paths):
    rst = 0
    for path in paths:
        rst += len(path) - 1
    return rst



def compute_heuristics(my_map, goal):
    # Use Dijkstra to build a shortest-path tree rooted at the goal location
    open_list = []
    closed_list = dict()
    root = {'loc': goal, 'cost': 0}
    heapq.heappush(open_list, (root['cost'], goal, root))
    closed_list[goal] = root
    while len(open_list) > 0:
        (cost, loc, curr) = heapq.heappop(open_list)
        for dir in range(5):
            child_loc = move(loc, dir)
            child_cost = cost + 1
            if child_loc[0] < 0 or child_loc[0] >= len(my_map) \
               or child_loc[1] < 0 or child_loc[1] >= len(my_map[0]):
               continue
            if my_map[child_loc[0]][child_loc[1]]:
                continue
            child = {'loc': child_loc, 'cost': child_cost}
            if child_loc in closed_list:
                existing_node = closed_list[child_loc]
                if existing_node['cost'] > child_cost:
                    closed_list[child_loc] = child
                    # open_list.delete((existing_node['cost'], existing_node['loc'], existing_node))
                    heapq.heappush(open_list, (child_cost, child_loc, child))
            else:
                closed_list[child_loc] = child
                heapq.heappush(open_list, (child_cost, child_loc, child))

    # build the heuristics table
    h_values = dict()
    for loc, node in closed_list.items():
        h_values[loc] = node['cost']
    return h_values



def build_constraint_table(constraints, agent):
    ##############################
    # Task 1.2/1.3: Return a table that constains the list of constraints of
    #               the given agent for each time step. The table can be used
    #               for a more efficient constraint violation check in the 
    #               is_constrained function.

    table = []
    times = []
    goals_avoid = []
    
    for constraint in constraints:
        if constraint['agent'] == agent and type(constraint['timestep']) == int:
            # Regular Constraints
            table.append((constraint['loc'], constraint['timestep']))
            times.append(constraint['timestep'])
        # Goal Constraints
        if constraint['agent'] == agent and type(constraint['timestep']) != int:
            goals_avoid.append((constraint['loc'], constraint['timestep'][0]))


            
    if len(constraints) == 0:
        table.append(0)
        times.append(0)
    
    
    return table, max(times), goals_avoid



def get_location(path, time):
    if time < 0:
        return path[0]
    elif time < len(path):
        return path[time]
    else:
        return path[-1]  # wait at the goal location



def get_path(goal_node):
    path = []
    curr = goal_node
    while curr is not None:
        path.append(curr['loc'])
        curr = curr['parent']
    path.reverse()
    return path


def is_constrained(curr_loc, next_loc, next_time, constraint_table):
    ##############################
    # Task 1.2/1.3: Check if a move from curr_loc to next_loc at time step next_time violates
    #               any given constraint. For efficiency the constraints are indexed in a constraint_table
    #               by time step, see build_constraint_table.

    index_vertex = ([next_loc],next_time)
    index_edge = ([curr_loc, next_loc], next_time)

    if index_vertex in constraint_table:
        vertex = True
    else:
        vertex = False

    if index_edge in constraint_table:
        edge = True
    else:
        edge = False

    return vertex + edge

    


def push_node(open_list, node):
    heapq.heappush(open_list, (node['g_val'] + node['h_val'], node['h_val'], node['loc'],random.random(),node))


def pop_node(open_list):
    _, _, _, _,curr = heapq.heappop(open_list)
    return curr


def compare_nodes(n1, n2):
    """Return true is n1 is better than n2."""
    return n1['g_val'] + n1['h_val'] < n2['g_val'] + n2['h_val']



def illegal(location,timestep,dir,my_map,constraint_table):

    # CHECK MOVE IS LEGAL
    ###################################################
    # Checks place u moving to is in the map
    child_loc = move(location, dir)
    # Within Bounds
    x_good =  0 <= child_loc[0] <= len(my_map) -1
    y_good =  0 <= child_loc[1] <= len(my_map[0]) -1

    if x_good + y_good < 2:
        return True

    # Not Obstacle
    if my_map[child_loc[0]][child_loc[1]]:
        return True

    # Check any constraint violations
    if is_constrained(location, child_loc, timestep, constraint_table):
        return True
    # Move is legal
    return False



def a_star(my_map, start_loc, goal_loc, h_values, agent, constraints, inner=False, goals_avoid = None, latest_time= None):
    """ my_map      - binary obstacle map
        start_loc   - start position
        goal_loc    - goal position
        agent       - the agent that is being re-planned
        constraints - constraints defining where robot should or cannot go at each timestep
    """
    ##############################
    # Task 1.1: Extend the A* search to search in the space-time domain
    #           rather than space domain, only.
    
    # Load Constraints
    if inner:
        constraint_table = constraints
        latest_time = latest_time
        goals_avoid = goals_avoid
    else:
        constraint_table, latest_time, goals_avoid = build_constraint_table(constraints, agent)
    # Create Open and Closed List and calculate heuristics
    open_list = []
    closed_list = dict()
    earliest_goal_timestep = 0
    h_value = h_values[start_loc]
    root = {'loc': start_loc, 'g_val': 0, 'h_val': h_value, 'timestep':earliest_goal_timestep, 'parent': None}
    push_node(open_list, root)
    closed_list[(root['loc'],root['timestep'])] = root
   
    while len(open_list) > 0:

        curr = pop_node(open_list)
        #############################
        # Task 1.4: Adjust the goal test condition to handle goal constraints
        if curr['loc'] == goal_loc:# and curr['timestep'] >= latest_time:
                path = get_path(curr)

                if curr['timestep'] >= latest_time:
                    return path

                else:
                    
                    timestep = len(path)
                    location = path[-1]
                    
                    while not is_constrained(location,location, timestep,constraint_table):
                        path.append(location)
                        if len(path) >= latest_time:
                            return path
                        timestep +=1
                    
                    for dir in range(5):
                        if illegal(location, timestep, dir, my_map, constraint_table):
                            continue
                        location = move(location, dir)
                        break
                    # Path to go back to goal
                    constraint_table_mod = [(x[0],x[1]-timestep) for x in constraint_table]
                    goals_avoid_mod = [(x[0],x[1]-timestep) for x in goals_avoid]
                    latest_time_mod = latest_time - timestep
                    sub_path = a_star(my_map, location,goal_loc,h_values,agent,constraint_table_mod,inner=True,goals_avoid=goals_avoid_mod,latest_time=latest_time_mod)
                    return path + sub_path
                    
                                

        # Generate Goal Constraints and add them to the table on the spot

        for const in goals_avoid:
            if const[1] <= curr['timestep']:
                constraint_table.append((const[0], curr['timestep']+1))

        
        for dir in range(5):
            if illegal(curr['loc'],curr['timestep']+1, dir, my_map, constraint_table):
                continue
            
            child_loc = move(curr['loc'],dir)

            # Create child node info
            child = {'loc': child_loc,
                    'g_val': curr['g_val'] + 1,
                    'h_val': h_values[child_loc],
                    'timestep':curr['timestep']+1,
                    'parent': curr}

            # Feels like the heuristic stops working at some point
            if (child['loc']) in closed_list:
                existing_node = closed_list[(child['loc'], child['timestep'])]
                if compare_nodes(child, existing_node):
                    closed_list[(child['loc'], child['timestep'])] = child
                    push_node(open_list, child)


            else:
                closed_list[(child['loc'], child['timestep'])] = child
                push_node(open_list, child)

            
    

    return   # Failed to find solutions
