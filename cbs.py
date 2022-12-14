import time as timer
import heapq
import random
from single_agent_planner import compute_heuristics, a_star, get_location, get_sum_of_cost
import itertools



def detect_collision(path1, path2):
    ##############################
    # Task 3.1: Return the first collision that occurs between two robot paths (or None if there is no collision)
    #           There are two types of collisions: vertex collision and edge collision.
    #           A vertex collision occurs if both robots occupy the same location at the same timestep
    #           An edge collision occurs if the robots swap their location at the same timestep.
    #           You should use "get_location(path, t)" to get the location of a robot at time t.

    # How many timesteps to inspect
    length = max([len(path1), len(path2)])
    
    for timestep in range(length):
        # Check vertex collision
        if get_location(path1, timestep) == get_location(path2, timestep):
            return ([get_location(path1, timestep)], timestep)

        # Check Edge collision
        if timestep != 0:
    
            p1t0 = get_location(path1, timestep-1)
            p1t1 = get_location(path1, timestep)
            p2t0 = get_location(path2, timestep-1)
            p2t1 = get_location(path2, timestep)

            if p1t0 == p2t1 and p1t1 == p2t0:
                return ([p1t0, p1t1], timestep)
                
    return None        


def detect_collisions(paths):
    ##############################
    # Task 3.1: Return a list of first collisions between all robot pairs.
    #           A collision can be represented as dictionary that contains the id of the two robots, the vertex or edge
    #           causing the collision, and the timestep at which the collision occurred.
    #           You should use your detect_collision function to find a collision between two robots.

    collisions = []

    for path1, path2 in itertools.combinations(paths, 2):
        collision = detect_collision(path1, path2)
        if collision != None:
            collisions.append({'IDs':[paths.index(path1),paths.index(path2)], 'loc':collision[0], 'timestep':collision[1]})

    return collisions

def standard_splitting(collision):
    ##############################
    # Task 3.2: Return a list of (two) constraints to resolve the given collision
    #           Vertex collision: the first constraint prevents the first agent to be at the specified location at the
    #                            specified timestep, and the second constraint prevents the second agent to be at the
    #                            specified location at the specified timestep.
    #           Edge collision: the first constraint prevents the first agent to traverse the specified edge at the
    #                          specified timestep, and the second constraint prevents the second agent to traverse the
    #                          specified edge at the specified timestep


    # Figure out wether this is a vertex or an edge constraint
    constraints = [{'agent':collision['IDs'][0], 'loc':collision['loc'], 'timestep':collision['timestep']},
                   {'agent':collision['IDs'][1], 'loc':collision['loc'][::-1], 'timestep':collision['timestep']},]
    return constraints


def disjoint_splitting(collision):
    ##############################
    # Task 4.1: Return a list of (two) constraints to resolve the given collision
    #           Vertex collision: the first constraint enforces one agent to be at the specified location at the
    #                            specified timestep, and the second constraint prevents the same agent to be at the
    #                            same location at the timestep.
    #           Edge collision: the first constraint enforces one agent to traverse the specified edge at the
    #                          specified timestep, and the second constraint prevents the same agent to traverse the
    #                          specified edge at the specified timestep
    #           Choose the agent randomly

    pass


class CBSSolver(object):
    """The high-level search of CBS."""

    def __init__(self, my_map, starts, goals):
        """my_map   - list of lists specifying obstacle positions
        starts      - [(x1, y1), (x2, y2), ...] list of start locations
        goals       - [(x1, y1), (x2, y2), ...] list of goal locations
        """

        self.my_map = my_map
        self.starts = starts
        self.goals = goals
        self.num_of_agents = len(goals)

        self.num_of_generated = 0
        self.num_of_expanded = 0
        self.passes = 0
        self.CPU_time = 0

        self.open_list = []

        # compute heuristics for the low-level search
        self.heuristics = []
        for goal in self.goals:
            self.heuristics.append(compute_heuristics(my_map, goal))

    def push_node(self, node):
        heapq.heappush(self.open_list, (node['cost'], len(node['collisions']), self.num_of_generated ,node))
        print("Generate node {}".format(self.num_of_generated))
        self.num_of_generated += 1

    def pop_node(self):
        _, _, id, node = heapq.heappop(self.open_list)
        print("Expand node {}".format(id))
        self.num_of_expanded += 1
        return node

    def find_solution(self, disjoint=True):
        """ Finds paths for all agents from their start locations to their goal locations

        disjoint    - use disjoint splitting or not
        """
        
        self.start_time = timer.time()

        # Generate the root node
        # constraints   - list of constraints
        # paths         - list of paths, one for each agent
        #               [[(x11, y11), (x12, y12), ...], [(x21, y21), (x22, y22), ...], ...]
        # collisions     - list of collisions in paths
        root = {'cost': 0,
                'constraints': [],
                'paths': [],
                'collisions': []}
        for i in range(self.num_of_agents):  # Find initial path for each agent
            path = a_star(self.my_map, self.starts[i], self.goals[i], self.heuristics[i],
                          i, root['constraints'])
            if path is None:
                raise BaseException('No solutions')
            root['paths'].append(path)

        root['cost'] = get_sum_of_cost(root['paths'])
        root['collisions'] = detect_collisions(root['paths'])
        self.push_node(root)
        


        ##############################
        # Task 3.3: High-Level Search
        #           Repeat the following as long as the open list is not empty:
        #             1. Get the next node from the open list (you can use self.pop_node()
        #             2. If this node has no collision, return solution
        #             3. Otherwise, choose the first collision and convert to a list of constraints (using your
        #                standard_splitting function). Add a new child node to your open list for each constraint
        #           Ensure to create a copy of any objects that your child nodes might inherit

        while len(self.open_list) != 0:

            # Pop node from OPEN List with the least cost
            p = self.pop_node()

            # Print Relevant Info
            print('PASS {a}, COST:{b} '.format(a = self.passes, b = p['cost']))
            print(p['collisions'])

            # Check if collision free
            if len(p['collisions']) == 0:
                return p['paths']
            # ======================================
            # Collisions have been found thus continue geenrating new nodes
            # ======================================

            # Select one of the collisions in Node P to split from
            selected_collision = random.choice(p['collisions'])
            # Generate constraints for this collision
            constraints = standard_splitting(selected_collision)
            # New node enforcing each of the generated constraints
            for constraint in constraints:
                # NEW NODE Q
                q = {}
                
                # Inherit constraints from p and add the new one
                q['constraints'] = p['constraints']
                q['constraints'].append(constraint)
                # Inherit paths
                q['paths'] = p['paths']
                
                # Find new path for agent involved in the collision we are trying to avoid
                agent_id = constraint['agent']
                path = a_star(self.my_map, self.starts[agent_id], self.goals[agent_id], self.heuristics[agent_id],
                          agent_id, q['constraints'])

                if path != None:

                    # Replace path by the new one
                    q['paths'][agent_id] = path
                    # Detect collisions
                    q['collisions'] = detect_collisions(q['paths'])
                    q['cost'] = get_sum_of_cost(q['paths'])
                    self.push_node(q)
        
        return None

        


    def print_results(self, node):
        print("\n Found a solution! \n")
        CPU_time = timer.time() - self.start_time
        print("CPU time (s):    {:.2f}".format(CPU_time))
        print("Sum of costs:    {}".format(get_sum_of_cost(node['paths'])))
        print("Expanded nodes:  {}".format(self.num_of_expanded))
        print("Generated nodes: {}".format(self.num_of_generated))
