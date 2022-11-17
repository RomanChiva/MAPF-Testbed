"""
This file contains a placeholder for the DistributedPlanningSolver class that can be used to implement distributed planning.

Code in this file is just provided as guidance, you are free to deviate from it.
"""

import time as timer
from single_agent_planner import compute_heuristics, a_star, get_sum_of_cost
from aircraft import AircraftDistributed
from cbs import detect_collision, detect_collisions
import numpy as np
import copy 

class DistributedPlanningSolver(object):
    """A distributed planner"""

    def __init__(self, my_map, starts, goals):
        """my_map   - list of lists specifying obstacle positions
        starts      - [(x1, y1), (x2, y2), ...] list of start locations
        goals       - [(x1, y1), (x2, y2), ...] list of goal locations
        """
        self.CPU_time = 0
        self.map = my_map
        self.starts = starts
        self.goals = goals
        
        # T.B.D.
        

    def agent_take_action(self, agent, current_map):

        # Retrieve agent's position
        x = agent.position[0]
        y = agent.position[1]

        # Pad the map with walls
        current_map = np.pad(current_map, pad_width=1, constant_values = 1)

        # Take out a snippet of the observation
        observation = current_map[x:x+3, y:y+3]
        #print(f'{agent.ID} :: {observation}')

        #Feed it to the agent and take step (Returns boolean. True if agent is at goal location)
        agent.step(observation)
        


    def generate_current_map(self, agents):
        
        # Create a copy of the map
        map = copy.deepcopy(self.map)
        # Find the positions of the agents
        positions = [agent.position for agent in agents]
        # Set other agents as obstacles
        for x in positions:
            map[x[0]][x[1]] = 1 
        
        return map


    def find_solution(self):
        """
        Finds paths for all agents from start to goal locations. 
        
        Returns:
            result (list): with a path [(s,t), .....] for each agent.
        """
        # Initialize constants       
        start_time = timer.time()
        result = []
        searching = True
        

        # Spawn Agents
        agents = [AircraftDistributed(self.starts[i], self.goals[i],i) for i in range(len(self.goals))]

        # Random Activation Scheduler
        while searching:
            
            # Only COnduct a step for agents which haven't reached their goal location yet
            mask = [agent.searching for agent in agents]
            searching_agents = [b for a, b in zip(mask,agents) if a]

            # Check if we are done
            if len(searching_agents) == 0:
                searching = False
                continue
            
            # Agents randomly ordered
            execution_order = np.random.choice(searching_agents, size=len(searching_agents), replace= False)

            for agent in execution_order:
                map = self.generate_current_map(agents)
                self.agent_take_action(agent,map)
        
            
        
        result = [agent.trajectory for agent in agents]
        

        self.CPU_time = timer.time() - start_time
        # Print final output
        print("\n Found a solution! \n")
        print("CPU time (s):    {:.2f}".format(self.CPU_time))
        print("Sum of costs:    {}".format(get_sum_of_cost(result)))  # Hint: think about how cost is defined in your implementation
        print(result)
        
        return result  # Hint: this should be the final result of the distributed planning (visualization is done after planning)