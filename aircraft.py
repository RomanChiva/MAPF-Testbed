"""
This file contains the AircraftDistributed class that can be used to implement individual planning.

Code in this file is just provided as guidance, you are free to deviate from it.
"""

import numpy as np


class AircraftDistributed(object):
    """Aircraft object to be used in the distributed planner."""

    def __init__(self, start, goal, path,ID):
        """
        my_map   - list of lists specifying obstacle positions
        starts      - (x1, y1) start location
        goals       - (x1, y1) goal location
        heuristics  - heuristic to goal location
        """
        # Agent knowledge
        self.position = start
        self.goal = goal
        self.trajectory = [start]
        self.ID = ID
        self.epsilon = 0.3
        self.path = path
        self.last_move = None

        # Agent Internal states
        self.searching= True
        self.following_path = True
        self.retreat = False
        self.bifurcation = False
        


        # Available actions
        self.actions = np.array([[0,1],[1,0],[0,-1],[-1,0]])
        self.action_reverse = [2,3,0,1]

    def unit_vector(self):
        # Give the prefered heading direction as a unit vector
        vector = np.asarray(self.goal) - np.asarray(self.position)
        return vector/np.linalg.norm(vector)

    def process_observation(self, observation):
        # (This counts as both walls and agents)
        # Returns an array of booleans with true if there is an obstacle
        obstacle_bool = np.ones(4)
        # Check the map
        obstacle_bool[0] = observation[1][2] # Right
        obstacle_bool[1] = observation[2][1] # Down
        obstacle_bool[2] = observation[1][0] # Left
        obstacle_bool[3] = observation[0][1] # Up

        return obstacle_bool.astype(bool)

    def step(self, observation):
        
        # Process observation
        obstacle_vector = self.process_observation(observation)

        if self.bifurcation:
            obstacle_vector[self.action_reverse[self.last_move]] = 1
            self.bifurcation = False

        if self.retreat:
            # Update constraint vector
            obstacle_vector[self.action_reverse[self.last_move]] = 1
            # Check if you are free to move again
            if sum(obstacle_vector) < 3:
                self.retreat = False
                self.bifurcation = True
                self.trajectory.append(self.position)
            # If completely blocked (by another agent), wait
            elif sum(obstacle_vector) == 4:
                self.trajectory.append(self.position)
            # Else, retreat
            else:
                escape_route  = int(np.where(obstacle_vector==False)[0])
                self.position = tuple(np.asarray(self.position) + np.asarray(self.actions[escape_route]))
                self.trajectory.append(self.position)
                self.last_move = escape_route

        elif self.following_path:
            # Check what the next step in the path is
            next_step = self.path[self.path.index(self.position) +1]
            # Check if there is an obstacle where you are trying to go
            action = [next_step[i]-self.position[i] for i in range(2)]
            action_index = [list(x) for x in self.actions].index(action)

            # Check if cornered
            
            if sum(obstacle_vector) == 3 and self.action_reverse[action_index] == self.last_move:
                self.retreat = True
                self.last_move = action_index
                self.trajectory.append(self.position)
            
            # If there is an obstacle switch modes
            elif obstacle_vector[action_index]:
                self.following_path = False
                self.trajectory.append(self.position)
                self.last_move = action_index
            else:
                self.position = next_step
                self.trajectory.append(self.position)
                self.last_move = action_index
            
            
        else:
        
            if all([obstacle == 1 for obstacle in obstacle_vector]):
                self.trajectory.append(self.position)
                return False
                
            # Calculate unit vector for directions
            heading = self.unit_vector()
            # Preferences: How much each direction is prefered from -1 to 1 using inner product
            weights = np.array([np.inner(action, heading) for action in self.actions])
            # Offset and scale (Add a small boost to differentiate between unfavorable directions and obstacles)
            weights = (weights + 1)/2 + 0.05
            weights[obstacle_vector] = 0 # Set weights to 0 if there is an obstacle in the way
            # Normalize
            probabilities = weights/weights.sum()

            # Pick an action follwing an e-greedy pilicy

            if np.random.random() < self.epsilon:
                selected_action = np.argmax(probabilities)
            else:
                selected_action = np.random.choice([0,1,2,3], p=probabilities)
            
            # Check if cornered
            if sum(obstacle_vector) == 3 and self.action_reverse[selected_action] == self.last_move:
                self.retreat = True
                self.last_move = selected_action
                self.trajectory.append(self.position)
            else:
                # Update your position and add it to the trajectory list
                self.position = tuple(np.asarray(self.position) + np.asarray(self.actions[selected_action]))

                self.trajectory.append(self.position)
                # Store action just taken
                self.last_move = selected_action

                if self.position in self.path:
                    self.following_path = True

        # Check if you are done searching
        if self.position == self.goal:
            self.searching = False

        
        



        












