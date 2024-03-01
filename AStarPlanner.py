import numpy as np
from heapq import *
class Node:
    def __init__(self,state,parent):
        self.state = state
        self.parent = parent
        self.g_value = 0
        self.h_value = 0
    def __lt__(self, other):  # in case of duplicate values
        return self.g_value + self.h_value < other.g_value + self.h_value

class AStarPlanner(object):    
    def __init__(self, planning_env, epsilon = 1):
        self.planning_env = planning_env
        self.epsilon = epsilon
        # used for visualizing the expanded nodes
        # make sure that this structure will contain a list of positions (states, numpy arrays) without duplicates
        self.expanded_nodes = [] 

    def plan(self):
        '''
        Compute and return the plan. The function should return a numpy array containing the states (positions) of the robot.
        '''

        # initialize an empty plan.
        plan = []
        OPEN = []
        # CLOSED_DICT = {}
        # define the variables used for a*
        actions = {"R": (1,0), "L": (-1,0),"U": (0,1),"D": (0,-1), "UL": (-1,1), "UR":(1,1),"DL":(-1,-1), "DR": (1,-1)}
        initial_state = self.planning_env.start
        goal_state = self.planning_env.goal
        
        OPEN.push(Node(initial_state,None))
        
        while len(OPEN) != 0:
            new_node = heappop(OPEN)
        

        self.planning_env.
        return np.array(plan)

    def get_expanded_nodes(self):
        '''
        Return list of expanded nodes without duplicates.
        '''

        # used for visualizing the expanded nodes
        return self.expanded_nodes



class GraphNode:
    def __init__(self,state,parent,action,cost):
        self.state = state
        self.parent = parent
        self.action = action
        self.cost = cost
        self.g_value = 0
        # self.h_value = 0
        # self.f_value = 0
        # self.weight = 0
    def __lt__(self, other):  # in case of duplicate values
        return self.state < other.state

class WeightedAStarAgent(Agent):
    
    def __init__(self):
        self.OPEN = heapdict.heapdict()
        self.CLOSED_DICT = {}
        super().__init__()

    def search(self, env: FrozenLakeEnv, h_weight) -> Tuple[List[int], float, int]:
        initial_state = env.reset()
        initial_node = GraphNode(initial_state, None, None, None)
        if env.is_final_state(initial_node.state):
            return ([], 0, 0)
        self.OPEN = heapdict.heapdict()
        # self.OPEN_SET = dict()
        self.CLOSE = set()
        self.CLOSED_DICT = {} # helpful in case a state is in CLOSED and we need to update it
        self.expanded_counter = 0
        self.OPEN[initial_state] = (initial_node.g_value, initial_node)

        while len(self.OPEN) > 0:
            node_to_expand_idx, (f_value, node_to_expand) = self.OPEN.popitem()
            self.CLOSE = self.CLOSE | {node_to_expand.state}
            self.CLOSED_DICT[node_to_expand.state] = node_to_expand
            if env.is_final_state(node_to_expand.state):
                return self.get_path(node_to_expand, self.expanded_counter)
            self.expanded_counter += 1

            for act, tup in env.succ(node_to_expand.state).items():
                next_state, cost, terminated = tup
                if terminated and not env.is_final_state(next_state):  # it's a hole: continue
                    continue
                elif (next_state not in self.OPEN) and (next_state not in self.CLOSE):
                    # hasn't been discovered yet,create a new graph node
                    new_node = GraphNode(next_state, node_to_expand, act, cost)
                    new_node.g_value = node_to_expand.g_value + cost

                    # w is between 0 and 1, thus we use the other version of wA*:
                    new_f_value = (1 - h_weight)*new_node.g_value + h_weight*self.h_MSAP(env, env.goals, next_state)

                    self.OPEN[next_state] = (new_f_value, new_node)
                elif next_state in self.OPEN:
                    next_node = self.OPEN[next_state][1]
                    new_cost = node_to_expand.g_value + cost
                    if new_cost < next_node.g_value: # since h_value and weight are consistent, the only difference is the g_value.
                        # shorter/cheaper path discovered, need to update g(v) and f(v) in queue:
                        next_node.g_value = new_cost
                        next_node.parent = node_to_expand
                        next_node.action = act
                        next_node.cost = cost
                        new_value = (1 - h_weight)*next_node.g_value + h_weight*self.h_MSAP(env, env.goals, next_state)
                        self.OPEN[next_state] = (new_value, next_node)
                else:
                    curr_node = self.CLOSED_DICT[next_state]
                    new_g_value = (node_to_expand.g_value + cost) 
                    new_f_value = (1-h_weight)*new_g_value + (h_weight)*self.h_MSAP(env, env.goals, next_state)
                    curr_f_value = (1-h_weight)*curr_node.g_value + (h_weight)*self.h_MSAP(env, env.goals, next_state)
                    if new_f_value < curr_f_value:
                        curr_node.g_value = new_g_value
                        curr_node.f_value = new_f_value
                        curr_node.action = act
                        curr_node.cost = cost
                        curr_node.parent = node_to_expand
                        self.OPEN[curr_node.state] = (new_f_value, curr_node)
                        self.CLOSE.remove(next_state)
                        del self.CLOSED_DICT[next_state]
                        # after this the node is in OPEN so we're not going to make a duplicate