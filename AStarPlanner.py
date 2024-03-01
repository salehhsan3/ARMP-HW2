import numpy as np
from heapq import *
class Node:
    def __init__(self,state,parent):
        self.state = state
        self.parent = parent
        self.g_value = 0
        self.h_value = 0
    def __lt__(self, other):  # in case of duplicate values
        return self.g_value + self.h_value < other.g_value + other.h_value

def states_are_equal(state1, state2):
    return state1[0] == state2[0] and state1[1] == state2[1]

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
        CLOSE = []
        # CLOSED_DICT = {}
        # define the variables used for a*
        actions = {"R": (1,0), "L": (-1,0),"U": (0,1),"D": (0,-1), "UL": (-1,1), "UR":(1,1),"DL":(-1,-1), "DR": (1,-1)}
        initial_state = self.planning_env.start
        goal_state = self.planning_env.goal
        
        OPEN.push(Node(initial_state,None))
        
        while len(OPEN) != 0:
            # Pop the new best node
            new_node = heappop(OPEN)
            # for telemetry
            self.expanded_nodes.push(new_node.state)
            CLOSE.push(new_node)

            # expand it's successors
            for action, direction in actions.items():
                succ_state = np.nparray(new_node.state) + np.nparray(direction)
                # TODO: preliminary backround out of bounds checker (on the map successor)

                if(any(states_are_equal(succ_state, n.state) for n in CLOSE)):
                    # better path exists
                    pass
                elif any(states_are_equal(succ_state, n.state) for n in OPEN):
                    # we found a better path
                    # TODO: update old state to be this state
                    pass
                else:
                    heappush(Node(succ_state,new_node, action, -99999))
                    # TODO: add state to open
            
            
        
        return np.array(plan)

    def get_expanded_nodes(self):
        '''
        Return list of expanded nodes without duplicates.
        '''

        # used for visualizing the expanded nodes
        return self.expanded_nodes



class Node:
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
    
    def get_path(self, goal_node: Node):
        lst = []
        node_iter = goal_node
        cost = 0
        while node_iter.parent is not None:
            lst.insert(0,node_iter.action)
            cost += node_iter.cost
            node_iter = node_iter.parent
        return np.array(lst)

    def search(self, env: FrozenLakeEnv, h_weight) -> Tuple[List[int], float, int]:
        plan = []
        OPEN = heapdict.heapdict()
        CLOSED_DICT = {}
        initial_state = self.planning_env.start
        goal_state = self.planning_env.goal
        initial_node = Node(initial_state, None)
        if initial_state == goal_state:
            return np.array(plan)
        CLOSE = set()
        CLOSED_DICT = {} # helpful in case a state is in CLOSED and we need to update it
        self.expanded_counter = 0
        OPEN[initial_state] = (initial_node.g_value, initial_node)

        while len(OPEN) > 0:
            node_to_expand_idx, (f_value, node_to_expand) = OPEN.popitem()
            CLOSE = CLOSE | {node_to_expand.state}
            CLOSED_DICT[node_to_expand.state] = node_to_expand
            if node_to_expand.state == goal_state:
                return self.get_path(node_to_expand, self.expanded_counter)
            self.expanded_counter += 1

            for act, tup in env.succ(node_to_expand.state).items():
                next_state, cost, terminated = tup
                if terminated and not env.is_final_state(next_state):  # it's a hole: continue
                    continue
                elif (next_state not in OPEN) and (next_state not in CLOSE):
                    # hasn't been discovered yet,create a new graph node
                    new_node = Node(next_state, node_to_expand, act, cost)
                    new_node.g_value = node_to_expand.g_value + cost

                    # w is between 0 and 1, thus we use the other version of wA*:
                    new_f_value = (1 - h_weight)*new_node.g_value + h_weight*self.h_MSAP(env, env.goals, next_state)

                    OPEN[next_state] = (new_f_value, new_node)
                elif next_state in OPEN:
                    next_node = OPEN[next_state][1]
                    new_cost = node_to_expand.g_value + cost
                    if new_cost < next_node.g_value: # since h_value and weight are consistent, the only difference is the g_value.
                        # shorter/cheaper path discovered, need to update g(v) and f(v) in queue:
                        next_node.g_value = new_cost
                        next_node.parent = node_to_expand
                        next_node.action = act
                        next_node.cost = cost
                        new_value = (1 - h_weight)*next_node.g_value + h_weight*self.h_MSAP(env, env.goals, next_state)
                        OPEN[next_state] = (new_value, next_node)
                else:
                    curr_node = CLOSED_DICT[next_state]
                    new_g_value = (node_to_expand.g_value + cost) 
                    new_f_value = (1-h_weight)*new_g_value + (h_weight)*self.h_MSAP(env, env.goals, next_state)
                    curr_f_value = (1-h_weight)*curr_node.g_value + (h_weight)*self.h_MSAP(env, env.goals, next_state)
                    if new_f_value < curr_f_value:
                        curr_node.g_value = new_g_value
                        curr_node.f_value = new_f_value
                        curr_node.action = act
                        curr_node.cost = cost
                        curr_node.parent = node_to_expand
                        OPEN[curr_node.state] = (new_f_value, curr_node)
                        CLOSE.remove(next_state)
                        del CLOSED_DICT[next_state]
                        # after this the node is in OPEN so we're not going to make a duplicate