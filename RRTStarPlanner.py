import numpy as np
from RRTTree import RRTTree
import time

class RRTStarPlanner(object):

    def __init__(self, planning_env, ext_mode, goal_prob, k):

        # set environment and search tree
        self.planning_env = planning_env
        self.tree = RRTTree(self.planning_env)

        # set search params
        self.ext_mode = ext_mode
        self.goal_prob = goal_prob
        self.k = k
        
    def rewire_rrt_star(self, potential_parent_idx, child_idx):
        child, potential_parent = self.tree.vertices[child_idx], self.tree.vertices[potential_parent_idx]
        if self.planning_env.edge_validity_checker( potential_parent.state, child.state):
            edge_cost = self.planning_env.compute_distance( potential_parent.state, child.state)
            if (potential_parent.cost + edge_cost) < child.cost:
                self.tree.add_edge(potential_parent_idx, child_idx, potential_parent.cost + edge_cost)


    def plan(self):
        '''
        Compute and return the plan. The function should return a numpy array containing the states (positions) of the robot.
        '''
        start_time = time.time()

        # initialize an empty plan.
        plan = []

        # TODO: Task 4.4
        self.tree.add_vertex(self.planning_env.start)
        while not self.tree.is_goal_exists(self.planning_env.goal):
            goal_bias = np.random.random()
            x_limit, y_limit = self.planning_env.xlimit, self.planning_env.ylimit
            if goal_bias < self.goal_prob: # correct way??
                random_state = np.array([np.random.uniform(x_limit[0], x_limit[1]), np.random.uniform(y_limit[0], y_limit[1])])
            else:
                random_state = self.planning_env.goal
            nearest_state_idx, nearest_state = self.tree.get_nearest_state(random_state)
            new_state = self.extend(nearest_state, random_state)
            if self.planning_env.state_validity_checker(new_state) and self.planning_env.edge_validity_checker(nearest_state, new_state):
                self.tree.add_vertex(new_state)
                self.tree.add_edge(nearest_state_idx, self.tree.get_idx_for_state(new_state), self.planning_env.compute_distance(nearest_state, new_state))
                new_state_idx = self.tree.get_idx_for_state(new_state)
                k_nearest_idxs, k_nearest_states = self.tree.get_k_nearest_neighbors(new_state, self.k)
                for idx in k_nearest_idxs:
                    self.rewire_rrt_star(idx, new_state_idx)
                for idx in k_nearest_idxs:
                    self.rewire_rrt_star(new_state_idx, idx)
        
        # print total path cost and time
        plan.append(self.planning_env.goal)
        curr_idx = self.tree.get_idx_for_state(self.planning_env.goal)
        start_idx = self.tree.get_idx_for_state(self.planning_env.start)
        while curr_idx != start_idx:
            curr_idx = self.tree.edges[curr_idx]
            next_state = self.tree.vertices[curr_idx].state
            plan.append(next_state)
        plan.reverse()
        
        print('Total cost of path: {:.2f}'.format(self.compute_cost(plan)))
        print('Total time: {:.2f}'.format(time.time()-start_time))

        return np.array(plan)

    def compute_cost(self, plan):
        '''
        Compute and return the plan cost, which is the sum of the distances between steps.
        @param plan A given plan for the robot.
        '''
        # TODO: Task 4.4
        cost = 0
        for i in range(1, len(plan)):
            cost += self.planning_env.compute_distance(plan[i-1],plan[i])
        return cost

    def extend(self, near_state, rand_state):
        '''
        Compute and return a new position for the sampled one.
        @param near_state The nearest position to the sampled position.
        @param rand_state The sampled position.
        '''
        # TODO: Task 4.4
        n = 0.2 # a changeable parameter
        if self.ext_mode == "E1":
            return rand_state
        dist = self.planning_env.compute_distance(near_state, rand_state)
        direction = (rand_state - near_state) / dist
        new_state = rand_state + (n * dist * direction)
        return new_state
    