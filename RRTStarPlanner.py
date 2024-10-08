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
        self.real_k = max(1, int(np.log(1 + len(self.tree.vertices)))) if (self.k == 1) else self.k # 1+log(n) to ensure it's larger than zero -- n could be 1
        self.time_limit = 0
        self.time_interval = 20 # time interval to check solution after
        self.costs = []
        self.times = []
        
    def plan_long_term(self):
        self.plan()
        if len(self.costs) == 0:
            self.costs = [0]
            self.times = [self.time_limit]
        return self.costs, self.times
    
    def rewire_children(self, parent_idx):
        # Get the list of children vertices
        children_idxs = [idx for idx, parent in self.tree.edges.items() if parent == parent_idx]
        
        # Iterate through the children and rewire them if necessary
        for child_idx in children_idxs:
            child_vertex = self.tree.vertices[child_idx]
            
            # Recompute the cost of the child considering the new parent
            new_cost = self.tree.vertices[parent_idx].cost + self.planning_env.compute_distance(self.tree.vertices[parent_idx].state, child_vertex.state)
            
            # If the new cost is lower than the child's current cost, update the child's cost and rewire its children recursively
            if new_cost < child_vertex.cost:
                child_vertex.cost = new_cost
                self.rewire_children(child_idx)
    
    def rewire_rrt_star(self, potential_parent_idx, child_idx):
        # Get the child and potential parent vertices
        child_vertex = self.tree.vertices.get(child_idx)
        potential_parent_vertex = self.tree.vertices.get(potential_parent_idx)

        if child_vertex and potential_parent_vertex:
            # Check if the edge between potential parent and child is valid
            if self.planning_env.edge_validity_checker(potential_parent_vertex.state, child_vertex.state):
                # Compute the cost of the edge between potential parent and child
                edge_cost = self.planning_env.compute_distance(potential_parent_vertex.state, child_vertex.state)

                # Calculate the total cost if we rewire the child to the potential parent
                total_cost = potential_parent_vertex.cost + edge_cost

                # Check if rewiring reduces the cost of the child
                if total_cost < child_vertex.cost:
                    # Update the existing edge if it exists
                    if child_idx in self.tree.edges:
                        # Update the cost of the child
                        child_vertex.cost = total_cost
                        # Update the parent of the child to the potential parent
                        self.tree.edges[child_idx] = potential_parent_idx
                        # Rewire children recursively if necessary
                        self.rewire_children(child_idx)
                    else:
                        # print("Error: Child index not found in tree edges.")
                        pass
        else:
            # print("Error: Child or potential parent vertex not found in tree.")
            pass

    def plan(self):
        '''
        Compute and return the plan. The function should return a numpy array containing the states (positions) of the robot.
        '''
        start_time = time.time()
        # self.time_limit = 300 # time limit for the algorithm's run time
        prev_time = start_time
        # initialize an empty plan.
        plan = []
        # TODO: Task 4.4
        self.tree.add_vertex(self.planning_env.start)
        while (time.time() - start_time < self.time_limit) or (not self.tree.is_goal_exists(self.planning_env.goal)):
            self.real_k = max(1, int(np.log(1 + len(self.tree.vertices)))) if (self.k == 1) else self.k # to update the value!!
            goal_bias = np.random.random()
            x_limit, y_limit = self.planning_env.xlimit, self.planning_env.ylimit
            if goal_bias < self.goal_prob: # correct way??
                random_state = self.planning_env.goal
            else:
                random_state = np.array([np.random.uniform(x_limit[0], x_limit[1]), np.random.uniform(y_limit[0], y_limit[1])])
            nearest_state_idx, nearest_state = self.tree.get_nearest_state(random_state)
            new_state = self.extend(nearest_state, random_state)
            if self.planning_env.state_validity_checker(new_state) and self.planning_env.edge_validity_checker(nearest_state, new_state):
                new_state_idx = self.tree.add_vertex(new_state)
                self.tree.add_edge(nearest_state_idx, new_state_idx, self.planning_env.compute_distance(nearest_state, new_state))
                if len([(_, vertex) for _, vertex in self.tree.vertices.items()]) > self.k: # make sure the state has at least has k neighbors
                    k_nearest_idxs, k_nearest_states = self.tree.get_k_nearest_neighbors(new_state, self.real_k)
                    for idx in k_nearest_idxs:
                        self.rewire_rrt_star(idx, new_state_idx)
                    for idx in k_nearest_idxs:
                        self.rewire_rrt_star(new_state_idx, idx)
            current_time = time.time() - prev_time
            if (self.tree.is_goal_exists(self.planning_env.goal)) and (current_time >= self.time_interval):
                if (time.time() - start_time) <= self.time_limit:
                    plan = self.compute_plan([])
                    self.costs.append(self.compute_cost(plan))
                    self.times.append(time.time() - start_time)
                    prev_time = time.time()
        
        # print total path cost and time
        plan = self.compute_plan([])
        if (time.time() - start_time) <= self.time_limit:
            self.costs.append(self.compute_cost(plan))
            self.times.append(time.time() - start_time)
        
        print('Total cost of path: {:.2f}'.format(self.compute_cost(plan)))
        print('Total time: {:.2f}'.format(time.time()-start_time))

        return np.array(plan)
    
    def compute_plan(self, plan):
        curr_idx = self.tree.get_idx_for_state(self.planning_env.goal)
        start_idx = self.tree.get_idx_for_state(self.planning_env.start)
        while curr_idx != start_idx:
            # print(self.tree.edges)
            plan.append(self.tree.vertices[curr_idx].state)
            curr_idx = self.tree.edges[curr_idx]
        # Add the start state to the plan.
        plan.append(self.planning_env.start)
        plan.reverse()
        return plan

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
        n = 19 # a changeable parameter for step-size
        if self.ext_mode == "E1" or self.planning_env.compute_distance(near_state, rand_state) < n:
            return rand_state
        dist = self.planning_env.compute_distance(near_state, rand_state)
        normed_direction = (rand_state - near_state) / dist # normed vector
        new_state = near_state + (n * normed_direction)
        return new_state
    