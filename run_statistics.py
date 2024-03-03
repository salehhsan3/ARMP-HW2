import argparse
from MapEnvironment import MapEnvironment
from MapEnvironment import *
from RRTPlanner import RRTPlanner
from RRTStarPlanner import RRTStarPlanner
from AStarPlanner import AStarPlanner
import time
import numpy as np
import matplotlib.pyplot as plt

def run_and_average(args, runs_num, planning_env):
    total_time = 0
    total_value = 0
    path = f'images\\{args.planner}\\{args.ext_mode}\\bias={args.goal_prob}'
    if args.planner == 'rrtstar':
        if k_value == 1:
            path = f'images\\{args.planner}\\{args.ext_mode}\\bias={args.goal_prob}\\k=log(n)'
        else:
            path = f'images\\{args.planner}\\{args.ext_mode}\\bias={args.goal_prob}\\k={args.k}'
    if not os.path.exists(path):
        # Create the directory and its parent directories recursively
        os.makedirs(path)
    costs = []
    times = []
    for i in range(runs_num):
        # Record the start time
        start_time = time.perf_counter()
        # Run the algorithm
        if args.planner == 'rrt':
            planner = RRTPlanner(planning_env=planning_env, ext_mode=args.ext_mode, goal_prob=args.goal_prob)
        elif args.planner == 'rrtstar':
            planner = RRTStarPlanner(planning_env=planning_env, ext_mode=args.ext_mode, goal_prob=args.goal_prob, k=args.k)
        elif args.planner == 'astar':
            planner = AStarPlanner(planning_env=planning_env,epsilon=1) 
        plan = planner.plan()
        plan_cost = planner.compute_cost(plan)
        costs.append(plan_cost)
        # Record the end time and calculate the elapsed time
        end_time = time.perf_counter()
        elapsed_time = end_time - start_time
        times.append(elapsed_time)
        # Add the elapsed time and return value to the totals
        total_time += elapsed_time
        total_value += plan_cost
        planner.planning_env.visualize_map(plan=plan, tree_edges=planner.tree.get_edges_as_states(), path=path, ind=i)
    # Calculate and return the averages
    avg_time = total_time / runs_num
    avg_value = total_value / runs_num
    with open(os.path.join(path, 'results.txt'), 'w') as f:
        for cost, timer in zip(costs, times):
            f.write('Total cost of path: {:.2f}\n'.format(cost))
            f.write('Total time: {:.2f} seconds\n\n'.format(timer))
    return avg_time, avg_value

if __name__ == "__main__":

    parser = argparse.ArgumentParser(description='script for testing planners')
    parser.add_argument('-map', '--map', type=str, default='map1.json', help='Json file name containing all map information')
    parser.add_argument('-planner', '--planner', type=str, default='astar', help='The planner to run. Choose from [astar, rrt, rrtstar]')
    parser.add_argument('-ext_mode', '--ext_mode', type=str, default='E1', help='edge extension mode for RRT and RRTStar')
    parser.add_argument('-goal_prob', '--goal_prob', type=float, default=0.05, help='probability to draw goal vertex for RRT and RRTStar')
    parser.add_argument('-k', '--k', type=int, default=1, help='number of nearest neighbours for RRTStar')
    args = parser.parse_args()

    success_rates_constant = []
    solution_qualities_constant = []
    success_rates_log = []
    solution_qualities_log = []
    times = []
    k_values = [1, 3, 11, 47, 73]

    # prepare the map
    planning_env = MapEnvironment(json_file=args.map)
    runs_num = 10
    # for planner in ['rrt', 'rrtstar']:
    #     for bias in [0.2, 0.05]:
    #         for extension_policy in ['E1', 'E2']:
    #             for k_value in k_values:
    for planner in ['rrtstar']:
        for bias in [0.2, 0.05]:
            for extension_policy in ['E1', 'E2']:
                for k_value in [1]:
                    args.planner = planner
                    args.goal_prob = bias
                    args.ext_mode = extension_policy
                    path = f'images\\{args.planner}\\{args.ext_mode}\\bias={args.goal_prob}'
                    if planner == "rrtstar":
                        args.k = k_value
                        if k_value == 1:
                            path = f'images\\{args.planner}\\{args.ext_mode}\\bias={args.goal_prob}\\k=log(n)'
                        else:
                            path = f'images\\{args.planner}\\{args.ext_mode}\\bias={args.goal_prob}\\k={args.k}'
                    else:
                        if k_value != k_values[0]:
                            continue
                    
                    # Execute plan
                    avg_time, avg_cost = run_and_average(args, runs_num, planning_env)
                    if not os.path.exists(path):
                        # Create the directory and its parent directories recursively
                        os.makedirs(path)
                    with open(os.path.join(path, 'results.txt'), 'a') as f:
                        # Print and write the results to the file
                        if planner == "rrtstar":
                            if k_value == 1:
                                f.write(f'Planner: {args.planner}, Ext Mode: {args.ext_mode}, Goal Prob: {args.goal_prob}, k: log(n)\n')
                            else:
                                f.write(f'Planner: {args.planner}, Ext Mode: {args.ext_mode}, Goal Prob: {args.goal_prob}, k: {args.k}\n')
                        else:
                            f.write(f'Planner: {args.planner}, Ext Mode: {args.ext_mode}, Goal Prob: {args.goal_prob}\n')
                        f.write('Average cost of path: {:.2f}\n'.format(avg_cost))
                        f.write('Average time: {:.2f} seconds\n\n'.format(avg_time))

    # execute plan
    avg_time, avg_cost = run_and_average(args, runs_num, planning_env)
    print('Average cost of path: {:.2f}'.format(avg_cost))
    print('Average time: {:.2f} seconds'.format(avg_time))
    
# the following line is used to run this file:
# python run_statistics.py -map map2.json -planner rrt -ext_mode E1 -goal_prob 0.05

# to run RRT*, add a value for 'k' - the number of nearest neighbors

# Define a function to run RRT* with a given value of k
# def run_rrt_star(planning_env, args):
#     planner = RRTStarPlanner(planning_env=planning_env, ext_mode=args.ext_mode, goal_prob=args.goal_prob, k=args.k)
#     start_time = time.time()
#     plan = planner.plan()
#     end_time = time.time()
#     success = len(plan) > 0
#     plan_cost = planner.compute_cost(plan)
#     time_taken = end_time - start_time
#     return success, plan_cost, time_taken

# # Define a function to plot the success rate to find a solution as a function of time
# def plot_success_rate(times, success_rates):
#     plt.plot(times, success_rates)
#     plt.xlabel('Time')
#     plt.ylabel('Success Rate')
#     plt.title('Success Rate of RRT* as a Function of Time')
#     plt.show()

# # Define a function to plot the quality of the solution obtained as a function of time
# def plot_solution_quality(times, solution_qualities):
#     plt.plot(times, solution_qualities)
#     plt.xlabel('Time')
#     plt.ylabel('Solution Quality')
#     plt.title('Solution Quality of RRT* as a Function of Time')
#     plt.show()

# def RRT_STAR_compute_plots():
#     # Define parameters
#     k_values = [3, 11, 47, 2663]  # Choose several values for k
#     num_runs = 10
#     planning_env = MapEnvironment(json_file=args.map)  # Initialize your planning environment

#     success_rates_constant = []
#     solution_qualities_constant = []
#     success_rates_log = []
#     solution_qualities_log = []
#     times = []

#     for k in k_values:
#         success_rates = []
#         solution_qualities = []
#         for _ in range(num_runs):
#             success, cost, time_taken = run_rrt_star(planning_env, args)
#             success_rates.append(int(success))
#             solution_qualities.append( cost if success else 0)
#             times.append(time_taken)
#         success_rates_constant.append(np.mean(success_rates))
#         solution_qualities_constant.append(np.mean(solution_qualities))

#     # Plot success rate to find a solution as a function of time
#     plot_success_rate(k_values, success_rates_constant)

#     # Plot the quality of the solution obtained as a function of time
#     plot_solution_quality(k_values, solution_qualities_constant)
