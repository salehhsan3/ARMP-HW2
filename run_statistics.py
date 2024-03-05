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
        if int(args.k) == 1:
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

def RRT_Versions_simulators(args):
    k_values = [1, 3, 11, 47, 73]

    # prepare the map
    planning_env = MapEnvironment(json_file=args.map)
    runs_num = 10
    for planner in ['rrt', 'rrtstar']:
        for bias in [0.2, 0.05]:
            for extension_policy in ['E1', 'E2']:
                for k_value in k_values:
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
                        
# Define a function to run RRT* with a given value of k
def run_rrt_star_success(planning_env, args):
    planner = RRTStarPlanner(planning_env=planning_env, ext_mode=args.ext_mode, goal_prob=args.goal_prob, k=args.k)
    start_time = time.time()
    plan = planner.plan() # for success rate plot
    end_time = time.time()
    success = len(plan) > 0
    time_taken = end_time - start_time
    return success, time_taken

# Define a function to plot the success rate to find a solution as a function of time
def plot_success_rate(times, success_rates, k_values):
    for k_value in k_values:
        times_sorted = sorted(times[k_value])
        if k_value == 1:
            plt.plot(times_sorted, success_rates, label=f'k=log(n)', marker='o', linestyle='-')     
        else:
            plt.plot(times_sorted, success_rates, label=f'k={k_value}', marker='o', linestyle='-')
    plt.xlabel('Time')
    plt.ylabel('Success Rate')
    plt.title(f'Success Rate of RRT* as a Function of Time')
    plt.grid(True)
    plt.legend()
    plt.show()

def RRT_STAR_compute_success_rate_plot(args):
    # Define parameters
    k_values = [1, 3, 11, 47, 73]  # Choose several values for k
    num_runs = 10
    planning_env = MapEnvironment(json_file=args.map)  # Initialize your planning environment

    success_rates_constant = [i / num_runs for i in range(1, 1 + num_runs)]
    times = {}
    for k_value in k_values:
        times[k_value] = []

    for k_value in k_values:
        args.k = k_value
        for _ in range(num_runs):
            success, time_taken = run_rrt_star_success(planning_env, args)
            times[k_value].append(time_taken)
        # Plot success rate to find a solution as a function of time
    plot_success_rate(times, success_rates_constant, k_values)
    
# Define a function to plot the quality of the solution obtained as a function of time
def plot_solution_quality(times, costs, k_values):
    for k_value in k_values:
        times_k = (times[k_value])
        costs_k = (costs[k_value])
        if k_value == 1:
            plt.plot(times_k, costs_k, label=f'k=log(n)', marker='o', linestyle='-')     
        else:
            plt.plot(times_k, costs_k, label=f'k={k_value}', marker='o', linestyle='-')
    plt.xlabel('Time')
    plt.ylabel('Solution Quality')
    plt.title(f'Solution Quality of RRT* as a Function of Time')
    plt.grid(True)
    plt.legend()
    plt.show()
    
def run_rrt_star_solution(planning_env, args):
    planner = RRTStarPlanner(planning_env=planning_env, ext_mode=args.ext_mode, goal_prob=args.goal_prob, k=args.k)
    planner.time_interval = args.time_interval
    planner.time_limit = args.time_limit
    costs, times = planner.plan_long_term() # for success rate plot
    return costs, times

def RRT_STAR_compute_solution_quality_plot(args):
    # Define parameters
    k_values = [1, 3, 11, 47, 73]  # Choose several values for k
    num_runs = 10
    planning_env = MapEnvironment(json_file=args.map)  # Initialize your planning environment

    times = {}
    costs = {}
    representative_costs = {}  # Store costs for representative runs
    representative_times = {}  # Store times for representative runs
    min_len = float('inf')
    for k_value in k_values:
        args.k = k_value
        costs[k_value] = []
        times[k_value] = []
        for _ in range(num_runs):
            costs_computed, times_computed = run_rrt_star_solution(planning_env, args)
            times[k_value].append(times_computed)
            costs[k_value].append(costs_computed)
        min_len = min([len(seq) for seq in costs[k_value]])
        
    for k_value in k_values:
        median_idx = num_runs // 2 # try plotting for a specific median_idx and then try calculating the actual value! (safe fail)
        representative_costs[k_value] = costs[k_value][median_idx]
        representative_times[k_value] = times[k_value][median_idx]
    plot_solution_quality(representative_times, representative_costs, k_values)
    
    for k_value in k_values:
        trimmed_costs = [seq[:int(min_len)] for seq in costs[k_value]]
        trimmed_times = [seq[:int(min_len)] for seq in times[k_value]]
        costs[k_value] = trimmed_costs
        times[k_value] = trimmed_times
        avg_list = min(len(seq) for seq in costs[k_value]) * [0]
        for i in range(min(len(seq) for seq in costs[k_value])):
            avg_list[i] = sum([seq[i] for seq in costs[k_value]]) 

        min_diff = float('inf')
        median_idx = None
        for i, cost_list in enumerate(costs[k_value]):
            cost_list = np.array(cost_list) 
            avg_list = np.array(avg_list)
            diff = np.sum(np.abs(cost_list - avg_list))
            if diff < min_diff:
                min_diff = diff
                median_idx = i
        representative_costs[k_value] = costs[k_value][median_idx]
        representative_times[k_value] = times[k_value][median_idx]

    # Plot solution quality for representative runs
    plot_solution_quality(representative_times, representative_costs, k_values)
    
def plot_solution_quality_for_each_k_i(times, costs, k_value, i, figure, path):
    if k_value == 1:
        figure.plot(times, costs, label=f'k=log(n)', marker='o', linestyle='-')     
    else:
        figure.plot(times, costs, label=f'k={k_value}', marker='o', linestyle='-')
    figure.grid(True)
    figure.legend()
    # figure.show()
    
def RRT_STAR_compute_solution_quality_plot_for_each_k_i(args):
    # Define parameters
    k_values = [1, 3, 11, 47, 73]  # Choose several values for k
    num_runs = 10
    planning_env = MapEnvironment(json_file=args.map)  # Initialize your planning environment

    times = {}
    costs = {}
    plots = {}
    for i in range(num_runs):
        plots[i] = plt.figure(i)
        (plots[i]).xlabel('Time')
        (plots[i]).ylabel('Solution Quality')
        (plots[i]).title(f'Solution Quality of RRT* as a Function of Time')
    path = f'images\\{args.planner}\\{args.ext_mode}\\bias={args.goal_prob}\\solution_quality'
    if not os.path.exists(path):
        os.makedirs(path)
    for k_value in k_values:
        args.k = k_value
        costs[k_value] = []
        times[k_value] = []
        for _ in range(num_runs):
            costs_computed, times_computed = run_rrt_star_solution(planning_env, args)
            plot_solution_quality_for_each_k_i(times_computed, costs_computed, k_value,i, plots[k_value][i], )
            
    for i in range(num_runs):
        plots[k_value][i] = plt.figure(i)
        (plots[k_value][i]).xlabel('Time')
        (plots[k_value][i]).ylabel('Solution Quality')
        (plots[k_value][i]).title(f'Solution Quality of RRT* as a Function of Time')
        filename = f'figure_i={i}.png'
        filepath = os.path.join(path, filename)
        plt.savefig(filepath)

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='script for testing planners')
    parser.add_argument('-map', '--map', type=str, default='map1.json', help='Json file name containing all map information')
    parser.add_argument('-planner', '--planner', type=str, default='astar', help='The planner to run. Choose from [astar, rrt, rrtstar]')
    parser.add_argument('-ext_mode', '--ext_mode', type=str, default='E1', help='edge extension mode for RRT and RRTStar')
    parser.add_argument('-goal_prob', '--goal_prob', type=float, default=0.05, help='probability to draw goal vertex for RRT and RRTStar')
    parser.add_argument('-k', '--k', type=int, default=1, help='number of nearest neighbours for RRTStar')
    parser.add_argument('-time_limit', '--time_limit', type=int, default=0, help='number of seconds to limit the running time of RRT*')
    parser.add_argument('-time_interval', '--time_interval', type=int, default=15, help='number of seconds to record an intermediate solution in RRT*')
    args = parser.parse_args()
    
    # RRT_Versions_simulators(args)
    # RRT_STAR_compute_success_rate_plot(args)
    RRT_STAR_compute_solution_quality_plot(args)
