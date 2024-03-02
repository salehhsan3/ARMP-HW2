import argparse
from MapEnvironment import MapEnvironment
from MapEnvironment import *
from RRTPlanner import RRTPlanner
from RRTStarPlanner import RRTStarPlanner
from AStarPlanner import AStarPlanner
import time

def run_and_average(args, runs_num, planning_env):
  total_time = 0
  total_value = 0
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
    # Record the end time and calculate the elapsed time
    end_time = time.perf_counter()
    elapsed_time = end_time - start_time
    # Add the elapsed time and return value to the totals
    total_time += elapsed_time
    total_value += plan_cost
    path = 'images\\rrt\\E2\\bias=0.05'
    planner.planning_env.visualize_map(plan=plan, tree_edges=planner.tree.get_edges_as_states(), path=path, ind=i)
  # Calculate and return the averages
  avg_time = total_time / runs_num
  avg_value = total_value / runs_num
  return avg_time, avg_value

if __name__ == "__main__":

    parser = argparse.ArgumentParser(description='script for testing planners')
    parser.add_argument('-map', '--map', type=str, default='map1.json', help='Json file name containing all map information')
    parser.add_argument('-planner', '--planner', type=str, default='astar', help='The planner to run. Choose from [astar, rrt, rrtstar]')
    parser.add_argument('-ext_mode', '--ext_mode', type=str, default='E1', help='edge extension mode for RRT and RRTStar')
    parser.add_argument('-goal_prob', '--goal_prob', type=float, default=0.05, help='probability to draw goal vertex for RRT and RRTStar')
    parser.add_argument('-k', '--k', type=int, default=1, help='number of nearest neighbours for RRTStar')
    args = parser.parse_args()

    # prepare the map
    planning_env = MapEnvironment(json_file=args.map)

    # execute plan
    runs_num = 10
    avg_time, avg_cost = run_and_average(args, runs_num, planning_env)
    print('Average cost of path: {:.2f}'.format(avg_cost))
    print('Average time: {:.2f} seconds'.format(avg_time))
    
# the following line is used to run this file:
# python run_statistics.py -map map2.json -planner rrt -ext_mode E1 -goal_prob 0.05

# to run RRT*, add a value for 'k' - the number of nearest neighbors