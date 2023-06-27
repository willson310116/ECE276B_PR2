from utils import *
import Planner
from Collision import CollisionChecker

def runtest(args, verbose = True):
  '''
  This function:
   * loads the provided mapfile
   * creates a motion planner
   * plans a path from start to goal
   * checks whether the path is collision free and reaches the goal
   * computes the path length as a sum of the Euclidean norm of the path segments
  '''
  # Load a map and instantiate a motion planner
  boundary, blocks = load_map(args.map_file)

  if args.plan == "AStar":
    MP = Planner.AstarPlanner(boundary, blocks, resolution=args.res)
    # check_grid_valid(MP)
    t0 = tic()
    path, num_considered_node = MP.plan(args.start, args.goal, epsilon=args.eps)

  elif args.plan == "RRT":
    MP = Planner.RRTPlanner(boundary, blocks)
    t0 = tic()
    path, num_considered_node = MP.plan(args.start, args.goal, args.r, args.max_samples, prc=0.1)

  elif args.plan == "RRTConnect":
    MP = Planner.RRTConnectPlanner(boundary, blocks)
    t0 = tic()
    path, num_considered_node = MP.plan(args.start, args.goal, args.r, args.max_samples, prc=0.1)
    
  elif args.plan == "RRTStar":
    MP = Planner.RRTStarPlanner(boundary, blocks)
    t0 = tic()
    path, num_considered_node = MP.plan(args.start, args.goal, args.r, args.max_samples, args.rewire_count, prc=0.1)

  time_spent = toc(t0,"Planning")

  # Display the environment
  if verbose:
    fig, ax, hb, hs, hg = draw_map(boundary, blocks, args.start, args.goal, args.map, args.plan)
    ax.plot(path[:,0],path[:,1],path[:,2],'r-')


  # TODO: You should verify whether the path actually intersects any of the obstacles in continuous space
  # TODO: You can implement your own algorithm or use an existing library for segment and 
  #       axis-aligned bounding box (AABB) intersection
  cc = CollisionChecker(path, ax)
  collision = False
  for block in blocks:
    if cc.check_intersect(block):
      collision = True
      break
  
  goal_reached = sum((path[-1]-args.goal)**2) <= 0.1
  success = (not collision) and goal_reached
  print(f"goal_reached: {goal_reached}")
  print(f"collision: {collision}")

  pathlength = np.sum(np.sqrt(np.sum(np.diff(path,axis=0)**2,axis=1)))
  # plt.savefig(f"./fig/{args.map}_{args.plan}_{args.res}_{args.eps}.png")
  result = f"{args.map}, {args.res}, {args.eps}, {args.plan}, {time_spent}, {pathlength}, {num_considered_node}, {success}\n"

  with open("log.txt", "a") as f:
    f.write(result)

  return success, pathlength

def wrap_test(args):
  verbose = True
  print(f"Running {args.map} test...\n")
  success, pathlength = runtest(args, verbose)
  print(f"Success: {success}")
  print(f"Path length: {pathlength}")
