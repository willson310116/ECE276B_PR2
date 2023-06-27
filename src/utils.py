import time
import argparse
import numpy as np
import matplotlib.pyplot as plt; plt.ion()
from mpl_toolkits.mplot3d import Axes3D
from mpl_toolkits.mplot3d.art3d import Poly3DCollection

def command_line_param():
    parser = argparse.ArgumentParser()
    parser.add_argument("-r", "--res", type=float, default=0.1, help="res")
    parser.add_argument("-m", "--map", type=str, default="single_cube", help="which map to use [single_cube, maze, window, tower, flappy_bird, room, monza]")
    parser.add_argument("-e", "--eps", type=float, default=1.0, help="epsilon for weighted A*")
    parser.add_argument("-p", "--plan", type=str, default="AStar", help="Planning algorithm to use [AStar, RRT, RRTStar, RRTConnect]")
    parser.add_argument("--r", type=float, default=0.025, help="length of smallest edge to check for intersection with obstacles")
    parser.add_argument("--max_samples", type=int, default=50000, help="max number of samples to take before timing out")
    parser.add_argument("--rewire_count", type=int, default=32, help="optional, number of nearby branches to rewire")
    args = parser.parse_args()
    start, goal, map_file = get_map_config(args.map)
    args.start = start
    args.goal = goal
    args.map_file = map_file
        
    for arg in vars(args):
        print(f'{arg} = {getattr(args, arg)}')
    print()  
    return args

def get_map_config(name):
    start, goal, map_file = None, None, None
    if name == "single_cube":
        start = np.array([2.3, 2.3, 1.3])
        goal = np.array([7.0, 7.0, 5.5])
        map_file = "../maps/single_cube.txt"

    elif name == "maze":
        start = np.array([0.0, 0.0, 1.0])
        goal = np.array([12.0, 12.0, 5.0])
        map_file = "../maps/maze.txt"

    elif name == "window":
        start = np.array([0.2, -4.9, 0.2])
        goal = np.array([6.0, 18.0, 3.0])
        map_file = "../maps/window.txt"

    elif name == "tower":
        start = np.array([2.5, 4.0, 0.5])
        goal = np.array([4.0, 2.5, 19.5])
        map_file = "../maps/tower.txt"

    elif name == "flappy_bird":
        start = np.array([0.5, 2.5, 5.5])
        goal = np.array([19.0, 2.5, 5.5])
        map_file = "../maps/flappy_bird.txt"

    elif name == "room":
        start = np.array([1.0, 5.0, 1.5])
        goal = np.array([9.0, 7.0, 1.5])
        map_file = "../maps/room.txt"
    
    elif name == "monza":
        start = np.array([0.5, 1.0, 4.9])
        goal = np.array([3.8, 1.0, 0.1])
        map_file = "../maps/monza.txt"
    
    else:
        print("Invalid input!")
    return start, goal, map_file

def tic():
  return time.time()

def toc(tstart, nm=""):
  time_spent = time.time() - tstart
  print('%s took: %s sec.\n' % (nm,time_spent))
  return time_spent

def load_map(fname):
  '''
  Loads the bounady and blocks from map file fname.
  
  boundary = [['xmin', 'ymin', 'zmin', 'xmax', 'ymax', 'zmax','r','g','b']]
  
  blocks = [['xmin', 'ymin', 'zmin', 'xmax', 'ymax', 'zmax','r','g','b'],
            ...,
            ['xmin', 'ymin', 'zmin', 'xmax', 'ymax', 'zmax','r','g','b']]
  '''
  mapdata = np.loadtxt(fname,dtype={'names': ('type', 'xmin', 'ymin', 'zmin', 'xmax', 'ymax', 'zmax','r','g','b'),\
                                    'formats': ('S8','f', 'f', 'f', 'f', 'f', 'f', 'f','f','f')})
  blockIdx = mapdata['type'] == b'block'
  boundary = mapdata[~blockIdx][['xmin', 'ymin', 'zmin', 'xmax', 'ymax', 'zmax','r','g','b']].view('<f4').reshape(-1,11)[:,2:]
  blocks = mapdata[blockIdx][['xmin', 'ymin', 'zmin', 'xmax', 'ymax', 'zmax','r','g','b']].view('<f4').reshape(-1,11)[:,2:]
  return boundary, blocks

def draw_map(boundary, blocks, start, goal, map_name, planner):
  '''
  Visualization of a planning problem with environment boundary, obstacle blocks, and start and goal points
  '''
  fig = plt.figure()
  ax = fig.add_subplot(111, projection='3d')
  hb = draw_block_list(ax,blocks)
  hs = ax.plot(start[0:1],start[1:2],start[2:],'ro',markersize=7,markeredgecolor='k')
  hg = ax.plot(goal[0:1],goal[1:2],goal[2:],'go',markersize=7,markeredgecolor='k')  
  ax.set_xlabel('X')
  ax.set_ylabel('Y')
  ax.set_zlabel('Z')
  ax.set_xlim(boundary[0,0],boundary[0,3])
  ax.set_ylim(boundary[0,1],boundary[0,4])
  ax.set_zlim(boundary[0,2],boundary[0,5])
  fig.suptitle(f"{map_name} with {planner}")

  return fig, ax, hb, hs, hg

def draw_block_list(ax,blocks):
  '''
  Subroutine used by draw_map() to display the environment blocks
  '''
  v = np.array([[0,0,0],[1,0,0],[1,1,0],[0,1,0],[0,0,1],[1,0,1],[1,1,1],[0,1,1]],dtype='float')
  f = np.array([[0,1,5,4],[1,2,6,5],[2,3,7,6],[3,0,4,7],[0,1,2,3],[4,5,6,7]])
  clr = blocks[:,6:]/255
  n = blocks.shape[0]
  d = blocks[:,3:6] - blocks[:,:3] 
  vl = np.zeros((8*n,3))
  fl = np.zeros((6*n,4),dtype='int64')
  fcl = np.zeros((6*n,3))
  for k in range(n):
    vl[k*8:(k+1)*8,:] = v * d[k] + blocks[k,:3]
    fl[k*6:(k+1)*6,:] = f + k*8
    fcl[k*6:(k+1)*6,:] = clr[k,:]
  
  if type(ax) is Poly3DCollection:
    ax.set_verts(vl[fl])
  else:
    pc = Poly3DCollection(vl[fl], alpha=0.25, linewidths=1, edgecolors='k')
    pc.set_facecolor(fcl)
    h = ax.add_collection3d(pc)
    return h

def get_area(obj):
    # obj boundary
    return (obj[3]-obj[0]) * (obj[4]-obj[1]) * (obj[5]-obj[2])

def check_grid_valid(astar):
    """
    check grid-total_space area ratio of decretization in astar
    """
    total_area = get_area(astar.boundary)
    block_area = 0
    for block in astar.blocks:
        block_area += get_area(block)
    true_ratio = block_area / total_area
    
    l,w,h = astar.grid.shape
    grid_ratio = astar.grid.sum() / (l * w * h)
    print(f"True ratio: {true_ratio}")
    print(f"Grid ratio: {grid_ratio}")