import numpy as np
from pqdict import pqdict

import sys
sys.path.insert(1, 'rrt-algorithms')
from src.rrt.rrt import RRT
from src.rrt.rrt_star import RRTStar
from src.rrt.rrt_connect import RRTConnect
from src.search_space.search_space import SearchSpace

class RRTreeBase:
    def __init__(self, boundary, blocks):
        b_list = []
        for b in blocks:
            b_list += [b[:6]]   
        x_min, y_min, z_min, x_max, y_max, z_max = boundary[0,:6]

        self.X_dimensions = np.array([(x_min, x_max), (y_min, y_max), (z_min, z_max)])
        self.Obstacles = np.array(b_list)
        
class RRTPlanner(RRTreeBase):
    def __init__(self, boundary, blocks):
        super().__init__(boundary, blocks)
    
    def plan(self, start_pos, goal_pos, r, max_samples, prc=0.1):
        x_init = tuple(start_pos)  # starting location
        x_goal = tuple(goal_pos)  # goal location
        Q = np.array([(1, 1)])  # length of tree edges
        
        # create Search Space
        X = SearchSpace(self.X_dimensions, self.Obstacles)
        # create rrt_search
        rrt = RRT(X, Q, x_init, x_goal, max_samples, r, prc)
        path, num_considered_node = rrt.rrt_search()
        path = np.array(path)
        return path, num_considered_node

class RRTConnectPlanner(RRTreeBase):
    def __init__(self, boundary, blocks):
        super().__init__(boundary, blocks)
    
    def plan(self, start_pos, goal_pos, r, max_samples, prc=0.1):
        x_init = tuple(start_pos)  # starting location
        x_goal = tuple(goal_pos)  # goal location
        Q = np.array([(1, 1, 1)])  # length of tree edges
        
        # create Search Space
        X = SearchSpace(self.X_dimensions, self.Obstacles)
        # create rrt_search
        rrt = RRTConnect(X, Q, x_init, x_goal, max_samples, r, prc)
        path, num_considered_node = rrt.rrt_connect()
        path = np.array(path)
        return path, num_considered_node

class RRTStarPlanner(RRTreeBase):
    def __init__(self, boundary, blocks):
        super().__init__(boundary, blocks)
    
    def plan(self, start_pos, goal_pos, r, max_samples, rewire_count=32, prc=0.1):
        x_init = tuple(start_pos)  # starting location
        x_goal = tuple(goal_pos)  # goal location
        Q = np.array([(1, 1, 1)])  # length of tree edges
        
        # create Search Space
        X = SearchSpace(self.X_dimensions, self.Obstacles)
        # create rrt_search
        rrt = RRTStar(X, Q, x_init, x_goal, max_samples, r, prc, rewire_count)
        path, num_considered_node = rrt.rrt_star()
        path = np.array(path)
        return path, num_considered_node

class AstarPlanner:
    '''
    Weighted A* on grid map with obstacles
    Using Octile distance as the heursitic function
    '''
    def __init__(self, boundary, blocks, resolution=0.1):
        self.boundary = boundary[0,:6]
        self.x_min, self.y_min, self.z_min, \
        self.x_max, self.y_max, self.z_max = boundary[0,:6]

        self.blocks = blocks
        self.resolution = resolution
        self.mesh = self.create_mesh_3d()
        self.grid = np.zeros(self.mesh.shape[1:])
        self.specify_obstacle()
        
        parent = np.empty((self.grid.shape + (3,)))
        parent.fill(-1)
        # parent[tuple([0,0,2])] = child position
        self.parent = parent
        
        self.open_list = None
        self.close_list = None
        self.g_grid = None

    def create_mesh_3d(self):
        # Extract boundary information
        
        # Calculate the number of vertices in each dimension
        num_vertices_x = int((self.x_max - self.x_min) / self.resolution) + 1
        num_vertices_y = int((self.y_max - self.y_min) / self.resolution) + 1
        num_vertices_z = int((self.z_max - self.z_min) / self.resolution) + 1
        # num_vertices_x = int(np.ceil((x_max - x_min) / self.resolution))
        # num_vertices_y = int(np.ceil((y_max - y_min) / self.resolution))
        # num_vertices_z = int(np.ceil((z_max - z_min) / self.resolution))
        
        
        # Create mesh grid
        x = np.linspace(self.x_min, self.x_max, num=num_vertices_x)
        y = np.linspace(self.y_min, self.y_max, num=num_vertices_y)
        z = np.linspace(self.z_min, self.z_max, num=num_vertices_z)
        
        # x = np.arange(0, num_vertices_x+1) * self.resolution + self.x_min
        # y = np.arange(0, num_vertices_y+1) * self.resolution + self.y_min
        # z = np.arange(0, num_vertices_z+1) * self.resolution + self.z_min
        
        
        # mesh = np.meshgrid(x, y, z)
        mesh = np.stack(np.meshgrid(x, y, z, indexing='ij'))
        
        return mesh
    
    def get_block_indices(self, obstacle_boundary):
        # idx = np.where((self.mesh[0] >= obstacle_boundary[0]) & (self.mesh[0] <= obstacle_boundary[3]) &
        #                 (self.mesh[1] >= obstacle_boundary[1]) & (self.mesh[1] <= obstacle_boundary[4]) &
        #                 (self.mesh[2] >= obstacle_boundary[2]) & (self.mesh[2] <= obstacle_boundary[5]))
        # idx = np.vstack((idx[0], idx[1], idx[2])).T
        # return idx
        pos_min = obstacle_boundary[:3]
        pos_max = obstacle_boundary[3:6]
        g_min = self.cord_to_grid(pos_min)
        g_max = self.cord_to_grid(pos_max)
        indices = np.indices(self.grid.shape)
        mask = (indices[0] >= g_min[0]) & (indices[0] <= g_max[0]) & \
                (indices[1] >= g_min[1]) & (indices[1] <= g_max[1]) & \
                (indices[2] >= g_min[2]) & (indices[2] <= g_max[2])
        result_indices = np.argwhere(mask)
        return result_indices
    
    def specify_obstacle(self):
        for block in self.blocks:
            block_idx = self.get_block_indices(block)
            self.grid[tuple(block_idx.T)] = 1
    
    def build_grid(self):
        grid = np.zeros(self.mesh.shape[1:])
        self.specify_obstacle(grid)
        return grid
    
    def pos_to_str(self, pos):
        # pos is in grid coordinate
        return "_".join([str(i) for i in pos])

    def str_to_pos(self, pos_string):
        # pos_string is in grid coordinate
        return np.array([int(i) for i in pos_string.split("_")])

    def cord_to_grid(self, pos):
        # pos is in env(real) coordinate
        x = int((pos[0] - self.x_min)  / self.resolution)
        y = int((pos[1] - self.y_min)  / self.resolution)
        z = int((pos[2] - self.z_min)  / self.resolution)
        return tuple((x,y,z))
    
    def grid_to_cord(self, idx):
        x = idx[0] * self.resolution + self.x_min
        y = idx[1] * self.resolution + self.y_min
        z = idx[2] * self.resolution + self.z_min
        return [x,y,z]

    def get_hValue(self, cur_pos, goal_pos):
        # Octile distance in grid coordinate
        cur_pos = np.array(cur_pos)
        goal_pos = np.array(goal_pos)
        return max(abs(goal_pos - cur_pos)) + (len(goal_pos)**0.5 - 1) * min(abs(goal_pos - cur_pos))

    def get_fValue(self, cur_pos, goal_pos, epsilon):
        # cur_pos, goal_pos: tuple
        return self.g_grid[cur_pos] + epsilon * self.get_hValue(cur_pos, goal_pos)

    def check_boundary(self, cur_pos):
        # in grid coordinate
        x_max,y_max,z_max = self.grid.shape
        x,y,z = cur_pos
        
        return (x >= 0 and x < x_max and
                y >= 0 and y < y_max and
                z >= 0 and z < z_max)

    def get_path_from_parent(self, start_id_str, goal_id_str):
        path = []
        cur = tuple(self.str_to_pos(goal_id_str))
        while cur != tuple(self.str_to_pos(start_id_str)):
            path = [cur] + path
            cur = tuple(self.parent[cur].astype(int))
        path = [cur] + path
        path_grid = np.array(path)

        path_env = []
        for p in path_grid:
            x,y,z = self.grid_to_cord(p)
            # x = p[0] * self.resolution + self.x_min
            # y = p[1] * self.resolution + self.y_min
            # z = p[2] * self.resolution + self.z_min
            path_env += [[x,y,z]]
        
        return np.array(path_env)

    def plan(self, start_pos, goal_pos, epsilon=1.0):
        # actions
        [dX,dY,dZ] = np.meshgrid([-1,0,1],[-1,0,1],[-1,0,1])
        dR = np.vstack((dX.flatten(),dY.flatten(),dZ.flatten()))
        dR = np.delete(dR,13,axis=1)
        dR = dR.T

        # default min heap, grid_pos_str:h-val
        self.open_list = pqdict()
        # init empty close list ()
        self.close_list = set()

        # initialize A* alg
        start_id = self.cord_to_grid(start_pos)
        goal_id = self.cord_to_grid(goal_pos)
        start_id_str = self.pos_to_str(start_id)
        goal_id_str = self.pos_to_str(goal_id)

        # initialize g-values
        self.g_grid = np.empty(self.grid.shape)
        self.g_grid.fill(float("inf"))
        self.g_grid[start_id] = 0

        # add start into open_list
        self.open_list[start_id_str] = self.get_fValue(start_id, goal_id, epsilon=epsilon)
        last_id_str = start_id_str
        num_considered_node = 0

        # start planning
        while goal_id_str not in self.close_list:
            if len(self.open_list) == 0:
                print(f"Fail to find a way!")
                path = self.get_path_from_parent(start_id_str, last_id_str)
                return path
            
            # remove node with smallest f-value from OPEN
            # open_list -> pos str -> pos array
            cur_str = self.open_list.pop()
            num_considered_node += 1
            # cur_pos in grid coordinate
            cur_pos = self.str_to_pos(cur_str)
            # insert node into CLOSE
            self.close_list.add(cur_str)
            
            # visit all children (neighbor)
            for dir in dR:
                next_pos = tuple(cur_pos + dir)
                
                # next pos in bound and not block
                if not self.check_boundary(next_pos) or self.grid[next_pos] != 0:
                    continue
                
                # next pos not in CLOSE
                if self.pos_to_str(next_pos) not in self.close_list:
                    # label correction
                    if self.g_grid[next_pos] > (self.g_grid[tuple(cur_pos)] + np.linalg.norm(dir)):
                        self.g_grid[next_pos] = self.g_grid[tuple(cur_pos)] + np.linalg.norm(dir)
                        self.parent[next_pos] = cur_pos
                        next_pos_str = self.pos_to_str(next_pos)
                        
                        # if next_pos_str in open_list:
                        #     # updade priority (f-value)
                        #     open_list[next_pos_str] = get_fValue(next_pos, goal_id, g_grid)
                        # else:
                        #     # add into openlist
                        #     open_list[next_pos_str] = get_fValue(next_pos, goal_id, g_grid)
                        
                        self.open_list[next_pos_str] = self.get_fValue(next_pos, goal_id, epsilon)
                        last_id_str = next_pos_str
        
        path = self.get_path_from_parent(start_id_str, goal_id_str)
        print(f"Number of considered node: {num_considered_node}")
        return path, num_considered_node

class MyPlanner:
  '''
  Starter code
  '''
  __slots__ = ['boundary', 'blocks']
  
  def __init__(self, boundary, blocks):
    self.boundary = boundary
    self.blocks = blocks


  def plan(self,start,goal):
    path = [start]
    numofdirs = 26
    [dX,dY,dZ] = np.meshgrid([-1,0,1],[-1,0,1],[-1,0,1])
    dR = np.vstack((dX.flatten(),dY.flatten(),dZ.flatten()))
    dR = np.delete(dR,13,axis=1)
    dR = dR / np.sqrt(np.sum(dR**2,axis=0)) / 2.0
    
    for _ in range(2000):
      mindisttogoal = 1000000
      node = None
      for k in range(numofdirs):
        next = path[-1] + dR[:,k]
        
        # Check if this direction is valid
        if( next[0] < self.boundary[0,0] or next[0] > self.boundary[0,3] or \
            next[1] < self.boundary[0,1] or next[1] > self.boundary[0,4] or \
            next[2] < self.boundary[0,2] or next[2] > self.boundary[0,5] ):
          continue
        
        valid = True
        for k in range(self.blocks.shape[0]):
          if( next[0] >= self.blocks[k,0] and next[0] <= self.blocks[k,3] and\
              next[1] >= self.blocks[k,1] and next[1] <= self.blocks[k,4] and\
              next[2] >= self.blocks[k,2] and next[2] <= self.blocks[k,5] ):
            valid = False
            break
        if not valid:
          continue
        
        # Update next node
        disttogoal = sum((next - goal)**2)
        if( disttogoal < mindisttogoal):
          mindisttogoal = disttogoal
          node = next
      
      if node is None:
        break
      
      path.append(node)
      
      # Check if done
      if sum((path[-1]-goal)**2) <= 0.1:
        break
      
    return np.array(path)
