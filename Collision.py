class CollisionChecker:
    def __init__(self, path, ax):
        self.path = path
        self.ax = ax

    def check_intersect(self, obj):
        min_bound = obj[:3]
        max_bound = obj[3:6]

        for i in range(1, len(self.path)):
            start = self.path[i-1]
            dir = self.path[i] - self.path[i-1]

            # inside the block
            if (start[0] > min_bound[0]) and (start[0] < max_bound[0]) and \
                (start[1] > min_bound[1]) and (start[1] < max_bound[1]) and \
                (start[2] > min_bound[2]) and (start[2] < max_bound[2]):
                print(f"inside box: {obj}")
                return True

            for axis in range(3):
                t = 0.0
                if abs(dir[axis]) > 0:
                    if dir[axis] > 0:
                        t = (min_bound[axis] - start[axis]) / dir[axis]
                    else:
                        t = (max_bound[axis] - start[axis]) / dir[axis]
                
                # ray intersects with the plane
                if t > 0 and t <= 1:
                    pt = start + t * dir
                    if (min_bound[(axis + 1) % 3] < pt[(axis + 1) % 3]) and \
                        (max_bound[(axis + 1) % 3] > pt[(axis + 1) % 3]) and \
                        (min_bound[(axis + 2) % 3] < pt[(axis + 2) % 3]) and \
                        (max_bound[(axis + 2) % 3] > pt[(axis + 2) % 3]):
                        
                        print(f"intersect with box: {obj} at pt: {pt}, t={t}")
                        self.ax.plot(pt[0], pt[1], pt[2],'bo',markersize=7,markeredgecolor='k')
                        return True
        
        return False
