import numpy as np
import matplotlib.pyplot as plt
from typing import Deque, List, Tuple
from nav_msgs.msg import OccupancyGrid, MapMetaData
import rospy


Grid = Tuple[int, int]

class GridMap:	
    UNKNOWN = -1	
    CUR = 0	
    OCCUPIED = 1	

    HALF_GRID_WIDTH = 21	
    GRID_WIDTH = 2 * HALF_GRID_WIDTH	

    def __init__(self, shape=(GRID_WIDTH, GRID_WIDTH)):	
        """ Map should initialized with -1"""	
        self.__grid_map = np.full(shape=shape, fill_value=GridMap.UNKNOWN, dtype=np.int8)
        self.pub_grid = rospy.Publisher("/gem/GridMap", OccupancyGrid, queue_size=1)	

    def __getitem__(self, key):	
        return self.__grid_map.__getitem__(key)	

    def __setitem__(self, key, value):	
        # if key[0] >= GridMap.HALF_GRID_WIDTH or key[1] >= GridMap.HALF_GRID_WIDTH or key[0] < -GridMap.HALF_GRID_WIDTH or key[1] < -GridMap.HALF_GRID_WIDTH:	
        #     return 	
        return self.__grid_map.__setitem__(key, value)
        

    def __delitem__(self, key):	
        return self.__grid_map.__delitem__(key)	

    def __repr__(self):	
        return repr(self.__grid_map.tolist())	

    def publish(self):
        grid_map = OccupancyGrid()
        mapInfo = MapMetaData()
        # NOTE For the map info, we might need to set the resolution and origin later
        mapInfo.width = GridMap.GRID_WIDTH
        mapInfo.height = GridMap.GRID_WIDTH
        # mapInfo.origin = XXX
        # mapInfo.resolution = XXX
        grid_map.info = mapInfo
        # Need to flatten the grid map array to fit into the message
        grid_map.data = self.__grid_map.flatten().tolist()
        self.pub_grid.publish(grid_map)

    def show(self):	
        # viz_map = np.full(shape=self.__grid_map.shape, fill_value=-1, dtype=np.int8)	
        # for x in range(GridMap.GRID_WIDTH):	
        #     for y in range(GridMap.GRID_WIDTH):	
        #         _x, _y = x - GridMap.HALF_GRID_WIDTH, y - GridMap.HALF_GRID_WIDTH	
        #         viz_map[y][x] = self.__grid_map[_x][_y]  # XXX the image has to be transposed	

        plt.imshow(self.__grid_map) #, origin='lower')	
        plt.pause(0.005)	

    # @staticmethod	
    # def quantize(x: float, y: float) -> Grid:	
    #     return tuple(np.floor([x, y]).astype(int))	

    # @staticmethod	
    # def _nbr_grids(grid: Grid) -> List[Grid]:	
    #     x, y = grid	
    #     return [(x-1, y), (x+1, y), (x, y-1), (x, y+1)]	

    # def get_frontier_grids(self, curr_grid: Grid) -> List[Grid]:	
    #     """ Get the frontier empty grids that are next to unknown grids.	
    #     These frontier nodes must be reachable from current grid thru a connected empty grids.	
    #     :param curr_grid: Current grid	
    #     :return: Frontier grids	
    #     """	
    #     assert self[curr_grid] != GridMap.OCCUPIED	

    #     frontier = []	
    #     visited = {curr_grid}	
    #     q = Deque()	
    #     q.append(curr_grid)	
    #     # Simple BFS to find the list of frontier grids. TODO optimization?	
    #     while q:	
    #         s = q.popleft()	

    #         nbr_grids = self._nbr_grids(s)	
    #         # If any of the neighbors is unknown, this grid is a frontier grid	
    #         if any([self[n] == GridMap.UNKNOWN for n in nbr_grids]):	
    #             frontier.append(s)	
    #         # Add unvisited empty neighbors to queue	
    #         q.extend([n for n in nbr_grids if n not in visited and self[n] == GridMap.EMPTY])	

    #         visited |= set(nbr_grids)  # Add neighbors to visited	

    #     return frontier