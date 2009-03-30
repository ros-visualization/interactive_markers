import roslib, rospy, sys, math, utils
roslib.update_path('robot_srvs')
from robot_srvs.srv import StaticMap



class ConfigSpace:
    """Abstract configuration space class.  See subclasses below."""
    

    _FREE = 0
    _OCCUPIED = 100
    _UNKNOWN = -1


    def __init__(self, (mapWidth, mapHeight), (robotWidth, robotHeight)):
        """
        An occMap2D message has the following attributes:
        width - num cols in map
        height - num rows in map
        data - map cell data row-major order, where
           -1  = unknown
           1   = free space
           100 = occupied space
        The mapWidth and mapHeight parameters are the dimensions
        of the stage map we are using, which allows us to convert from
        stage coords to config space coords.
        The robotWidth and robotHeight parameters are used to determine
        whether a point lies in free space (i.e., the robot isn't intersecting
        an obstacle).
        """

        # get map from map_server
        rospy.wait_for_service( 'static_map' )
        try:
            occMap2D = rospy.ServiceProxy( 'static_map', StaticMap )().map
        except rospy.ServiceException, e:
            print 'Service call failed: %s' % e
            sys.exit()

        # convert from ascii codes to ints
        self._data = []
        for i in range(len(occMap2D.data)):
            x = occMap2D.data[i]
            if x == '\0':
                self._data.append(ConfigSpace._FREE)
            elif x == 'd':
                self._data.append(ConfigSpace._OCCUPIED)
            else:
                self._data.append(ConfigSpace._UNKNOWN)

        self._imageDims = (occMap2D.width, occMap2D.height)
        self._stageDims = (mapWidth, mapHeight)
        self._robotDims = (robotWidth, robotHeight)


    def isFree(self, (x, y)):
        """
        Is (x,y) in free space (stage coords)?
        This takes into account the size and rotation of the robot.
        Any points within half of the robot's diagonal length must be free.
        """
        radius = max([math.sqrt(2) * self._robotDims[i]/2 for i in range(2)])
        bottomLeft = self._stageToImage([(x, y)[i] - radius for i in range(2)])
        upperRight = self._stageToImage([(x, y)[i] + radius for i in range(2)])
        for row in range(bottomLeft[0], upperRight[0]+1):
            for col in range(bottomLeft[1], upperRight[1]+1):
                if utils.dist(self._imageToStage((row, col)), (x, y)) <= radius \
                        and not self._isFreeCell((row, col)):
                    return False
        return True


    def _isFreeCell(self, (row, col)):
        """True if the cell at (row, col) (in image coords) is not in an obstacle"""
        offset = self._getRowMajorOffset(row, col)
        return self._data[offset] == ConfigSpace._FREE


    def _getRowMajorOffset(self, row, col):
        return row * self._imageDims[0] + col


    def _stageToImage(self, (x, y)):
        """Convert (x, y) stage coords into (row, col) OccMap2D image coords."""
        col, row = utils.scaleCoords((x,y), self._stageDims, self._imageDims, False)
        return (int(row), int(col))


    def _imageToStage(self, (row, col)):
        """Convert (row, col) OccMap2D image coords into (x, y) stage coords."""
        return utils.scaleCoords((col,row), self._imageDims, self._stageDims, False)

    
    def getNeighbors(self, (x, y)):
        """
        Get all free points (in stage coords) connected to the given point 
        (also in stage coords).  This will be implemented by subclasses.
        """
        raise NotImplementedError
        
                

class GridConfigSpace(ConfigSpace):
    """A grid-discretized 2D config space for a given map."""


    # controls the granularity of the grid, 
    # i.e. the horizontal/vertical distance between adjacent points 
    _DEFAULT_GRID_SPACE = .25   # in meters

    # The number of grid spaces to search for neighbors in each direction.
    # For example if the range is 10, all nodes within 10 grid spaces to 
    # the right, left, top, or bottom of a node will be considered its neighbors
    _DEFAULT_NEIGHBOR_RANGE = 3

    
    def __init__(self, (mapWidth, mapHeight),
                 (robotWidth, robotHeight),
                 gridSpace = _DEFAULT_GRID_SPACE,
                 neighborRange = _DEFAULT_NEIGHBOR_RANGE):
        ConfigSpace.__init__(self, (mapWidth, mapHeight), (robotWidth, robotHeight))
        self._gridSpace = gridSpace
        self._neighborCoords = GridConfigSpace._getRelativeNeighborCoords(gridSpace, neighborRange)
        

    @staticmethod
    def _getRelativeNeighborCoords(gridSpace, neighborRange):
        """
        Get the relative positions of all neighbor grid points, in the form (dx, dy).
        For example, returning the list [(0, 1), (1, 0)] implies that a point at (45, 50)
        has neighbors at (45, 51) and (46, 50).
        """

        # list of coords of neighbors, relative to point
        neighborCoords = []

        # set of slopes we have already seen - eliminated redundant neighbors
        slopes = {}

        # relative vertical and horizantal distances of neighbor points
        dists = [float(gridSpace) * i for i in range(neighborRange)]

        for dx in dists:
            for dy in dists:

                # a point can't be its own neighbor
                if not dx and not dy:
                    continue
                
                # add coords if slope is not redundant
                if dx:
                    slope = dy/dx
                else:
                    slope = utils.INFINITY
                if not slopes.has_key(slope):
                    slopes[slope] = 1
                    neighborCoords.append((dx, dy))
        
        # add negative coords
        for pt in neighborCoords[:]:
            if pt[0]:
                neighborCoords.append((-pt[0], pt[1]))
            if pt[1]:
                neighborCoords.append((pt[0], -pt[1]))
            if pt[0] and pt[1]:
                neighborCoords.append((-pt[0], -pt[1]))
                
        return neighborCoords                               


    def getNearestGridPos(self, (x, y)):
        """Round (x,y) to the nearest point on the grid (in stage coordinates)."""
        gridPos = []
        for i in range(2):
            t = (x,y)[i]
            remainder = t % self._gridSpace
            base = t - remainder
            if remainder >= self._gridSpace/2.:
                gridPos.append(base + self._gridSpace)
            else:
                gridPos.append(base)
        return tuple(gridPos)


    def getNeighbors(self, (x, y)):
        """ Find all free points connected to this point (in stage coords). """
        x, y = self.getNearestGridPos((x,y))
        neighbors = []
        for dx, dy in self._neighborCoords:
            neighbor = (x + dx, y + dy)
            if self.isFree(neighbor):
                neighbors.append(neighbor)
        return neighbors


    def getGridSpace(self):
        return self._gridSpace




