import heapq, utils, sys, math



class AStarPathPlanner:
    

    def __init__(self, configSpace):
        self._configSpace = configSpace 


    @staticmethod
    def _compNodes(node1, node2):
        """Node comparison function for priority queue."""
        f = lambda node: node.getCost() #+ node.getHeuristicDist()
        return f(node1) - f(node2)


    def _goalTest(self, pt, goal):
        """
        Whether a node at the given position should be considered at the goal.
        True if the goal is within one grid square of the given pos.
        """
        return utils.dist(pt, goal) <= \
            math.sqrt(2) * self._configSpace.getGridSpace()


    def getRobotPath(self, fingerPath):

        # set start as first valid point on finger path 
        # and set goal as last valid point
        fingerPath = fingerPath[:]  # make copy
        start = fingerPath[0]
        goal = fingerPath[-1]
        while not self._configSpace.isFree(start):
            fingerPath.pop(0)
            if not fingerPath: return []
            start = fingerPath[0]       
        while not self._configSpace.isFree(goal):
            fingerPath.pop()
            if not fingerPath: return []
            goal = fingerPath[-1]

        # initialize queue
        costFunc = _FingerPathCostFunc(fingerPath, self._configSpace)
        root = _QueueNode([start], 0, goal, costFunc.getCost, 
                          costFunc.getHeuristic, self._goalTest, 
                          AStarPathPlanner._compNodes)
        queue = [root]

        # set of positions of nodes we have already expanded
        visitedStates = {start : 1}

        # map from positions to unexpanded nodes on the fringe
        fringe = {start : root}

        # begin search
        while queue:

            # pop best node and do goal test
            node = heapq.heappop(queue)
            if node.isGoalNode():
                return node.getPath()

            # remove from fringe and mark as expanded
            del fringe[node.getPos()]
            visitedStates[node.getPos()] = 1

            # expand node
            for child in node.expand(self._configSpace):

                # don't add child if we've already expanded from its pos
                if not visitedStates.has_key(child.getPos()):

                    # don't add child if there is a better node on the fringe
                    if fringe.has_key(child.getPos()):
                        other = fringe[child.getPos()]
                        if AStarPathPlanner._compNodes(child, other) >= 0:
                            continue
                        else:
                            # remove old node from queue if new node is better
                            queue.remove(other)
                            heapq.heapify(queue)

                    # add child to fringe
                    fringe[child.getPos()] = child
                    heapq.heappush(queue, child)

        assert False



class _FingerPathCostFunc:
    """
    A cost function that takes into account the robot's deviation
    from the path specified by the operator's finger.  The cost of 
    moving from point p1 to p2 depends both on the distance from 
    p1 to p2, as well as how much closer p1 is to the finger path
    than p2.  The relative weights of these two factors are controlled
    by ALPHA.
    """    
 

    def __init__(self, fingerPath, configSpace):
        self._fingerPath = [ pt[:] for pt in fingerPath ]
        self._configSpace = configSpace
        

    def getCost(self, p1, p2):
        """
        For each "step" between p1 and p2, add the distance from p2 to the 
        finger path, where a step is the size of the grid spacing.
        """
        cost = 0
        angle = math.atan2(p1[1] - p2[1], p1[0] - p2[0])
        numSteps = math.ceil(utils.dist(p1, p2) / self._configSpace.getGridSpace())
        for i in range(numSteps - 1):
            x = p1[0] + i * self._configSpace.getGridSpace() * math.cos(angle)
            y = p1[1] + i * self._configSpace.getGridSpace() * math.sin(angle)
            cost += self._getDistFromFingerPath((x, y))
        return cost + self._getDistFromFingerPath(p2)


    def getHeuristic(self, pt, goal):
        return min([ self._getDistFromFingerPath(n) \
                         for n in self._configSpace.getNeighbors(pt) ])


    def _getDistFromFingerPath(self, p):
        """
        Find the average distance from p to each of the points in 
        the finger path.
        """
        return min([ utils.dist(p, fp) for fp in self._fingerPath ])



class _QueueNode:

    
    _GOAL_TOLERANCE = .1


    def __init__(self, path, cost, goal, costFunc, heurFunc, goalTest, compareFunc):
        """
        path - list of config space points that led to this state
        cost - cost of the path we've traversed so far
        goal - goal in config space coords
        costFunc - returns cost of traveling from one point to another
        heurFunc - returns heuristic cost based on current pos and the goal pos
        goalTest - returns True is this node's position is close enough to the goal
        compareFunc - compares 2 QueueNodes; compareFunc(self, other)
                      returns 0 if equal, 1 if self > other, -1 if self < other
                      (used for priorityqueue)
        """
        self._path = path[:]
        self._cost = cost
        self._goal = goal
        self._costFunc = costFunc
        self._heurFunc = heurFunc
        self._goalTest = goalTest
        self._compareFunc = compareFunc


    def expand(self, configSpace):
        childNodes = []
        neighborPts = configSpace.getNeighbors(self.getPos())
        for pt in neighborPts:
            cost = self._cost + self._costFunc(self.getPos(), pt)
            self._path.append(pt)
            childNodes.append(_QueueNode(self._path, cost, self._goal, 
                                         self._costFunc, self._heurFunc, 
                                         self._goalTest, self._compareFunc))
            self._path.pop()
        return childNodes


    def getCost(self):
        return self._cost
    
    def getPath(self):
        return self._path[:]

    def getPos(self):
        return self._path[-1]

    def getHeuristicDist(self):
        return self._heurFunc(self.getPos(), self._goal)

    def isGoalNode(self):
        return self._goalTest(self.getPos(), self._goal)
    

    # override comparison operators for priority queue

    def __lt__(self, other):
        return self._compareFunc(self, other) < 0

    def __gt__(self, other):
        return self._compareFunc(self, other) > 0

    def __le__(self, other):
        return self._compareFunc(self, other) <= 0

    def __ge__(self, other):
        return self._compareFunc(self, other) >= 0
