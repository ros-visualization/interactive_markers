import rospy, math, time
from std_msgs.msg import *
from robot_msgs.msg import *
from utils import *



class Robot:
    


    # The minimum distance between a new goal and the last goal in the robot's path.
    # This prevents adding redundant goals to the path while the user is holding
    # his or her finger down in the same spot for a long time.
    MIN_DIST_BETWEEN_GOALS = .05

    # The amount of time we allow the robot to reach its goal.
    GOAL_TIMEOUT = 15.

    # The amount of time the robot can stand still before we issue the next goal.
    # This is used for removing goals from a path that go through obstacles.
    PATH_PLAN_TIMEOUT = 1.

    # The distance/rotation the robot must travel before the path-plan timer is reset.
    MIN_DIST_FOR_MOVEMENT = .01
    MIN_ROT_FOR_MOVEMENT = math.pi/16.



    def __init__(self, id=0):

        self._id = id          # robot id used for publishing/receiving messages
        self._pos = [0,0]      # position in stage map coordinates
        self._theta = 0        # rotation in radians (0 = east, increasing counter-clockwise)
        self._path = []        # list of (x, y) points
        self._go = False       # whether robot is moving
        self._selected = False # whether robot is selected

        # movement tracking variables
        self._lastMoveTime = time.time()  # amount of time since our last movement
        self._lastMovePos = self._pos[:]  # position where we last stoppped moving
        self._lastMoveTheta = self._theta # rotation where we last stopped moving

        # Eventually, we will only publish goals when we don't care about
        # the path.  When we care about the path, we will compute the PoseDot
        # velocities for moving from point to point and publish those directly
        # to rosstage.  Currently, to follow a specific path, we just publish
        # points along that path and let wavefront_player compute the path from
        # point to point.  This is inefficient and causes the robot to constantly
        # stop and go.
        self._goalPub = rospy.Publisher("robot_" + str(id) + "/goal", Planner2DGoal)

        # Listen for the robot's state when using an outside path planner.
        # Note that this only applies when we have published a goal and don't
        # care about the path taken.
        rospy.Subscriber("robot_" + str(id) + "/state", Planner2DState, Robot._stateMsgCallback, self)


    @staticmethod
    def _stateMsgCallback(state, self):
        """static callback for receiving ros robot state message"""
        self._updateState(state)    



    def _updateState(self, state):
        """handle ros robot state message, called by __callback__"""

        self._pos = (state.pos.x, state.pos.y)
        self._theta = state.pos.th

        # check if we are active and actually moving
        if dist(self._pos, self._lastMovePos) >= Robot.MIN_DIST_FOR_MOVEMENT\
                or abs(self._theta - self._lastMoveTheta) >= Robot.MIN_ROT_FOR_MOVEMENT:
            self._lastMovePos = self._pos[:]
            self._lastMoveTheta = self._theta
            self._lastMoveTime = time.time()

        # if not moving, remove the current goal from the path and send the next one
        if self._go and self._path:
            if time.time() - self._lastMoveTime > Robot.PATH_PLAN_TIMEOUT:
                self._path.pop(0)
                if self._path:
                    self._sendNextGoalInPath()
                else:
                    self._go = False



    def _sendNextGoalInPath(self):
        """Send the robot to the next position in the path."""
        theta = angle(self._pos, self._path[0])
        self._sendGoal( self._path[0], theta )


    def getID(self):
        return self._id


    def getPos(self):
        return self._pos[:]


    def getPath(self):
        return [pt[:] for pt in self._path]


    def getTheta(self):
        return self._theta


    def isMoving(self):
        return self._go


    def toggleSelected(self):
        self._selected = not self._selected


    def setSelected(self, selected):
        self._selected = selected


    def isSelected(self):
        return self._selected


    def getDistFromGoal(self):
        assert self._path
        return dist( self._pos, self._path[0] )


    def addPointToPath(self, point):
        if not self._path:
            assert len(point) == 2
            self._path.append(point)
            return
        lastPoint = self._path[len(self._path)-1]
        if dist(point, lastPoint) > Robot.MIN_DIST_BETWEEN_GOALS:
            self._path.append(point)


    def setPath(self, path):
        self._path = path


    def turnAround( self ):
        self.rotateBy( math.pi )


    def rotateBy( self, dTheta ):
        """
        Stop and rotate by the given angle (in radians),
        relative to the robot's current orientation.
        """
        self._rotateTo( self._theta + dTheta )


    def rotateTo( self, theta ):
        """
        Stop and rotate by the given angle (in radians),
        relative to the positive x-axis (east).
        """
        self.stop()
        self._sendGoal( self._pos, theta )


    def go(self):
        """Start moving to the next point in the robot's path"""
        assert self._path
        self._go = True
        self._sendNextGoalInPath()


    def _sendGoal(self, pos, theta):
        goal = Planner2DGoal()
        goal.goal.x = pos[0]
        goal.goal.y = pos[1]
        goal.goal.th = theta
        goal.enable = 1
        goal.timeout = Robot.GOAL_TIMEOUT
        self._goalPub.publish(goal)
        self._lastMoveTime = time.time()
        self._lastMovePos = self._pos[:]


    def stop(self):
        """Stop moving."""
        self._sendGoal( self._pos, self._theta )
        self._go = False


    def stopAndClearPath(self):
        self.stop()
        self._path = []        
