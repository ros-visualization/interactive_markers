import roslib, rospy, math, time, threading
roslib.update_path('laser_scan')
from laser_scan.msg import LaserScan
from utils import *

class RobotCoordinator:
    """Tracks the robots and makes sure they don't run into each other."""


    # How often we poll the robots to see if they are near each other.
    POLL_RATE = 1.

    # The minimum distance between 2 robots. If two robots are closer than
    # this, the commander tells one to get out of the way.
    MIN_DIST_BETWEEN_ROBOTS = 1.75

    # Time to wait for one robot to get out of the other's way
    WAIT_TIME = 5

    # The minimum available space for sending a robot to a new location.
    # For example, if we want to send the robot from (5,5) to (8,8), there
    # must be at least MIN_SCAN_RANGE meters open in the 45 degree 
    # direction.
    MIN_SCAN_RANGE = 3.

    # The amount we increment the robot's rotation by when searching for a 
    # new direction to move.
    SEARCH_ROTATION_INC = math.pi


    def __init__(self):
        self._robots = {}      # hashset of _RobotTrackers (see class below)
        self._robotPairs = {}  # hashset of pairs of robots
        class PollTimer ( threading.Thread ):
            """A timer for polling robots and checking for collisions."""
            def __init__(self, robotCoordinator):
                threading.Thread.__init__(self)
                self._robotCoordinator = robotCoordinator
            def run(self):
                while (True):
                    self._robotCoordinator._pollRobots()
                    time.sleep(RobotCoordinator.POLL_RATE)
        PollTimer(self).start()


    def registerRobot(self, r):
        r = _RobotTracker(r)
        for other in self._robots:
            self._robotPairs[(other, r)] = 1
        self._robots[r] = 1

   
    def _pollRobots(self):

        # iterate through all pairs of robots and check if they are too close
        for r1,r2 in self._robotPairs.keys():

            # if we are already coordinating both of these robots, continue
            if r1.isMovingAwayFrom(r2) or r2.isMovingAwayFrom(r1):
                continue

            # if they are not moving, they will not collide
            if not r1.isMoving() and not r2.isMoving():
                continue

            d = dist( r1.getPos(), r2.getPos() )
            if d < RobotCoordinator.MIN_DIST_BETWEEN_ROBOTS:
                
                # Pick which robot to move out of the other's way
                # If one does not have a goal, pick it.
                if not r1.isMoving():
                    r1.moveAwayFrom(r2)
                elif not r2.isMoving():
                    r2.moveAwayFrom(r1)
                
                # Otherwise, pick the one which is farther from its goal.
                else:
                    d1 = r1.getDistFromGoal()
                    d2 = r2.getDistFromGoal()
                    print "d1", d1, "d2", d2
                    if d1 > d2:
                        r1.moveAwayFrom(r2)
                    else:
                        r2.moveAwayFrom(r1)
        


class _RobotTracker:
    """
    Manages a single robot, moving it out of other robots' ways and keeping track
    of which robots it is currently moving away from.
    """

    def __init__(self, robot):

        self._robot = robot
        self._movingAwayFrom = {}  # set of robots this robot is currently moving away from
        self._scanData = None      # most recent laser scan data for this robot

        # register for laser scan message
        msgName = "robot_" + str(r.getID()) + "/base_scan"
        callback = lambda scan, self: self._scanData = scan
        rospy.Subscriber(msgName, LaserScan, callback, self)


    def moveAwayFrom(self, other):
        """
        Move the this robot out of the other's way.
        This is done on a separate thread so that we can keep checking other
        pairs of robots while these two are moving.
        """
        self._moveAwayFrom[other] = 1
        # change run to start to make this multi-threaded
        _DivertTimer(self, other).run()

    
    def isMovingAwayFrom(self, other):
        return self._movingAwayFrom.has_key(other)


    # delegate methods from robot:

    def isMoving(self):
        return self._robot.isMoving()

    def getDistFromGoal(self):
        return self._robot.getDistFromGoal()

    def stop(self):
        self._robot.stop()

    def stopAndClearPath(self):
        self._robot.stopAndClearPath()

    def go(self):
        self._robot.go()

    def addPointToPath(pt):
        self._robot.addPointToPath(pt)

    def getPath(self):
        return self._robot.getPath()



class _DivertTimer (threading.Thread):
    """Temporarily moves one robot out of the other's way."""


    def __init__(self, robotToMove, robotWaiting):
        threading.Thread.__init__(self)
        self._robotToMove = robotToMove
        self._robotWaiting = robotWaiting

           
    def run(self):

        self._robotToMove.stop()
        self._robotWaiting.stop()

        p1 = self._robotWaiting.getPos()
        p2 = self._robotToMove.getPos()
        dx, dy = [p2[i] - p1[i] for i in range(2)]

        angle = math.atan2(dy, dx)
        if angle < 0: angle += 2*math.pi
        goal = ( p2[0] + (2) * math.cos(angle),
                 p2[1] + (2) * math.sin(angle))

        oldPath = self._robotToMove.getPath()
        self._robotToMove.stopAndClearPath()
        self._robotToMove.addPointToPath(goal)
        self._robotToMove.go()

        time.sleep( RobotCoordinator.WAIT_TIME )

        self._robotToMove.stopAndClearPath()
        if oldPath:
            for pt in oldPath:
                self._robotToMove.addPointToPath(pt)
            self._robotToMove.go()
        if self._robotWaiting.getPath():
            self._robotWaiting.go()
