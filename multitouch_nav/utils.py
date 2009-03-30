import math, threading, time


INFINITY = 10e99999


# colors
BLACK = (0,0,0)
GREEN = (0,255,0)
RED = (255,0,0)
BLUE = (0,0,255)
YELLOW = (255,255,0)
ORANGE = (255, 165, 0)
PURPLE = (160, 32, 240)
GOLD = (255, 215, 0)
OLIVE_GREEEN = (79, 79, 47)
BROWN = (165, 42, 42)
SKY_BLUE = (50, 153, 204)
TURQUOISE = (64, 244, 208)
GREY = (84, 84, 84)
STEEL_BLUE = (35, 107, 142)
WHITE = (255, 255, 255)


def dist(pt1, pt2):
    """Compute the Euclidean distance between two points."""
    assert len(pt1) == len(pt2)
    dist = 0
    for i in range(len(pt1)):
        diff = pt1[i] - pt2[i]
        dist += diff * diff
    return dist



def angle(pt1, pt2):
    """Compute the angle between two points."""
    assert len(pt1) == len(pt2) and len(pt1) == 2
    dx = pt1[0] - pt2[0]
    dy = pt1[1] - pt2[1]
    return math.atan2(dy, dx) + math.pi



def ptLineDist(pt1, pt2, pt3):
    """Compute the distance from pt1 to the line connecting p2 and p3"""
    dx1, dy1 = [ pt2[i] - p1[i] for i in range(2) ]
    dx2, dy2 = [ pt3[i] - p2[i] for i in range(2) ]
    return abs(dx2*dy1 - dx1*dy2)/math.sqrt(dx2*dx2 + dy2*dy2)



def manhatDist(pt1, pt2):
    return sum([ abs(pt1[i]-pt2[i]) for i in range(2) ])



def scaleCoords(pos, oldDims, newDims, flipY=True):
    """
    Translate pygame, touchKit, or map coordinates into one of the other two.
    Note that we must sometimes flip the y-coordinate, since pygame and touchKit
    put (0,0) in the upper-left, but the map puts it in the lower-left.
    """
    assert len(pos) == len(oldDims) and len(pos) == len(newDims)
    x = pos[0] * newDims[0] / float(oldDims[0])
    y = pos[1] * newDims[1] / float(oldDims[1])
    if flipY: y = newDims[1] - y
    return (x, y)



class Timer ( threading.Thread ):

    def __init__(self, func, rate, *args, **kwargs):
        threading.Thread.__init__(self)
        self._func = func
        self._rate = rate
        self._args = args
        self._kwargs = kwargs

    def run(self):
        while (True):
            self._func(*self._args, **self._kwargs)
            time.sleep(self._rate)
