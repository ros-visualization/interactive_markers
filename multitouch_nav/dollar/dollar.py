#
#  The $1 Gesture Recognizer
#
#      Jacob O. Wobbrock
#      The Information School
#      University of Washington
#      Mary Gates Hall, Box 352840
#      Seattle, WA 98195-2840
#      wobbrock@u.washington.edu
#
#      Andrew D. Wilson
#      Microsoft Research
#      One Microsoft Way
#      Redmond, WA 98052
#      awilson@microsoft.com
#
#      Yang Li
#      Department of Computer Science and Engineering
#      University of Washington
#      The Allen Center, Box 352350
#      Seattle, WA 98195-2840
#      yangli@cs.washington.edu
#
# Python port:
# Charlie Von Metzradt, November 2008
# http://sleepygeek.org
#
# Usage example:
#
# from dollar import Recognizer
#
# r = Recognizer()
# r.addTemplate('square', [(1, 10), (3, 8) ... ])
# r.addTemplate('circle', [(4, 7), (5, 13) ... ])
#
# (name, score) = r.recognize([(5, 6), (7, 12), ... ])
#

import math

# Contants. Tweak at your peril. :)

numPoints      = 64
squareSize     = 250.0
halfDiagonal   = 0.5 * math.sqrt(250.0 * 250.0 + 250.0 * 250.0)
angleRange     = 45.0
anglePrecision = 2.0
phi            = 0.5 * (-1.0 + math.sqrt(5.0)) # Golden Ratio

INFINITY = 1000000000.
 
class Recognizer:
    """The $1 gesture recognizer. See http://sleepygeek.org/projects.dollar for more, or http://depts.washington.edu/aimgroup/proj/dollar/ for the original implementation and paper."""
    templates = []

    def recognize(self, points):
        """Determine which gesture template most closely matches the gesture represented by the input points. 'points' is a list of tuples, eg: [(1, 10), (3, 8) ...]. Returns a tuple of the form (name, score) where name is the matching template, and score is a float [0..1] representing the match certainty."""

        points = [Point(point[0], point[1]) for point in points]
        points = _resample(points, numPoints);
        points = _rotateToZero(points);
        points = _scaleToSquare(points, squareSize);
        points = _translateToOrigin(points);
        
        bestDistance = INFINITY
        bestTemplate = None
        
        for template in self.templates:
            distance = _distanceAtBestAngle(points, template, -angleRange, +angleRange, anglePrecision)
            if distance <= bestDistance:
                bestDistance = distance
                bestTemplate = template

        score = 1.0 - (bestDistance / halfDiagonal)
        return (bestTemplate.name, score)

    def addTemplate(self, name, points):
        """Add a new template, and assign it a name. Multiple templates can be given the same name, for more accurate matching. Returns an integer representing the number of templates matching this name."""
        self.templates.append(Template(name, points))
        
        # Return the number of templates with this name.
        return len([t for t in self.templates if t.name == name])        

    def deleteTemplates(self, name):
        """Remove all templates matching a given name. Returns an integer representing the new number of templates."""

        self.templates = [t for t in self.templates if t.name != name]
        return len(self.templates);

class Point:
    """Simple representation of a point."""
    def __init__(self, x, y):
        self.x = x
        self.y = y

class Rectangle:
    """Simple representation of a rectangle."""
    def __init__(self, x, y, width, height):
        self.x = x
        self.y = y
        self.width = width
        self.height = height

class Template:
    """A gesture template. Used internally by Recognizer."""
    def __init__(self, name, points):
        """'name' is a label identifying this gesture, and 'points' is a list of tuple co-ordinates representing the gesture positions. Example: [(1, 10), (3, 8) ...]"""
        self.name = name
        self.points = [Point(point[0], point[1]) for point in points]
        self.points = _resample(self.points, numPoints);
        self.points = _rotateToZero(self.points);
        self.points = _scaleToSquare(self.points, squareSize);
        self.points = _translateToOrigin(self.points);


def _resample(points, n):
    """Resample a set of points to a roughly equivalent, evenly-spaced set of points."""
    I = _pathLength(points) / (n - 1) # interval length
    D = 0.0
    newpoints = [points[0]]
    i = 1
    while i < len(points) - 1:
        d = _distance(points[i - 1], points[i])
        if (D + d) >= I:
            qx = points[i - 1].x + ((I - D) / d) * (points[i].x - points[i - 1].x)
            qy = points[i - 1].y + ((I - D) / d) * (points[i].y - points[i - 1].y)
            q = Point(qx, qy)
            newpoints.append(q)
            # Insert 'q' at position i in points s.t. 'q' will be the next i
            points.insert(i, q)
            D = 0.0
        else:
            D += d
        i += 1

    # Sometimes we fall a rounding-error short of adding the last point, so add it if so.
    if len(newpoints) == n - 1:
        newpoints.append(points[-1])
    return newpoints;

def _rotateToZero(points):
    """Rotate a set of points such that the angle between the first point and the centre point is 0."""
    c = _centroid(points)
    theta = math.atan2(c.y - points[0].y, c.x - points[0].x)
    return _rotateBy(points, -theta)

def _rotateBy(points, theta):
    """Rotate a set of points by a given angle."""
    c = _centroid(points);
    cos = math.cos(theta);
    sin = math.sin(theta);
    
    newpoints = [];
    for point in points:
        qx = (point.x - c.x) * cos - (point.y - c.y) * sin + c.x
        qy = (point.x - c.x) * sin + (point.y - c.y) * cos + c.y
        newpoints.append(Point(qx, qy))
    return newpoints

def _scaleToSquare(points, size):
    """Scale a scale of points to fit a given bounding box."""
    B = _boundingBox(points)
    newpoints = []
    for point in points:
        qx = point.x * (size / B.width)
        qy = point.y * (size / B.height)
        newpoints.append(Point(qx, qy))
    return newpoints

def _translateToOrigin(points):
    """Translate a set of points, placing the centre point at the origin."""
    c = _centroid(points)
    newpoints = []
    for point in points:
        qx = point.x - c.x
        qy = point.y - c.y
        newpoints.append(Point(qx, qy))
    return newpoints;

def _distanceAtBestAngle(points, T, a, b, threshold):
    """Search for the best match between a set of points and a template, using a set of tolerances. Returns a float representing this minimum distance."""
    x1 = phi * a + (1.0 - phi) * b
    f1 = _distanceAtAngle(points, T, x1)
    x2 = (1.0 - phi) * a + phi * b
    f2 = _distanceAtAngle(points, T, x2)

    while abs(b - a) > threshold:
        if f1 < f2:
            b = x2
            x2 = x1
            f2 = f1
            x1 = phi * a + (1.0 - phi) * b
            f1 = _distanceAtAngle(points, T, x1)
        else:
            a = x1
            x1 = x2
            f1 = f2
            x2 = (1.0 - phi) * a + phi * b
            f2 = _distanceAtAngle(points, T, x2)
    return min(f1, f2)

def _distanceAtAngle(points, T, theta):
    """Returns the distance by which a set of points differs from a template when rotated by theta."""
    newpoints = _rotateBy(points, theta)
    return _pathDistance(newpoints, T.points)

def _centroid(points):
    """Returns the centre of a given set of points."""
    x = 0.0
    y = 0.0
    for point in points:
        x += point.x
        y += point.y
    x /= len(points)
    y /= len(points)
    return Point(x, y)

def _boundingBox(points):
    """Returns a Rectangle representing the bounding box that contains the given set of points."""
    minX = INFINITY
    maxX = -INFINITY
    minY = INFINITY
    maxY = -INFINITY

    for point in points:
        if point.x < minX:
            minX = point.x
        if point.x > maxX:
            maxX = point.x
        if point.y < minY:
            minY = point.y
        if point.y > maxY:
            maxY = point.y

    return Rectangle(minX, minY, maxX - minX, maxY - minY)

def _pathDistance(pts1, pts2):
    """'Distance' between two paths."""
    d = 0.0;
    for index in range(len(pts1)): # assumes pts1.length == pts2.length
        d += _distance(pts1[index], pts2[index])
    return d / len(pts1)

def _pathLength(points):
    """Sum of distance between each point, or, length of the path represented by a set of points."""
    d = 0.0;
    for index in range(1, len(points)):
        d += _distance(points[index - 1], points[index])
    return d

def _distance(p1, p2):
    """Distance between two points."""
    dx = p2.x - p1.x
    dy = p2.y - p1.y
    return math.sqrt(dx * dx + dy * dy)