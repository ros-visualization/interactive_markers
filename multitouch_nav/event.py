import pygame
from pygame.locals import *

import dollar

from table import Table
from Finger import Finger
import utils



class EventQueue:
    """Keeps a queue of pygame (mouse and keyboard) and touch events."""


    # dollar recognizer labels for selection gesture
    _SELECT_CIRCLE = 'selectCircle'
    _SELECT_RECT = 'selectRect'

    # minimum dollar recognizer score for classifying a select gesture
    _MIN_RECOGNIZER_SCORE = .75

    # The id for a simulated finger (using the mouse).
    # self._fingerPaths[_MOUSE_ID] will be the path dragged out by the mouse.
    _MOUSE_ID = -1


    def __init__(self, (screenWidth, screenHeight)):

        # list of keyboard, mouse, and touch events
        self._events = []

        # dollar gesture recognizer
        self._recognizer = dollar.dollar.Recognizer()
        self._recognizer.addTemplate(EventQueue._SELECT_RECT, dollar.examples.squarePoints)
        self._recognizer.addTemplate(EventQueue._SELECT_CIRCLE, dollar.examples.circlePoints)

        # table for receiving touch events
        self._table = Table(self.handleTouchEvent)

        # dimensions to which we need to convert touch event coordinates
        self._screenDims = (screenWidth, screenHeight)

        # map from finger IDs to finger paths for all currently touching fingers
        self._fingerPaths = { EventQueue._MOUSE_ID: [] }

        # true if more than one finger was used in the current touch event
        self._isMultiTouch = False

        # start polling timer
        utils.Timer(self._pollPygame, .01).start()


    def handleTouchEvent(self, finger, fingerPaths):
        """
        finger - ros Finger message (see Finger.py)
        fingerPaths - map from finger IDs to lists of points
        """

        # get the path of the finger that generated the event, in screen coordinates
        scaledPath = []
        for pt in fingerPaths[finger.id]:
            pt = utils.scaleCoords(pt, (Table.WIDTH, Table.HEIGHT),
                                   self._screenDims, False)
            scaledPath.append(pt)

        if finger.eventType == finger.FINGER_UP:
            event = self._getFingerUpEvent(finger.id)
            if event:
                self._event.append(event)
            del self._fingerPaths[finger.id]
            if not self._fingerPaths:
                self._isMultiTouch = False

        elif finger.eventType == finger.FINGER_DOWN:
            self._events.append(FingerDownEvent(finger.id, scaledPath))

        elif finger.eventType == finger.FINGER_DRAG:
            if self._fingerPaths:
                self._isMultiTouch = True
            self._fingerPaths[finger.id] = scaledPath            
            event = self._getFingerDragEvent(finger.id, self._fingerPaths)
            if event:
                self._event.append(event)


    def _getFingerDragEvent(self, id):
        """Return the event appropriate for dragging the given finger."""
        
        # If not a multi-touch gesture, return a normal FingerDragEvent.
        if not self._isMultiTouch:
            return FingerDragEvent(id, self._fingerPaths[id])

        # If the user was making a multi-touch gesture, but has already removed
        # one finger, ignore the other finger.
        if len(self._fingerPaths) < 2:
            return None

        # Get directions of leftmost and rightmost fingers.
        leftMost = rightMost = self._fingerPaths[0];
        for id, fingerPath in self._fingerPaths.items():
            if fingerPath[-1][0] < leftMost[-1][0]:
                leftMost = fingerPath
            if fingerPath[-1][0] > rightMost[-1][0]:
                rightMost = fingerPath
        assert leftMost != rightMost
        (dx1, dy1), (dx2, dy2) = [ [path[-1][i] - path[-1][i] for i in range(2)]
                                    for path in (leftMost, rightMost) ]
        
        # If one of the fingers is not moving, ignore.
        if (dx1 == 0 and dy1 == 0) or (dx2 == 0 and dy2 == 0):
            return None

        if dx1 < 0 and dx2 > 0:
            return ZoomOutEvent()

        if dx1 > 0 and dx2 > 0:
            return ZoomOutEvent()

        dx = (dx1 + dx2)/2.
        dy = (dy1 + dy2)/2.
        return PanEvent(dx, dy)


    def _getFingerUpEvent(self, id):
        """Check if the finger was making a gesture. Otherwise return a normal finger up event."""
        if self._isMultiTouch: # currently, no multitouch events use fingerUp 
            return None
        fingerPath = self._fingerPaths[id]
        gesture = self._checkSelectGesture(fingerPath)
        if gesture:
            hitTestFunc = self._getHitTestFunc(gesture, fingerPath)
            print gesture
            return SelectEvent(hitTestFunc)
        else:
            return FingerUpEvent(id, fingerPath)


    def _pollPygame(self):
        """Convert each event in the pygame queue into a simulated multitouch event."""

        def handleNewMousePoint(pos, eventType):
            """Add a point to the current mouse path and fire the appropriate event."""
            self._fingerPaths[EventQueue._MOUSE_ID].append(pos)
            self._events.append(eventType(EventQueue._MOUSE_ID, self._fingerPaths[EventQueue._MOUSE_ID]))            

        for event in pygame.event.get():

            if event.type == KEYDOWN:
                if event.key == K_ESCAPE:
                    self._events.append(QuitEvent())

                if event.key == K_i:
                    self._events.append(ZoomInEvent(1))

                if event.key == K_o:
                    self._events.append(ZoomOutEvent(1))

            if event.type == MOUSEBUTTONDOWN:
                if event.button == 1:
                    handleNewMousePoint(event.pos, FingerDownEvent)

            if event.type == MOUSEBUTTONUP:
                if event.button == 1:
                    self._events.append(self._getFingerUpEvent(EventQueue._MOUSE_ID))
                    self._fingerPaths[EventQueue._MOUSE_ID] = []

            if event.type == MOUSEMOTION:
                if event.buttons[0]:
                    handleNewMousePoint(event.pos, FingerDragEvent)
                if event.buttons[2]:
                    self._events.append(PanEvent(event.rel[0], event.rel[1])) 


    def _checkSelectGesture(self, path):
        """
        Determine if the given path represents a selection gesture (e.g., a circle).
        Uses the Dollar gesture recognizer.
        """
        try:
            name, score = self._recognizer.recognize(path)
            if score >= EventQueue._MIN_RECOGNIZER_SCORE:
                return name
        except ZeroDivisionError:
            pass
        except AttributeError:
            pass
        return None


    def _getHitTestFunc(self, gestureName, fingerPath):
        """
        Returns a function for determining if a given point lies within 
        the selection range specified by the given finger path.  For now,
        we just comput the rectangular bounds of the selection region.
        Later, we may want to do something different if the gestureName
        is "selectCircle" as opposed to "selectRect".
        """
        minX = maxX = fingerPath[0][0]
        minY = maxY = fingerPath[0][1]
        for pt in fingerPath:
            minX = min(minX, pt[0])
            maxX = max(maxX, pt[0])
            minY = min(minY, pt[1])
            maxY = max(maxY, pt[1])
        return lambda pt: pt[0] >= minX and pt[0] <= maxX and \
                          pt[1] >= minY and pt[1] <= maxY
        

    def popEvents(self):
        """Remove all events from the queue and return a list containing the events."""
        events = self._events[:]
        self._events = []
        return events



class PanEvent():
    def __init__(self, dx, dy):
        self.dx = dx
        self.dy = dy


class _ZoomEvent():
    def __init__(self, extent):
        self.extent = extent


class ZoomInEvent(_ZoomEvent):
    pass


class ZoomOutEvent(_ZoomEvent):
    pass


class QuitEvent:
    pass


class _FingerEvent:
    def __init__(self, id, path):
        self.id = id
        self.path = path


class FingerUpEvent(_FingerEvent):
    pass


class FingerDragEvent(_FingerEvent):
    pass


class FingerDownEvent(_FingerEvent):
    pass


class SelectEvent:
    def __init__(self, hitTestFunc):
        self._hitTestFunc = hitTestFunc
    def containsPt(self, pt):
        return self._hitTestFunc(pt)

