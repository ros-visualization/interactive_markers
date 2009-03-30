import rospy
from Finger import Finger

class Table:
    """Tracks the paths of multiple fingers."""

    # keep these constants in sync with the resolution specifies in main.cpp
    # of the touchKit app
    WIDTH = 1024
    HEIGHT = 768

    def __init__(self, eventCallback):
        self._fingerPaths = {}   # map from integer IDs to lists of points
        self._eventCallback = eventCallback
        rospy.Subscriber("TouchEvent", Finger, Table._handleTouchEvent, self)

    @staticmethod
    def _handleTouchEvent(finger, self):
             
        # update path for the corresponding finger
        if not self._fingerPaths.has_key(finger.id):
            self._fingerPaths[finger.id] = []
        path = self._fingerPaths[finger.id]
        path.append((finger.pos.x, finger.pos.y))

        # notify listeners
        self._eventCallback(finger, self._fingerPaths)

        # remove finger tracker if finger up
        if finger.eventType == finger.FINGER_UP:
            self._fingerPaths.pop(finger.id)
