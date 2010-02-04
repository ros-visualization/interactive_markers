#!/usr/bin/env python

from ros import joy
from ros import rospy
from ros import marker_shmup

import joy.msg
import marker_shmup.msg

class Joy2Shmup():
    def __init__(self):
        self._sub = rospy.Subscriber("joy", joy.msg.Joy, self.joy)
        self._pub = rospy.Publisher("shmup_command", marker_shmup.msg.HeroCommand)

    def joy(self, msg):
        h = marker_shmup.msg.HeroCommand()
        h.x = msg.axes[3] * 5.0
        h.y = msg.axes[2] * 5.0
        h.fire = msg.buttons[11]
        h.shield = msg.buttons[10]
        self._pub.publish(h)

rospy.init_node('joy2shmup')
j = Joy2Shmup()

rospy.spin()
