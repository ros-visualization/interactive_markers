#!/usr/bin/env python
# Software License Agreement (BSD License)
#
# Copyright (c) 2008, Willow Garage, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of the Willow Garage nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

__doc__ = """
ROS_PLOT
A small utility to plot data from ROS topics."""



import  wx

import rostools
rostools.update_path('wxpy_ros')
import wxpy_ros
import rospy


# The list of subtopics we want to plot. It is a map between the name of the window and the list of pairs of (path, style)
#a_plot=('/mechanism_state/actuator_states[3]/position', 'r')
#another_plot=('/mechanism_state/actuator_states[2]/position', 'b')
#a_list_of_plots=[a_plot,another_plot]
## And we specify the timespan as well
#all_plots = {'positions' : (a_list_of_plots,10)}

#Here is what it looks like:

#plots1=[('/mechanism_state/actuator_states[8]/position', 'r'),\
  #('/mechanism_state/actuator_states[7]/position', 'b'), \
  #('/mechanism_state/actuator_states[6]/position', 'g'), \
  #('/mechanism_state/actuator_states[5]/position', 'y'), \
  #('/mechanism_state/actuator_states[4]/position', 'm'), \
  #('/mechanism_state/actuator_states[3]/position', 'y'), \
  #('/mechanism_state/actuator_states[2]/position', 'y')]
  #('/mechanism_state/actuator_states[6]/position', 'm')]

plots1=[]
#plots1.append(('/mechanism_state/joint_states[0]/velocity', 'r'))
 #+ 0     ==> elbow_right_joint
 #+ 1     ==> upperarm_roll_right_joint
 #+ 2     ==> shoulder_pitch_right_joint
 #+ 3     ==> shoulder_pan_right_joint
plots2=[]
plots2.append(('/mechanism_state/joint_states[0]/position', 'r'))
plots2.append(('/mechanism_state/joint_states[1]/position', 'g'))
plots2.append(('/mechanism_state/joint_states[2]/position', 'b'))
plots2.append(('/mechanism_state/joint_states[3]/position', 'm'))
#plots2.append(('/mechanism_state/joint_states[7]/position', 'm'))

#plots1.append(('/mechanism_state/joint_states[7]/commanded_effort', 'r'))
#plots1.append(('/mechanism_state/joint_states[7]/applied_effort', 'b'))
#plots1.append(('/mechanism_state/joint_states[1]/velocity', 'g'))
#plots1.append(('/mechanism_state/joint_states[2]/velocity', 'b'))
#plots1.append(('/mechanism_state/joint_states[3]/velocity', 'm'))

#plots2.append(('/mechanism_state/actuator_states[3]/position', 'r'))
  #('/mechanism_state/actuator_states[7]/velocity', 'b'), \
  #('/mechanism_state/actuator_states[6]/velocity', 'g'), \
  #('/mechanism_state/actuator_states[5]/velocity', 'y'), \
  #('/mechanism_state/actuator_states[4]/velocity', 'm'), \
  #('/mechanism_state/actuator_states[3]/velocity', 'y'), \
  #('/mechanism_state/actuator_states[2]/velocity', 'y')]

# And we specify the timespan as well
all_plots = {'plots 1' : (plots2,20),'plots 2' : (plots1,20)}


#
# You can stop reading at that point if you just want some stuff sliding on the screen
#

# Create a message handler, that will take care of the communication with ROS
messageHandler = wxpy_ros.getMessageHandler()


class NamesChannel(wxpy_ros.wxplot.Channel):
  def __init__(self, itempath, style):
    pass
    
  def callback(self, mechstate, time=None):
    print 'Act names:'
    i = 0
    for j in mechstate.actuator_states:
      print ' + %i \t ==> %s '%(i,j.name)
      i+=1
    print 'Joint names:'
    i = 0
    for j in mechstate.joint_states:
      print ' + %i \t ==> %s '%(i,j.name)
      i+=1

messageHandler.subscribe('/mechanism_state/', '', NamesChannel)

# Creating an app
app = wx.App()
# Creating and popuplating the plots
frames = []
for fname in all_plots:
  frame = wx.Frame(None, -1, fname)
  plot = wxpy_ros.WXSlidingPlot(frame)
  pobject = all_plots[fname]
  plot.setTimespan(pobject[1])
  for stopic in pobject[0]:
    subtopic_name = stopic[0]
    subtopic_style = stopic[1]
    channel = messageHandler.subscribe(subtopic_name, subtopic_style)
    plot.addChannel(channel)
  frame.Show()
  frames.append(frame)
app.MainLoop()
print 'quit'
rospy.signal_shutdown('GUI shutdown')
