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

##"""ROS_PLOT
##A small utility class for embedding dynamic plots in WXpython.
##"""


import  wx
import  wx.xrc  as  xrc

import rostools
rostools.update_path('wxpy_ros')
import wxpy_ros
import roscom
import rospy

class RosFrame(wx.Frame):
    def __init__(self, parent, id):
        wx.Frame.__init__(self, parent, id, 'Topics', size=(300,300))
        
        self.res = xrc.XmlResource('ros_panel.xrc')        

        self.res.LoadPanel(self, 'main_panel')

        self.plot_panel = xrc.XRCCTRL(self,'plot_panel')
        self.time_slider = xrc.XRCCTRL(self,'time_slider')
        self.add_but = xrc.XRCCTRL(self,'add_but')
        self.refresh_but = xrc.XRCCTRL(self,'refresh_but')
        self.topics_list = xrc.XRCCTRL(self,'topics_list')
        self.slot_list = xrc.XRCCTRL(self,'slot_list')
        self.topics_list = xrc.XRCCTRL(self,'topics_list')
        self.rem_but = xrc.XRCCTRL(self,'rem_but')
        self.style_txt = xrc.XRCCTRL(self,'style_txt')
        self.values_lctr = xrc.XRCCTRL(self,'values_lctr')
        
        self.plot = wxpy_ros.WXSlidingPlot(self.plot_panel)
        hb = wx.BoxSizer(wx.VERTICAL)
        hb.Add(self.plot)
        hb.Add(self.values_lctr, wx.ALL|wx.EXPAND)
        self.plot_panel.SetSizer(hb)
        
        #disabled for nw due to a frustrating layout problem
        self.values_lctr.InsertColumn(0,'Slot')
        self.values_lctr.InsertColumn(1,'Value')


        self.Center()
        self.Show(True)
        
        self.topics = []
        self.selectedTopic = None
        self.selectedSlot = None
        self.slots = None
        
        self.messageHandler = roscom.RosMessageHandler()
        
        self.Bind(wx.EVT_BUTTON, self.OnRefreshTopicList, self.refresh_but)
        self.Bind(wx.EVT_BUTTON, self.OnSubscribe, self.add_but)
        self.Bind(wx.EVT_LISTBOX, self.OnSelectedTopic, self.topics_list)
        self.Bind(wx.EVT_LISTBOX, self.OnSelectedSlot, self.slot_list)
        self.Bind(wx.EVT_SCROLL_THUMBRELEASE, self.OnChangedTime, self.time_slider)
        
        self.plot.setTimespan(self.time_slider.GetValue())
    
    def OnRefreshTopicList(self,e):
        self.topics = roscom.getTopicsList()
        print [ t[0] for t in self.topics ]
        self.topics_list.Set([ t[0] for t in self.topics ])
      
    def OnSelectedTopic(self, e):
        index = e.GetSelection()
        self.selectedTopic = self.topics[index]
        print 'selected %s' % self.selectedTopic[0]
        obj = roscom.getMessageInstance(self.selectedTopic[1])
        self.slots = obj.__slots__
        for s in self.slots:
            print s, type(getattr(obj,s))
        self.slot_list.Set(self.slots)
    
    def OnSubscribe(self, e):
        print 'subscribing ',self.selectedTopic, self.selectedSlot
        channel = self.messageHandler.subscribe(self.selectedTopic[0], self.selectedTopic[1], self.selectedSlot, self.style_txt.GetValue())
        self.plot.addChannel(channel)
        #print channel
      
    def OnSelectedSlot(self, e):
        index = e.GetSelection()
        self.selectedSlot = self.slots[index]
        
    def OnChangedTime(self, e):
        print self.time_slider.GetValue()
        self.plot.setTimespan(self.time_slider.GetValue())
        
    def OnClose(self, e):
        print 'closing'

app = wx.App()
frame = RosFrame(None, -1)
frame.Show()
app.MainLoop()
print 'quit'
rospy.signal_shutdown('GUI shutdown')