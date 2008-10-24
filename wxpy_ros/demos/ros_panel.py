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

import pdb
import  wx
import  wx.xrc  as  xrc

import rostools
rostools.update_path('wxpy_ros')
import wxpy_ros
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
        self.path_txt = xrc.XRCCTRL(self,'path_txt')
        
        self.plot = wxpy_ros.WXSlidingPlot(self.plot_panel)
#        hb = wx.BoxSizer(wx.VERTICAL)
#        self.plot_panel.SetSizer(hb)
#        hb.Add(self.plot)
#        hb.Add(self.values_lctr, wx.ALL|wx.EXPAND)
        wxpy_ros.NestPanel(self.plot_panel,self.plot)
        
        #disabled for now due to a frustrating layout problem
        #self.values_lctr.InsertColumn(0,'Slot')
        #self.values_lctr.InsertColumn(1,'Value')


        self.Center()
        self.Fit()
        self.Show(True)
        
        self.topics = []
        self.selectedTopic = None
        self.selectedSlot = None
        self.slots = None
        
        self.messageHandler = wxpy_ros.getHandler()
        
        self.Bind(wx.EVT_BUTTON, self.OnRefreshTopicList, self.refresh_but)
        self.Bind(wx.EVT_BUTTON, self.OnSubscribe, self.add_but)
        self.Bind(wx.EVT_LISTBOX, self.OnSelectedTopic, self.topics_list)
        self.Bind(wx.EVT_LISTBOX, self.OnSelectedSlot, self.slot_list)
        self.Bind(wx.EVT_TEXT, self.OnTextChanged, self.path_txt)
        self.Bind(wx.EVT_SCROLL_THUMBRELEASE, self.OnChangedTime, self.time_slider)
        
        self.add_but.Disable()
        self.plot.setTimespan(self.time_slider.GetValue())
    
    def OnRefreshTopicList(self,e):
        self.messageHandler.queryTopicsTree()
        self.topics = wxpy_ros.getTopicsList()
        print [ t[0] for t in self.topics ]
        self.topics_list.Set([ t[0] for t in self.topics ])
      
    def OnSelectedTopic(self, e):
        index = e.GetSelection()
        self.selectedTopic = self.topics[index]
        print 'selected %s' % self.selectedTopic[0]
        self.slots = self.messageHandler.getItemPaths(self.selectedTopic[0])
        for s in self.slots:
            print s
        self.slot_list.Set(self.slots)
    
    def OnSubscribe(self, e):
        if not self.selectedTopic or not self.selectedSlot:
            return
        print 'subscribing ',self.selectedTopic, self.selectedSlot
        channel = self.messageHandler.subscribe(self.path_txt.GetValue(), self.style_txt.GetValue())
        assert channel
        self.plot.addChannel(channel)
        #print channel
      
    def OnSelectedSlot(self, e):
        index = e.GetSelection()
        self.selectedSlot = self.slots[index]
        self.path_txt.SetValue(self.selectedSlot)
        
    def OnChangedTime(self, e):
        print self.time_slider.GetValue()
        self.plot.setTimespan(self.time_slider.GetValue())
        
    def OnClose(self, e):
        print 'closing'
    
    def OnTextChanged(self, e):
        s = self.path_txt.GetValue()
        if self.messageHandler.isValidItemPath(s):
            self.path_txt.SetBackgroundColour('Green')
            self.add_but.Enable()
            print '%s is valid' %s
        else:
            self.path_txt.SetBackgroundColour('Orange')
            self.add_but.Disable()
            print '%s is invalid' %s

app = wx.App()
frame = RosFrame(None, -1)
frame.Show()
app.MainLoop()
print 'quit'
rospy.signal_shutdown('GUI shutdown')