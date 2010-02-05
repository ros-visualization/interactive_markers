#!/usr/bin/env python

import roslib; roslib.load_manifest('marker_shmup')

import wx
import rospy
import marker_shmup.msg

class Frame(wx.Frame):
    def __init__(self):
        wx.Frame.__init__(self, None, wx.ID_ANY, "shmup control")
        
        self.timer = wx.Timer(self)
        self.Bind(wx.EVT_TIMER, self.on_timer)
        self.timer.Start(33)

        self.pub = rospy.Publisher("shmup_command", marker_shmup.msg.HeroCommand)
        self.Bind(wx.EVT_KEY_DOWN, self.on_key_down)
        self.Bind(wx.EVT_KEY_UP, self.on_key_up)
        self.h = marker_shmup.msg.HeroCommand()

    def on_timer(self, evt):
        self.pub.publish(self.h)

    def on_key_down(self, evt):
        code = evt.GetKeyCode()
        if (code == wx.WXK_UP):
            self.h.x = 5.0
        elif (code == wx.WXK_DOWN):
            self.h.x = -5.0
        elif (code == wx.WXK_LEFT):
            self.h.y = 5.0
        elif (code == wx.WXK_RIGHT):
            self.h.y = -5.0
        elif (code == wx.WXK_SPACE):
            self.h.fire = True
        
        if (evt.CmdDown()):
            self.h.shield = True

    def on_key_up(self, evt):
        code = evt.GetKeyCode()
        if (code == wx.WXK_UP and self.h.x == 5.0):
            self.h.x = 0.0
        elif (code == wx.WXK_DOWN and self.h.x == -5.0):
            self.h.x = 0.0
        elif (code == wx.WXK_LEFT and self.h.y == 5.0):
            self.h.y = 0.0
        elif (code == wx.WXK_RIGHT and self.h.y == -5.0):
            self.h.y = 0.0
        elif (code == wx.WXK_SPACE):
            self.h.fire = False
        
        if (not evt.CmdDown()):
            self.h.shield = False

rospy.init_node('wxkey2shmup')

app = wx.PySimpleApp()
frame = Frame()
frame.Show(True)
app.MainLoop()
