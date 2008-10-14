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

import rostools
rostools.update_path('pr2_gui')

import wx
from wx import xrc
import rospy
import rospy.service
import std_srvs.srv

PKG='pr2_gui'

class ResetPanel(wx.Panel):
    def __init__(self, parent):
        wx.Panel.__init__(self, parent)
        
        xrc_path = rostools.packspec.get_pkg_dir(PKG) + '/xrc/reset_panel.xrc'
        
        self._xrc = xrc.XmlResource(xrc_path)
        self._real_panel = self._xrc.LoadPanel(self, 'ResetPanel')
        sizer = wx.BoxSizer(wx.VERTICAL)
        sizer.Add(self._real_panel, 1, wx.EXPAND)
        self.SetSizer(sizer)
        
        self._reset_motors = xrc.XRCCTRL(self._real_panel, "reset_motors_button")
        
        self._reset_motors.Bind(wx.EVT_BUTTON, self.on_reset_motors)
        
    def on_reset_motors(self, event):
        reset = rospy.ServiceProxy("reset_motors", std_srvs.srv.Empty)
         
        try:
            reset()
        except rospy.service.ServiceException:
            rospy.logerr('Failed to reset the motors: service call failed')
            