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
import wx.aui
import wx.py.shell
import wx_camera_panel
import rxtools
import ogre_tools
import visualizer_panel
import runtime_monitor
import hardware_panel
from runtime_monitor.monitor_panel import *

import rospy

class PR2Frame(wx.Frame):
    _PERSPECTIVE_VERSION=2
    _CONFIG_PERSPECTIVE="/Perspective/Perspective"
    _CONFIG_PERSPECTIVE_VERSION="/Perspective/Version"
    
    _CONFIG_WINDOW_X="/Window/X"
    _CONFIG_WINDOW_Y="/Window/Y"
    _CONFIG_WINDOW_WIDTH="/Window/Width"
    _CONFIG_WINDOW_HEIGHT="/Window/Height"
    
    def __init__(self, parent, id=wx.ID_ANY, title='PR2 GUI', pos=wx.DefaultPosition, size=(1280, 1024), style=wx.DEFAULT_FRAME_STYLE):
        wx.Frame.__init__(self, parent, id, title, pos, size, style)
        
        rospy.ready('pr2_gui', anonymous=True)
        
        self._config = wx.Config("pr2_gui")
        self._aui_manager = wx.aui.AuiManager(self)

        self._visualizer_panel = visualizer_panel.DefaultVisualizationPanel(self)
        
        media_path = rostools.packspec.get_pkg_dir( "gazebo_robot_description" )
        media_path += "/world/Media/";
        
        media_paths = [media_path]
        media_paths.append(media_path)
        media_paths.append(media_path + "fonts")
        media_paths.append(media_path + "materials")
        media_paths.append(media_path + "materials/scripts")
        media_paths.append(media_path + "materials/programs")
        media_paths.append(media_path + "materials/textures")
        media_paths.append(media_path + "models")
        media_paths.append(media_path + "models/pr2")
        
        ogre_tools.initializeResources( media_paths )
        self._visualizer_panel.createDefaultVisualizers()
        
        self._rosout_panel = rxtools.RosoutPanel(self)
        
        self._monitor_panel = MonitorPanel(self)
        self._monitor_panel.set_new_errors_callback(self.on_monitor_errors_received)
        
        self._interpreter = wx.py.shell.Shell(self)
        
        self._hardware_panel = hardware_panel.HardwarePanel(self)
        
        self._aui_manager.AddPane(self._visualizer_panel, wx.aui.AuiPaneInfo().CenterPane().Center().Name('3dvis').Caption('3D Visualization'), '3D Visualization')
        
        self._aui_manager.AddPane(self._interpreter, wx.aui.AuiPaneInfo().BottomDockable().Bottom().Name('interpreter').Caption('Python Interpreter').Hide(), 'Python Interpreter')
    
        self._aui_manager.AddPane(self._rosout_panel, wx.aui.AuiPaneInfo().BottomDockable().Bottom().Layer(1).BestSize(wx.Size(700, 200)).Name('rosout').Caption('Rosout'), 'Rosout')
        self._aui_manager.AddPane(self._monitor_panel, wx.aui.AuiPaneInfo().BottomDockable().Bottom().Layer(1).BestSize(wx.Size(700,600)).Name('runtime_monitor').Caption('Runtime Monitor'), 'Runtime Monitor')
        self._aui_manager.AddPane(self._hardware_panel, wx.aui.AuiPaneInfo().BottomDockable().Bottom().Layer(1).BestSize(wx.Size(300,200)).Name('hardware').Caption('Hardware'), 'Hardware')
        
        self.add_camera_pane("forearm_r", False)
        self.add_camera_pane("forearm_l", False)
        self.add_camera_pane("axis_r", True)
        self.add_camera_pane("axis_l", True)
        self.add_camera_pane("stereo_l", False)
        
        self._aui_manager.Update()
        
        self.load_config()
        
        self.create_menu_bar()
        
        self.Bind(wx.aui.EVT_AUI_PANE_CLOSE, self.on_pane_close)
        self.Bind(wx.EVT_CLOSE, self.on_close)
        
    def add_camera_pane(self, name, ptz_enabled):   
        panel = wx_camera_panel.CameraPanel(self)
        panel.setName(name)
        panel.setPTZEnabled(ptz_enabled)
        panel.setMinimal(True)
        
        caption = 'Camera ' + name
        self._aui_manager.AddPane(panel, wx.aui.AuiPaneInfo().Right().Layer(1).Name(caption).Caption(caption), caption)
        
    def create_menu_bar(self):
        menubar = wx.MenuBar()
        
        # file menu
        self._file_menu = wx.Menu()
        self._file_menu.Append(wx.ID_EXIT, "E&xit")
        menubar.Append(self._file_menu, "&File")
        
        #panels menu
        self._panels_menu = wx.Menu()
        panes = self._aui_manager.GetAllPanes()
        for pane in panes:
            if (pane.IsMovable()):
                self._panels_menu.AppendCheckItem(pane.window.GetId(), pane.caption).Check(pane.IsShown())
            
        menubar.Append(self._panels_menu, "&Panels")
        
        self.SetMenuBar(menubar)
        
        self.Bind(wx.EVT_MENU, self.on_menu)
        
    def load_config(self):
        # Load the auimanager's perspective
        if (self._config.HasEntry(self._CONFIG_PERSPECTIVE)):
            version = self._config.ReadInt(self._CONFIG_PERSPECTIVE_VERSION)
            if (version == self._PERSPECTIVE_VERSION):
                perspective = self._config.Read(self._CONFIG_PERSPECTIVE)
                self._aui_manager.LoadPerspective(perspective, True)
                
        # Enable/disable the panels depending on whether they're shown or not
        panes = self._aui_manager.GetAllPanes()
        for pane in panes:
            set_enabled_func = getattr(pane.window, "setEnabled", None)
            if (set_enabled_func != None):
                set_enabled_func(pane.IsShown()) 
                
        # Load our window options
        (x, y) = self.GetPositionTuple()
        (width, height) = self.GetSizeTuple()
        if (self._config.HasEntry(self._CONFIG_WINDOW_X)):
            x = self._config.ReadInt(self._CONFIG_WINDOW_X)
        if (self._config.HasEntry(self._CONFIG_WINDOW_Y)):
            y = self._config.ReadInt(self._CONFIG_WINDOW_Y)
        if (self._config.HasEntry(self._CONFIG_WINDOW_WIDTH)):
            width = self._config.ReadInt(self._CONFIG_WINDOW_WIDTH)
        if (self._config.HasEntry(self._CONFIG_WINDOW_HEIGHT)):
            height = self._config.ReadInt(self._CONFIG_WINDOW_HEIGHT)
        
        self.SetPosition((x, y))
        self.SetSize((width, height))
        
        # load panel-specific configurations
        panes = self._aui_manager.GetAllPanes()
        for pane in panes:
            load_config_func = getattr(pane.window, "loadConfig", None)
            if (load_config_func != None):
                old_path = self._config.GetPath()
                self._config.SetPath(old_path + "/" + pane.name)
                load_config_func(self._config)
                self._config.SetPath(old_path)
        
    def save_config(self):
        config = self._config
        config.WriteInt(self._CONFIG_PERSPECTIVE_VERSION, self._PERSPECTIVE_VERSION)
        config.Write(self._CONFIG_PERSPECTIVE, self._aui_manager.SavePerspective())
        
        (x, y) = self.GetPositionTuple()
        (width, height) = self.GetSizeTuple()
        config.WriteInt(self._CONFIG_WINDOW_X, x)
        config.WriteInt(self._CONFIG_WINDOW_Y, y)
        config.WriteInt(self._CONFIG_WINDOW_WIDTH, width)
        config.WriteInt(self._CONFIG_WINDOW_HEIGHT, height)
        
        # save out panel-specific configurations
        panes = self._aui_manager.GetAllPanes()
        for pane in panes:
            save_config_func = getattr(pane.window, "saveConfig", None)
            if (save_config_func != None):
                old_path = config.GetPath()
                sub_path = old_path + "/" + pane.name;
                config.DeleteGroup(sub_path)
                config.SetPath(sub_path)
                save_config_func(config)
                config.SetPath(old_path)
        
        config.Flush()
        
    def on_close(self, event):
        self.save_config()
        
        self._aui_manager.UnInit()
        self.Destroy()
        
    def on_pane_close(self, event):
        pane = event.GetPane()
        window = pane.window
        self.GetMenuBar().Check(window.GetId(), False)
        
        set_enabled_func = getattr(window, "setEnabled", None)
        if (set_enabled_func != None):
            set_enabled_func(pane.IsShown()) 
        
        self._aui_manager.Update()
        
        
    def on_menu(self, event):
        if (event.GetEventObject() == self._file_menu):
            if (event.GetId() == wx.ID_EXIT):
                self.Close()
                return
        elif (event.GetEventObject() == self._panels_menu):
            panes = self._aui_manager.GetAllPanes()
            for pane in panes:
                if (pane.window.GetId() == event.GetId()):
                    pane.Show(event.IsChecked())
                    
                    set_enabled_func = getattr(pane.window, "setEnabled", None)
                    if (set_enabled_func != None):
                        set_enabled_func(event.IsChecked()) 
                    break
                
            self._aui_manager.Update()

    def on_monitor_errors_received(self):
        pane = self._aui_manager.GetPane(self._monitor_panel)
        if (not pane.IsShown()):
            pane.Show()
            self._aui_manager.Update()
