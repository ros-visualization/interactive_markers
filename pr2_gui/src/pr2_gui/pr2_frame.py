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
import wx_camera_panel
import wx_rosout
import ogre_tools
import visualizer_panel

class PR2Frame(wx.Frame):
    _PERSPECTIVE_VERSION=1
    _CONFIG_PERSPECTIVE="/Perspective/Perspective"
    _CONFIG_PERSPECTIVE_VERSION="/Perspective/Version"
    
    _CONFIG_WINDOW_X="/Window/X"
    _CONFIG_WINDOW_Y="/Window/Y"
    _CONFIG_WINDOW_WIDTH="/Window/Width"
    _CONFIG_WINDOW_HEIGHT="/Window/Height"
    
    def __init__(self, parent, id=wx.ID_ANY, title='PR2 GUI', pos=wx.DefaultPosition, size=(1280, 1024), style=wx.DEFAULT_FRAME_STYLE):
        wx.Frame.__init__(self, parent, id, title, pos, size, style)
        
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
        media_paths.append(media_path + "models/pr2_new")
        
        ogre_tools.initializeResources( media_paths )
        self._visualizer_panel.createDefaultVisualizers()
        
        self._rosout_panel = wx_rosout.RosoutPanel(self)
        self._rosout_panel.setEnabled(True)
        
        self._aui_manager.AddPane(self._visualizer_panel, wx.aui.AuiPaneInfo().CenterPane().Center().Name('3dvis').Caption('3D Visualization'), '3D Visualization')
        self._aui_manager.AddPane(self._rosout_panel, wx.aui.AuiPaneInfo().BottomDockable().Bottom().Name('rosout').Caption('Rosout'), 'Rosout')
        
        self.add_camera_pane("forearm_right", False)
        self.add_camera_pane("forearm_left", False)
        self.add_camera_pane("axis_right", True)
        self.add_camera_pane("axis_left", True)
        self.add_camera_pane("stereo_left", False)
        
        self._aui_manager.Update()
        
        self.create_menu_bar()
        
        self.load_config()
        
        self.Bind(wx.aui.EVT_AUI_PANE_CLOSE, self.on_pane_close)
        self.Bind(wx.EVT_CLOSE, self.on_close)
        
    def add_camera_pane(self, name, ptz_enabled):
        panel = wx_camera_panel.CameraPanel(self)
        panel.setName(name)
        panel.setEnabled(True)
        panel.setPTZEnabled(ptz_enabled)
        
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
        
        config.Flush()
        
    def on_close(self, event):
        self.save_config()
        
        self._aui_manager.UnInit()
        self.Destroy()
        
    def on_pane_close(self, event):
        pane = event.GetPane()
        window = pane.window
        self.GetMenuBar().Check(window.GetId(), False)
        
        window.setEnabled(False)
        
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
                    pane.window.setEnabled(event.IsChecked())
                    break
                
            self._aui_manager.Update()
