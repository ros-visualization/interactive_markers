#!/usr/bin/python

import rostools
import rostools.packspec
rostools.update_path('ogre_visualizer')

import wx
import ogre_visualizer
import ogre_tools

class VisualizerFrame(wx.Frame):
    _CONFIG_WINDOW_X="/Window/X"
    _CONFIG_WINDOW_Y="/Window/Y"
    _CONFIG_WINDOW_WIDTH="/Window/Width"
    _CONFIG_WINDOW_HEIGHT="/Window/Height"
    
    def __init__(self, parent, id=wx.ID_ANY, title='Standalone Visualizer', pos=wx.DefaultPosition, size=(800, 600), style=wx.DEFAULT_FRAME_STYLE):
        wx.Frame.__init__(self, parent, id, title, pos, size, style)
        
        self._config = wx.Config("standalone_visualizer")

        ogre_tools.initializeOgre()
        
        visualizer_panel = ogre_visualizer.VisualizationPanel(self)
        
        self._visualizer_panel = visualizer_panel
        
        media_path = rostools.packspec.get_pkg_dir( "gazebo_robot_description" )
        media_path += "/world/Media/";
        
        media_paths = [media_path]
        media_paths.append( media_path )
        media_paths.append( media_path + "fonts" )
        media_paths.append( media_path + "materials" )
        media_paths.append( media_path + "materials/scripts" )
        media_paths.append( media_path + "materials/programs" )
        media_paths.append( media_path + "materials/textures" )
        media_paths.append( media_path + "models" )
        media_paths.append( media_path + "models/pr2_new" )
        
        ogre_tools.initializeResources( media_paths )
        
        visualizer_panel.createGridVisualizer( "Grid", True )
        visualizer_panel.createAxesVisualizer( "Origin Axes", False )
        visualizer_panel.createMarkerVisualizer( "Visualization Markers", True )
        
        robot_vis = visualizer_panel.createRobotModelVisualizer( "Robot Model", False )
        robot_vis.setRobotDescription( "robotdesc/pr2" )
        
        planning = visualizer_panel.createPlanningVisualizer( "Planning", False )
        planning.initialize( "robotdesc/pr2", "display_kinematic_path" )
        
        point_cloud = visualizer_panel.createPointCloudVisualizer( "Stereo Full Cloud", True )
        point_cloud.setTopic("videre/cloud")
        point_cloud.setColor(ogre_visualizer.Color(1.0, 1.0, 1.0))
        
        point_cloud = visualizer_panel.createPointCloudVisualizer( "Head Full Cloud", True )
        point_cloud.setTopic( "full_cloud" )
        point_cloud.setColor(ogre_visualizer.Color(1.0, 1.0, 0.0))
        
        point_cloud = visualizer_panel.createPointCloudVisualizer( "World 3D Map", True )
        point_cloud.setTopic( "world_3d_map" )
        point_cloud.setColor(ogre_visualizer.Color(1.0, 0.0, 0.0))
        point_cloud.setBillboardSize( 0.01 )
        
        laser_scan = visualizer_panel.createLaserScanVisualizer( "Head Scan", True )
        laser_scan.setScanTopic( "tilt_scan" )
        laser_scan.setColor(ogre_visualizer.Color(1.0, 0.0, 0.0))
        laser_scan.setDecayTime( 30.0 )
        
        laser_scan = visualizer_panel.createLaserScanVisualizer( "Floor Scan", True )
        laser_scan.setScanTopic( "base_scan" )
        laser_scan.setColor(ogre_visualizer.Color(0.0, 1.0, 0.0))
        laser_scan.setDecayTime( 0.0 )
        
        visualizer_panel.createOctreeVisualizer( "Octree", True ).setOctreeTopic( "full_octree" )
        
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
        
        visualizer_panel.loadConfig(self._config)
        
        self.Bind(wx.EVT_CLOSE, self.on_close)
        
    def on_close(self, event):
        self._config.DeleteAll()
        
        (x, y) = self.GetPositionTuple()
        (width, height) = self.GetSizeTuple()
        self._config.WriteInt(self._CONFIG_WINDOW_X, x)
        self._config.WriteInt(self._CONFIG_WINDOW_Y, y)
        self._config.WriteInt(self._CONFIG_WINDOW_WIDTH, width)
        self._config.WriteInt(self._CONFIG_WINDOW_HEIGHT, height)
        
        self._visualizer_panel.saveConfig(self._config)
        self.Destroy()

class VisualizerApp(wx.App):
    def OnInit(self):
        frame = VisualizerFrame(None, wx.ID_ANY, "Standalone Visualizer", wx.DefaultPosition, wx.Size( 800, 600 ) )
        frame.Show(True)
        return True
        
    def OnExit(self):        
        ogre_tools.cleanupOgre()
        
        

if __name__ == "__main__":
    app = VisualizerApp()
    app.MainLoop()
