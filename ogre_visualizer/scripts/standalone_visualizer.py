#!/usr/bin/python

import rostools
import rostools.packspec
rostools.update_path('ogre_visualizer')

import wx
import ogre_visualizer
import ogre_tools

class VisualizerApp(wx.App):
    def OnInit(self):
        frame = wx.Frame(None, wx.ID_ANY, "Standalone Visualizer", wx.DefaultPosition, wx.Size( 800, 600 ) )
        frame.Show(True)

        ogre_tools.initializeOgre()
        
        visualizer_panel = ogre_visualizer.VisualizationPanel(frame)
        
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
        robot_vis.initialize( "robotdesc/pr2", "transform" )
        
        planning = visualizer_panel.createPlanningVisualizer( "Planning", False )
        planning.initialize( "robotdesc/pr2", "display_kinematic_path" )
        
        point_cloud = visualizer_panel.createPointCloudVisualizer( "Stereo Full Cloud", True )
        point_cloud.setTopic( "videre/cloud" )
        point_cloud.setColor( 1.0, 1.0, 1.0 )
        
        point_cloud = visualizer_panel.createPointCloudVisualizer( "Head Full Cloud", True )
        point_cloud.setTopic( "full_cloud" )
        point_cloud.setColor( 1.0, 1.0, 0.0 )
        
        point_cloud = visualizer_panel.createPointCloudVisualizer( "World 3D Map", True )
        point_cloud.setTopic( "world_3d_map" )
        point_cloud.setColor( 1.0, 0.0, 0.0 )
        point_cloud.setBillboardSize( 0.01 )
        
        laser_scan = visualizer_panel.createLaserScanVisualizer( "Head Scan", True )
        laser_scan.setScanTopic( "tilt_scan" )
        laser_scan.setColor( 1.0, 0.0, 0.0 )
        laser_scan.setDecayTime( 30.0 )
        
        laser_scan = visualizer_panel.createLaserScanVisualizer( "Floor Scan", True )
        laser_scan.setScanTopic( "base_scan" )
        laser_scan.setColor( 0.0, 1.0, 0.0 )
        laser_scan.setDecayTime( 0.0 )
        
        visualizer_panel.createOctreeVisualizer( "Octree", True ).setOctreeTopic( "full_octree" )
        
        frame.Layout()
        frame.Fit()
        
        return True
        
    def OnExit(self):
        ogre_tools.cleanupOgre()
        
        

if __name__ == "__main__":
    app = VisualizerApp()
    app.MainLoop()
