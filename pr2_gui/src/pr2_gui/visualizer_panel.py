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
import ogre_visualizer

class DefaultVisualizationPanel(ogre_visualizer.VisualizationPanel):
    def __init__(self, parent):
        ogre_visualizer.VisualizationPanel.__init__(self, parent)
        
    def createDefaultVisualizers(self):
        self.createGridVisualizer( "Grid", True )
        self.createAxesVisualizer( "Origin Axes", False )
        self.createMarkerVisualizer( "Visualization Markers", True )
        
        robot_vis = self.createRobotModelVisualizer( "Robot Model", False )
        robot_vis.setRobotDescription( "robotdesc/pr2" )
        
        planning = self.createPlanningVisualizer( "Planning", False )
        planning.initialize( "robotdesc/pr2", "display_kinematic_path" )
        
        point_cloud = self.createPointCloudVisualizer( "Stereo Full Cloud", True )
        point_cloud.setTopic( "videre/cloud" )
        point_cloud.setColor( ogre_visualizer.Color(1.0, 1.0, 1.0) )
        
        point_cloud = self.createPointCloudVisualizer( "Head Full Cloud", True )
        point_cloud.setTopic( "full_cloud" )
        point_cloud.setColor( ogre_visualizer.Color(1.0, 1.0, 0.0) )
        
        point_cloud = self.createPointCloudVisualizer( "World 3D Map", True )
        point_cloud.setTopic( "world_3d_map" )
        point_cloud.setColor( ogre_visualizer.Color(1.0, 0.0, 0.0) )
        point_cloud.setBillboardSize( 0.01 )
        
        laser_scan = self.createLaserScanVisualizer( "Head Scan", True )
        laser_scan.setScanTopic( "tilt_scan" )
        laser_scan.setColor( ogre_visualizer.Color(1.0, 0.0, 0.0) )
        laser_scan.setDecayTime( 30.0 )
        
        laser_scan = self.createLaserScanVisualizer( "Floor Scan", True )
        laser_scan.setScanTopic( "base_scan" )
        laser_scan.setColor( ogre_visualizer.Color(0.0, 1.0, 0.0) )
        laser_scan.setDecayTime( 0.0 )
        
        self.createOctreeVisualizer( "Octree", True ).setOctreeTopic( "full_octree" )
        
