/*
 * Copyright (c) 2008, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

/**
 * \file
 */

#include <wx/wx.h>

#include "ros/node.h"
#include "ros/common.h"

#include "ogre_tools/initialization.h"

#include "visualization_panel.h"
#include "visualizers/grid_visualizer.h"
#include "visualizers/axes_visualizer.h"
#include "visualizers/point_cloud_visualizer.h"
#include "visualizers/laser_scan_visualizer.h"
#include "visualizers/robot_model_visualizer.h"
#include "visualizers/marker_visualizer.h"
#include "visualizers/octree_visualizer.h"
#include "visualizers/planning_visualizer.h"

#include "Ogre.h"

using namespace ogre_vis;

class MyFrame : public wxFrame
{
public:
  MyFrame(wxWindow* parent) : wxFrame(parent, -1, _("Ogre Visualizer Test App"),
                      wxDefaultPosition, wxSize(800,600),
                      wxDEFAULT_FRAME_STYLE)
  {
    ogre_tools::initializeOgre();

    visualization_panel_ = new VisualizationPanel( this );

    root_ = Ogre::Root::getSingletonPtr();

    std::string mediaPath = ros::get_package_path( "gazebo_robot_description" );
    mediaPath += "/world/Media/";
    ogre_tools::V_string paths;
    paths.push_back( mediaPath );
    paths.push_back( mediaPath + "fonts" );
    paths.push_back( mediaPath + "materials" );
    paths.push_back( mediaPath + "materials/scripts" );
    paths.push_back( mediaPath + "materials/programs" );
    paths.push_back( mediaPath + "materials/textures" );
    paths.push_back( mediaPath + "models" );
    paths.push_back( mediaPath + "models/pr2_new" );

    ogre_tools::initializeResources( paths );

    visualization_panel_->createVisualizer<GridVisualizer>( "Grid", true );
    visualization_panel_->createVisualizer<AxesVisualizer>( "Origin Axes", true );

    RobotModelVisualizer* model = visualization_panel_->createVisualizer<RobotModelVisualizer>( "Robot Model", true );
    model->setRobotDescription( "robotdesc/pr2" );
    PlanningVisualizer* planning = visualization_panel_->createVisualizer<PlanningVisualizer>( "Planning", false );
    planning->initialize( "robotdesc/pr2", "display_kinematic_path" );

    PointCloudVisualizer* pointCloud = visualization_panel_->createVisualizer<PointCloudVisualizer>( "Stereo Full Cloud", true );
    pointCloud->setTopic( "videre/cloud" );
    pointCloud->setColor( Color( 1.0, 1.0, 1.0 ) );

    pointCloud = visualization_panel_->createVisualizer<PointCloudVisualizer>( "Head Full Cloud", true );
    pointCloud->setTopic( "full_cloud" );
    pointCloud->setColor( Color( 1.0, 1.0, 0.0 ) );

    pointCloud = visualization_panel_->createVisualizer<PointCloudVisualizer>( "World 3D Map", true );
    pointCloud->setTopic( "world_3d_map" );
    pointCloud->setColor( Color( 1.0f, 0.0f, 0.0f ) );
    pointCloud->setBillboardSize( 0.01 );

    LaserScanVisualizer* laserScan = visualization_panel_->createVisualizer<LaserScanVisualizer>( "Head Scan", true );
    laserScan->setScanTopic( "tilt_scan" );
    laserScan->setColor( Color( 1.0, 0.0, 0.0 ) );
    laserScan->setDecayTime( 30.0f );

    laserScan = visualization_panel_->createVisualizer<LaserScanVisualizer>( "Floor Scan", true );
    laserScan->setScanTopic( "base_scan" );
    laserScan->setColor( Color( 0.0f, 1.0f, 0.0f ) );
    laserScan->setDecayTime( 0.0f );

    visualization_panel_->createVisualizer<OctreeVisualizer>( "Octree", false )->setOctreeTopic( "full_octree" );
    visualization_panel_->createVisualizer<MarkerVisualizer>( "Visualization Markers", true );
  }

  ~MyFrame()
  {
    visualization_panel_->Destroy();

    delete root_;
  }

private:

  Ogre::Root* root_;
  Ogre::Camera* camera_;
  Ogre::SceneManager* scene_manager_;

  VisualizationPanel* visualization_panel_;
};

// our normal wxApp-derived class, as usual
class MyApp : public wxApp
{
public:
  char** localArgv;

  bool OnInit()
  {
    // create our own copy of argv, with regular char*s.
    localArgv =  new char*[ argc ];
    for ( int i = 0; i < argc; ++i )
    {
      localArgv[ i ] = strdup( wxString( argv[ i ] ).fn_str() );

      printf( "argv[%d]: %s\n", i, localArgv[i] );
    }

    ros::init(argc, localArgv);
    new ros::node( "Visualizer Test", ros::node::DONT_HANDLE_SIGINT );

    wxFrame* frame = new MyFrame(NULL);
    SetTopWindow(frame);
    frame->Show();
    return true;
  }

  int OnExit()
  {
    ros::fini();

    for ( int i = 0; i < argc; ++i )
    {
      free( localArgv[ i ] );
    }
    delete [] localArgv;

    return 0;
  }
};

DECLARE_APP(MyApp);
IMPLEMENT_APP(MyApp);
