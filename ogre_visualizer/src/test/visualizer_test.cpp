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


#include <wx/wx.h>

#include "../ogre_visualizer/visualization_panel.h"
#include "../ogre_visualizer/visualizers/grid_visualizer.h"
#include "../ogre_visualizer/visualizers/axes_visualizer.h"
#include "../ogre_visualizer/visualizers/point_cloud_visualizer.h"
#include "../ogre_visualizer/visualizers/laser_scan_visualizer.h"
#include "../ogre_visualizer/visualizers/robot_model_visualizer.h"
#include "../ogre_visualizer/visualizers/marker_visualizer.h"

#include "Ogre.h"

#include "ros/node.h"
#include "ros/common.h"

using namespace ogre_vis;

class MyFrame : public wxFrame
{
public:
  MyFrame(wxWindow* parent) : wxFrame(parent, -1, _("Ogre Visualizer Test App"),
                      wxDefaultPosition, wxSize(800,600),
                      wxDEFAULT_FRAME_STYLE)
  {
    root_ = new Ogre::Root();
    root_->loadPlugin( "RenderSystem_GL" );
    root_->loadPlugin( "Plugin_OctreeSceneManager" );
    root_->loadPlugin( "Plugin_CgProgramManager" );

    std::string mediaPath = ros::get_package_path( "gazebo_robot_description" );
    mediaPath += "/world/Media/";
    Ogre::ResourceGroupManager::getSingleton().addResourceLocation( mediaPath, "FileSystem", Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME );
    Ogre::ResourceGroupManager::getSingleton().addResourceLocation( mediaPath + "fonts", "FileSystem", Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME );
    Ogre::ResourceGroupManager::getSingleton().addResourceLocation( mediaPath + "materials", "FileSystem", Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME );
    Ogre::ResourceGroupManager::getSingleton().addResourceLocation( mediaPath + "materials/scripts", "FileSystem", Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME );
    Ogre::ResourceGroupManager::getSingleton().addResourceLocation( mediaPath + "materials/programs", "FileSystem", Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME );
    Ogre::ResourceGroupManager::getSingleton().addResourceLocation( mediaPath + "materials/textures", "FileSystem", Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME );
    Ogre::ResourceGroupManager::getSingleton().addResourceLocation( mediaPath + "models", "FileSystem", Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME );
    Ogre::ResourceGroupManager::getSingleton().addResourceLocation( mediaPath + "models/pr2_new", "FileSystem", Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME );

    // Taken from gazebo
    Ogre::RenderSystemList *rsList = root_->getAvailableRenderers();

    Ogre::RenderSystem* renderSystem = NULL;
    Ogre::RenderSystemList::iterator renderIt = rsList->begin();
    Ogre::RenderSystemList::iterator renderEnd = rsList->end();
    for ( ; renderIt != renderEnd; ++renderIt )
    {
        renderSystem = *renderIt;

        if ( renderSystem->getName() == "OpenGL Rendering Subsystem" )
        {
            break;
        }
    }

    if ( renderSystem == NULL )
    {
      printf( "Could not find the opengl rendering subsystem!\n" );
      exit(1);
    }

    renderSystem->setConfigOption("Full Screen","No");
    renderSystem->setConfigOption("FSAA","2");

    root_->setRenderSystem( renderSystem );

    try
    {
        root_->initialise( false );
    }
    catch ( Ogre::Exception& e )
    {
        printf( "Failed to initialize Ogre::Root: %s\n", e.what() );
        exit(1);
    }

    visualization_panel_ = new VisualizationPanel( this, root_ );

    // TODO: figure out how we can move this back to before panel (renderwindow) creation (manually creating a GpuProgramManager?)
    try
    {
      Ogre::ResourceGroupManager::getSingleton().initialiseAllResourceGroups();
    }
    catch ( Ogre::Exception& e )
    {
      printf( "Error initializing resource groups: %s", e.what() );
      exit( 1 );
    }

    visualization_panel_->CreateVisualizer<GridVisualizer>( "Grid", true );
    visualization_panel_->CreateVisualizer<AxesVisualizer>( "Origin Axes", true );

    RobotModelVisualizer* model = visualization_panel_->CreateVisualizer<RobotModelVisualizer>( "Robot Model", false );
    model->Initialize( "robotdesc/pr2", "transform" );

    PointCloudVisualizer* pointCloud = visualization_panel_->CreateVisualizer<PointCloudVisualizer>( "Stereo Full Cloud", false );
    pointCloud->SetTopic( "videre/cloud" );
    pointCloud->SetColor( 1.0, 1.0, 1.0 );

    pointCloud = visualization_panel_->CreateVisualizer<PointCloudVisualizer>( "Head Full Cloud", false );
    pointCloud->SetTopic( "full_cloud" );
    pointCloud->SetColor( 1.0, 1.0, 0.0 );

    LaserScanVisualizer* laserScan = visualization_panel_->CreateVisualizer<LaserScanVisualizer>( "Head Scan", false );
    laserScan->SetScanTopic( "tilt_scan" );
    laserScan->SetColor( 1.0, 0.0, 0.0 );
    laserScan->SetDecayTime( 30.0f );

    laserScan = visualization_panel_->CreateVisualizer<LaserScanVisualizer>( "Floor Scan", false );
    laserScan->SetScanTopic( "base_scan" );
    laserScan->SetColor( 0.0f, 1.0f, 0.0f );
    laserScan->SetDecayTime( 0.0f );

    visualization_panel_->CreateVisualizer<MarkerVisualizer>( "Visualization Markers", true );
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
      localArgv[ i ] = strdup( wxString( argv[ i ] ).char_str() );

      printf( "argv[%d]: %s\n", i, localArgv[i] );
    }

    ros::init(argc, localArgv);

    wxFrame* frame = new MyFrame(NULL);
    SetTopWindow(frame);
    frame->Show();
    return true;
  }

  int OnExit()
  {
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
