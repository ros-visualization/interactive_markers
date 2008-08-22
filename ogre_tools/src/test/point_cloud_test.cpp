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

#include "../ogre_tools/wx_ogre_render_window.h"
#include "../ogre_tools/point_cloud.h"

#include "Ogre.h"

class MyFrame : public wxFrame
{
public:
  MyFrame(wxWindow* parent) : wxFrame(parent, -1, _("Point Cloud Test App"),
                      wxDefaultPosition, wxSize(800,600),
                      wxDEFAULT_FRAME_STYLE)
  {
    root_ = new Ogre::Root();
    root_->loadPlugin( "RenderSystem_GL" );
    root_->loadPlugin( "Plugin_OctreeSceneManager" );

    Ogre::ResourceGroupManager::getSingleton().initialiseAllResourceGroups();

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
    renderSystem->setConfigOption("RTT Preferred Mode", "PBuffer");

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

    try
    {
        scene_manager_ = root_->createSceneManager( Ogre::ST_GENERIC, "TestSceneManager" );

        m_WXRenderWindow = new ogre_tools::wxOgreRenderWindow( root_, this );
        m_WXRenderWindow->SetSize( this->GetSize() );

        camera_ = scene_manager_->createCamera( "TestCam" );
        camera_->setPosition( Ogre::Vector3( 0, 0, 30 ) );
        camera_->lookAt( Ogre::Vector3( 0, 0, 0 ) );
        camera_->setNearClipDistance( 1 );
        m_WXRenderWindow->GetViewport()->setCamera( camera_ );

        m_PointCloud = new ogre_tools::PointCloud( scene_manager_ );
        std::vector<ogre_tools::PointCloud::Point> points;
        points.resize( 100 );
        for ( int32_t i =-5; i < 5; ++i )
        {
            for ( int32_t j = -5; j < 5; ++j )
            {
                ogre_tools::PointCloud::Point& point = points[ 10*(i+5) + (j+5) ];
                point.m_X = j;
                point.m_Y = i;
                point.m_Z = 0;

                point.r_ = abs(i) % 3 == 0 ? 1.0f : 0.0f;
                point.g_ = abs(i) % 3 == 1 ? 1.0f : 0.0f;
                point.b_ = abs(i) % 3 == 2 ? 1.0f : 0.0f;
            }
        }

        m_PointCloud->AddPoints( &points.front(), 200 );
    }
    catch ( Ogre::Exception& e )
    {
        printf( "Fatal error: %s\n", e.what() );
        exit(1);
    }

    r_enderTimer = new wxTimer( this );
    r_enderTimer->Start( 100 );

    Connect( r_enderTimer->GetId(), wxEVT_TIMER, wxTimerEventHandler( MyFrame::RenderCallback ), NULL, this );
  }

  ~MyFrame()
  {
    Disconnect( r_enderTimer->GetId(), wxEVT_TIMER, wxTimerEventHandler( MyFrame::RenderCallback ), NULL, this );

    delete m_PointCloud;

    delete r_enderTimer;

    m_WXRenderWindow->Destroy();
    delete root_;
  }

private:
  void RenderCallback( wxTimerEvent& event )
  {
    root_->renderOneFrame();
  }

  Ogre::Root* root_;
  Ogre::Camera* camera_;
  Ogre::SceneManager* scene_manager_;

  ogre_tools::wxOgreRenderWindow* m_WXRenderWindow;

  wxTimer* r_enderTimer;

  ogre_tools::PointCloud* m_PointCloud;
};

// our normal wxApp-derived class, as usual
class MyApp : public wxApp
{
public:

  bool OnInit()
  {
    wxFrame* frame = new MyFrame(NULL);
    SetTopWindow(frame);
    frame->Show();
    return true;
  }

  int OnExit()
  {
      return 0;
  }
};

DECLARE_APP(MyApp);
IMPLEMENT_APP(MyApp);
