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
#include "../ogre_tools/grid.h"
#include "../ogre_tools/fps_camera.h"
#include "../ogre_tools/orbit_camera.h"
#include "../ogre_tools/axes.h"
#include "../ogre_tools/cone.h"
#include "../ogre_tools/arrow.h"
#include "../ogre_tools/point_cloud.h"
#include "../ogre_tools/super_ellipsoid.h"
#include "../ogre_tools/billboard_line.h"
#include "../ogre_tools/initialization.h"

#include <OgreRoot.h>
#include <OgreSceneManager.h>
#include <OgreViewport.h>

class MyFrame : public wxFrame
{
public:
  MyFrame(wxWindow* parent)
  : wxFrame(parent, -1, _("Grid Test App"), wxDefaultPosition, wxSize(800,600), wxDEFAULT_FRAME_STYLE)
  , left_mouse_down_( false )
  , middle_mouse_down_( false )
  , right_mouse_down_( false )
  , mouse_x_( 0 )
  , mouse_y_( 0 )
  {
    ogre_tools::initializeOgre();
    ogre_tools::initializeResources( ogre_tools::V_string() );

    root_ = Ogre::Root::getSingletonPtr();

    try
    {
        scene_manager_ = root_->createSceneManager( Ogre::ST_GENERIC, "TestSceneManager" );

        m_WXRenderWindow = new ogre_tools::wxOgreRenderWindow( root_, this );
        m_WXRenderWindow->SetSize( this->GetSize() );

        camera_ = new ogre_tools::OrbitCamera( scene_manager_ );
        camera_->setPosition( 0, 0, 15 );
        camera_->getOgreCamera()->setNearClipDistance( 0.1 );

        m_WXRenderWindow->getViewport()->setCamera( camera_->getOgreCamera() );

        ogre_tools::Grid* grid = new ogre_tools::Grid( scene_manager_, NULL, 10, 1.0f, 1.0f, 0.0f, 0.0f );
        //grid->getSceneNode()->pitch( Ogre::Degree( 90 ) );

        ogre_tools::BillboardLine* line = new ogre_tools::BillboardLine( scene_manager_, NULL );
        for ( int i = 0; i < 5; ++i )
        {
          line->addPoint( Ogre::Vector3( i, 0.0f, 0.0f ) );
        }

        for ( int i = 0; i < 5; ++i )
        {
          line->addPoint( Ogre::Vector3( 4.0f, 0.0f, i ) );
        }

        line->setLineWidth( 0.05 );
        line->setColor( 0.0f, 1.0f, 0.0f, 0.5f );

        //ogre_tools::Axes* axes = new ogre_tools::Axes( scene_manager_ );
        //axes->setScale( Ogre::Vector3( 2.0f, 2.0f, 2.0f ) );

        /*ogre_tools::Cone* cone = new ogre_tools::Cone( scene_manager_, NULL );
        cone->setScale( Ogre::Vector3( 0.3f, 2.0f, 0.3f ) );*/

        /*ogre_tools::Arrow* arrow = new ogre_tools::Arrow( scene_manager_ );
        arrow->SetHeadColor( 1.0f, 0.0f, 0.0f );
        arrow->SetShaftColor( 0.0f, 0.0f, 1.0f );
        arrow->setOrientation( Ogre::Quaternion::IDENTITY );*/
        //arrow->setOrientation( Ogre::Quaternion( Ogre::Degree( 45 ), Ogre::Vector3::UNIT_X ) );
        //arrow->setScale( Ogre::Vector3( 1.0f, 1.0f, 3.0f ) );

        /*ogre_tools::PointCloud* pointCloud = new ogre_tools::PointCloud( scene_manager_ );
        std::vector<ogre_tools::PointCloud::Point> points;
        points.resize( 1000000 );
        for ( int32_t i = 0; i < 1000; ++i )
        {
            for ( int32_t j = 0; j < 1000; ++j )
            {
                ogre_tools::PointCloud::Point& point = points[ 1000*j + i ];
                point.m_X = j / 10.0f;
                point.m_Y = i / 10.0f;
                point.m_Z = 0;

                point.r_ = abs(i) % 3 == 0 ? 1.0f : 0.0f;
                point.g_ = abs(i) % 3 == 1 ? 1.0f : 0.0f;
                point.b_ = abs(i) % 3 == 2 ? 1.0f : 0.0f;
            }
        }

        pointCloud->AddPoints( &points.front(), 1000000 );*/

        /*ogre_tools::SuperEllipsoid* se = new ogre_tools::SuperEllipsoid( scene_manager_ );
        se->create( ogre_tools::SuperEllipsoid::Cube, 60, Ogre::Vector3( 1.0f, 5.0f, 1.0f ) );*/
    }
    catch ( Ogre::Exception& e )
    {
        printf( "Fatal error: %s\n", e.what() );
        exit(1);
    }

    m_WXRenderWindow->Connect( wxEVT_LEFT_DOWN, wxMouseEventHandler( MyFrame::OnMouseEvents ), NULL, this );
    m_WXRenderWindow->Connect( wxEVT_MIDDLE_DOWN, wxMouseEventHandler( MyFrame::OnMouseEvents ), NULL, this );
    m_WXRenderWindow->Connect( wxEVT_RIGHT_DOWN, wxMouseEventHandler( MyFrame::OnMouseEvents ), NULL, this );
    m_WXRenderWindow->Connect( wxEVT_MOTION, wxMouseEventHandler( MyFrame::OnMouseEvents ), NULL, this );
    m_WXRenderWindow->Connect( wxEVT_LEFT_UP, wxMouseEventHandler( MyFrame::OnMouseEvents ), NULL, this );
    m_WXRenderWindow->Connect( wxEVT_MIDDLE_UP, wxMouseEventHandler( MyFrame::OnMouseEvents ), NULL, this );
    m_WXRenderWindow->Connect( wxEVT_RIGHT_UP, wxMouseEventHandler( MyFrame::OnMouseEvents ), NULL, this );
    m_WXRenderWindow->Connect( wxEVT_MOUSEWHEEL, wxMouseEventHandler( MyFrame::OnMouseEvents ), NULL, this );
  }

  ~MyFrame()
  {
    m_WXRenderWindow->Disconnect( wxEVT_LEFT_DOWN, wxMouseEventHandler( MyFrame::OnMouseEvents ), NULL, this );
    m_WXRenderWindow->Disconnect( wxEVT_MIDDLE_DOWN, wxMouseEventHandler( MyFrame::OnMouseEvents ), NULL, this );
    m_WXRenderWindow->Disconnect( wxEVT_RIGHT_DOWN, wxMouseEventHandler( MyFrame::OnMouseEvents ), NULL, this );
    m_WXRenderWindow->Disconnect( wxEVT_MOTION, wxMouseEventHandler( MyFrame::OnMouseEvents ), NULL, this );
    m_WXRenderWindow->Disconnect( wxEVT_LEFT_UP, wxMouseEventHandler( MyFrame::OnMouseEvents ), NULL, this );
    m_WXRenderWindow->Disconnect( wxEVT_MIDDLE_UP, wxMouseEventHandler( MyFrame::OnMouseEvents ), NULL, this );
    m_WXRenderWindow->Disconnect( wxEVT_RIGHT_UP, wxMouseEventHandler( MyFrame::OnMouseEvents ), NULL, this );
    m_WXRenderWindow->Disconnect( wxEVT_MOUSEWHEEL, wxMouseEventHandler( MyFrame::OnMouseEvents ), NULL, this );

    m_WXRenderWindow->Destroy();
    delete root_;
  }

private:

  void OnMouseEvents( wxMouseEvent& event )
  {
    int lastX = mouse_x_;
    int lastY = mouse_y_;

    mouse_x_ = event.GetX();
    mouse_y_ = event.GetY();

    if ( event.LeftDown() )
    {
      left_mouse_down_ = true;
      middle_mouse_down_ = false;
      right_mouse_down_ = false;
    }
    else if ( event.MiddleDown() )
    {
      left_mouse_down_ = false;
      middle_mouse_down_ = true;
      right_mouse_down_ = false;
    }
    else if ( event.RightDown() )
    {
      left_mouse_down_ = false;
      middle_mouse_down_ = false;
      right_mouse_down_ = true;
    }
    else if ( event.LeftUp() )
    {
      left_mouse_down_ = false;
    }
    else if ( event.MiddleUp() )
    {
      middle_mouse_down_ = false;
    }
    else if ( event.RightUp() )
    {
      right_mouse_down_ = false;
    }
    else if ( event.Dragging() )
    {
      int32_t diff_x = mouse_x_ - lastX;
      int32_t diff_y = mouse_y_ - lastY;

      bool handled = false;
      if ( left_mouse_down_ )
      {
        camera_->mouseLeftDrag( diff_x, diff_y );

        handled = true;
      }
      else if ( middle_mouse_down_ )
      {
        camera_->mouseMiddleDrag( diff_x, diff_y );

        handled = true;
      }
      else if ( right_mouse_down_ )
      {
        camera_->mouseRightDrag( diff_x, diff_y );

        handled = true;
      }

      if ( handled )
      {
        m_WXRenderWindow->Refresh();
      }
    }

    if ( event.GetWheelRotation() != 0 )
    {
      camera_->scrollWheel( event.GetWheelRotation() );

      m_WXRenderWindow->Refresh();
    }
  }

  Ogre::Root* root_;
  Ogre::SceneManager* scene_manager_;

  ogre_tools::wxOgreRenderWindow* m_WXRenderWindow;

  ogre_tools::Grid* grid_;
  ogre_tools::CameraBase* camera_;

  // Mouse handling
  bool left_mouse_down_;
  bool middle_mouse_down_;
  bool right_mouse_down_;
  int mouse_x_;
  int mouse_y_;
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
