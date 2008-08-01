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

#include "Ogre.h"

class MyFrame : public wxFrame 
{
public:
  MyFrame(wxWindow* parent) : wxFrame(parent, -1, _("Grid Test App"),
                      wxDefaultPosition, wxSize(800,600),
                      wxDEFAULT_FRAME_STYLE)    
  , m_Orient( false )
  , m_Translate( false )   
  , m_MouseX( 0 )
  , m_MouseY( 0 )                       
  {
    m_Root = new Ogre::Root();
    m_Root->loadPlugin( "RenderSystem_GL" );
    m_Root->loadPlugin( "Plugin_OctreeSceneManager" );
    
    Ogre::ResourceGroupManager::getSingleton().initialiseAllResourceGroups();
    
    // Taken from gazebo
    Ogre::RenderSystemList *rsList = m_Root->getAvailableRenderers();

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
    
    m_Root->setRenderSystem( renderSystem );
    
    try
    {
        m_Root->initialise( false );
    }
    catch ( Ogre::Exception& e )
    {
        printf( "Failed to initialize Ogre::Root: %s\n", e.what() );
        exit(1);   
    }
    
    try
    {
        m_SceneManager = m_Root->createSceneManager( Ogre::ST_GENERIC, "TestSceneManager" );
        
        m_WXRenderWindow = new ogre_tools::wxOgreRenderWindow( m_Root, this );
        m_WXRenderWindow->SetSize( this->GetSize() );
        
        m_Camera = new ogre_tools::OrbitCamera( m_SceneManager );
        m_Camera->SetPosition( 0, 0, 15 ); 
        m_Camera->GetOgreCamera()->setNearClipDistance( 1 );
        
        m_WXRenderWindow->GetViewport()->setCamera( m_Camera->GetOgreCamera() );
        
        m_Grid = new ogre_tools::Grid( m_SceneManager, 10, 1.0f, 1.0f, 0.0f, 0.0f );
        m_Grid->GetSceneNode()->pitch( Ogre::Degree( 90 ) );
    }
    catch ( Ogre::Exception& e )
    {
        printf( "Fatal error: %s\n", e.what() );
        exit(1);
    }
    
    m_WXRenderWindow->Connect( wxEVT_LEFT_DOWN, wxMouseEventHandler( MyFrame::OnMouseEvents ), NULL, this );
    m_WXRenderWindow->Connect( wxEVT_RIGHT_DOWN, wxMouseEventHandler( MyFrame::OnMouseEvents ), NULL, this );
    m_WXRenderWindow->Connect( wxEVT_MOTION, wxMouseEventHandler( MyFrame::OnMouseEvents ), NULL, this );
    m_WXRenderWindow->Connect( wxEVT_LEFT_UP, wxMouseEventHandler( MyFrame::OnMouseEvents ), NULL, this );
    m_WXRenderWindow->Connect( wxEVT_RIGHT_UP, wxMouseEventHandler( MyFrame::OnMouseEvents ), NULL, this );
  }
  
  ~MyFrame()
  {
    m_WXRenderWindow->Disconnect( wxEVT_LEFT_DOWN, wxMouseEventHandler( MyFrame::OnMouseEvents ), NULL, this );
    m_WXRenderWindow->Disconnect( wxEVT_RIGHT_DOWN, wxMouseEventHandler( MyFrame::OnMouseEvents ), NULL, this );
    m_WXRenderWindow->Disconnect( wxEVT_MOTION, wxMouseEventHandler( MyFrame::OnMouseEvents ), NULL, this );
    m_WXRenderWindow->Disconnect( wxEVT_LEFT_UP, wxMouseEventHandler( MyFrame::OnMouseEvents ), NULL, this );
    m_WXRenderWindow->Disconnect( wxEVT_RIGHT_UP, wxMouseEventHandler( MyFrame::OnMouseEvents ), NULL, this );
    
    delete m_Grid;
    
    delete m_RenderTimer;
    
    m_WXRenderWindow->Destroy();
    delete m_Root;
  }
  
private:
  
  void OnMouseEvents( wxMouseEvent& event )
  {
    int lastX = m_MouseX;
    int lastY = m_MouseY;
    
    m_MouseX = event.GetX();
    m_MouseY = event.GetY();
    
    if ( event.LeftDown() )
    {
      m_Orient = true;
      m_Translate = false;
    }
    else if ( event.RightDown() )
    {
      m_Orient = false;
      m_Translate = true;
    }
    else if ( event.LeftUp() && m_Orient )
    {
      m_Orient = false;
    }
    else if ( event.RightUp() && m_Translate )
    {
      m_Translate = false;
    }
    else if ( event.Dragging() )
    {
      bool handled = false;
      if ( m_Orient )
      {
        m_Camera->Yaw( -(m_MouseX - lastX)*0.005 );
        m_Camera->Pitch( -(m_MouseY - lastY)*0.005 );
        
        handled = true;
      } 
      else if ( m_Translate )
      {
        m_Camera->Move( 0.0f, 0.0f, (m_MouseY - lastY)*.1 );
        
        handled = true;
      }
      
      if ( handled )
      {
        m_WXRenderWindow->Refresh();
      }
    }
  }

  Ogre::Root* m_Root;
  Ogre::SceneManager* m_SceneManager;
  
  ogre_tools::wxOgreRenderWindow* m_WXRenderWindow;
  
  wxTimer* m_RenderTimer;
  
  ogre_tools::Grid* m_Grid;
  ogre_tools::CameraBase* m_Camera;
  
  bool m_Orient;
  bool m_Translate;
  int m_MouseX;
  int m_MouseY;
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
