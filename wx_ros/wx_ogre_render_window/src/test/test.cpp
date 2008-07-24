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

#include "../wx_ogre_render_window/wx_ogre_render_window.h"

#include "Ogre.h"

class MyFrame : public wxFrame 
{
public:
  MyFrame(wxWindow* parent) : wxFrame(parent, -1, _("Ogre Render Window Test App"),
                      wxDefaultPosition, wxSize(800,600),
                      wxDEFAULT_FRAME_STYLE)                              
  {
    m_Root = new Ogre::Root();
    m_Root->loadPlugin( "RenderSystem_GL" );
    m_Root->loadPlugin( "Plugin_OctreeSceneManager" );
    
    Ogre::ResourceGroupManager::getSingleton().addResourceLocation( "Media", "FileSystem", "General");
    Ogre::ResourceGroupManager::getSingleton().addResourceLocation( "Media/models", "FileSystem", "General");
    Ogre::ResourceGroupManager::getSingleton().addResourceLocation( "Media/materials/scripts", "FileSystem", "General");
    Ogre::ResourceGroupManager::getSingleton().addResourceLocation( "Media/materials/programs", "FileSystem", "General");
    Ogre::ResourceGroupManager::getSingleton().addResourceLocation( "Media/materials/textures", "FileSystem", "General");
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
        
        m_WXRenderWindow = new wxOgreRenderWindow( m_Root, this );
        m_WXRenderWindow->SetSize( this->GetSize() );
        
        m_Camera = m_SceneManager->createCamera( "TestCam" );
        m_Camera->setPosition( Ogre::Vector3( 0, 0, 300 ) );
        m_Camera->lookAt( Ogre::Vector3( 0, 0, 0 ) );
        m_Camera->setNearClipDistance( 1 );
        
        Ogre::RenderWindow* ogreRenderWindow = m_WXRenderWindow->GetRenderWindow();
        Ogre::Viewport* viewport = ogreRenderWindow->addViewport( m_Camera );
    }
    catch ( Ogre::Exception& e )
    {
        printf( "Error initializing render window/viewport: %s\n", e.what() );
        exit(1);
    }
    
    m_RenderTimer = new wxTimer( this );
    m_RenderTimer->Start( 10 );
    
    Connect( m_RenderTimer->GetId(), wxEVT_TIMER, wxTimerEventHandler( MyFrame::RenderCallback ), NULL, this );
  }
  
  ~MyFrame()
  {
    Disconnect( m_RenderTimer->GetId(), wxEVT_TIMER, wxTimerEventHandler( MyFrame::RenderCallback ), NULL, this );
    
    delete m_RenderTimer;
    
    m_WXRenderWindow->Destroy();
    delete m_Root;
  }
  
private:
  void RenderCallback( wxTimerEvent& event )
  {
    m_Root->renderOneFrame();
  }

  Ogre::Root* m_Root;
  Ogre::Camera* m_Camera;
  Ogre::SceneManager* m_SceneManager;
  
  wxOgreRenderWindow* m_WXRenderWindow;
  
  wxTimer* m_RenderTimer;
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
