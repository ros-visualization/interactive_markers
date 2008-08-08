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

#include "visualization_panel.h"
#include "visualizer_base.h"
#include "common.h"

#include "ogre_tools/wx_ogre_render_window.h"
#include "ogre_tools/fps_camera.h"
#include "ogre_tools/orbit_camera.h"

#include "ros/common.h"
#include "ros/node.h"
#include <rosTF/rosTF.h>

#include <Ogre.h>
#include <wx/timer.h>

namespace ogre_vis
{

namespace ViewTools
{
enum ViewTool
{
  FPS = wxID_HIGHEST + 1,
  Orbit,
};
}
typedef ViewTools::ViewTool ViewTool;

VisualizationPanel::VisualizationPanel( wxWindow* parent, Ogre::Root* root )
    : VisualizationPanelGenerated( parent )
    , m_OgreRoot( root )
    , m_CurrentOptionsPanel( NULL )
    , m_LeftMouseDown( false )
    , m_MiddleMouseDown( false )
    , m_RightMouseDown( false )
    , m_MouseX( 0 )
    , m_MouseY( 0 )
{
  InitializeCommon();

  static int count = 0;
  std::stringstream ss;
  ss << "VisualizationPanelNode" << count++;
  m_ROSNode = new ros::node( ss.str() );
  m_TFClient = new rosTFClient( *m_ROSNode );

  m_SceneManager = m_OgreRoot->createSceneManager( Ogre::ST_GENERIC );

  m_RenderPanel = new ogre_tools::wxOgreRenderWindow( m_OgreRoot, m_3DPanel );
  m_3DSizer->Add( m_RenderPanel, 1, wxALL|wxEXPAND, 0 );

  m_FPSCamera = new ogre_tools::FPSCamera( m_SceneManager );
  m_FPSCamera->GetOgreCamera()->setNearClipDistance( 0.1f );
  m_FPSCamera->SetPosition( 0, 1, 30 );

  m_OrbitCamera = new ogre_tools::OrbitCamera( m_SceneManager );
  m_OrbitCamera->GetOgreCamera()->setNearClipDistance( 0.1f );
  m_OrbitCamera->SetPosition( 0, 0, 30 );

  m_CurrentCamera = m_OrbitCamera;

  m_RenderPanel->GetViewport()->setCamera( m_CurrentCamera->GetOgreCamera() );

  m_Views->AddRadioTool( ViewTools::Orbit, wxT("Orbit"), wxNullBitmap, wxNullBitmap, wxT("Orbit Camera Controls") );
  m_Views->AddRadioTool( ViewTools::FPS, wxT("FPS"), wxNullBitmap, wxNullBitmap, wxT("FPS Camera Controls") );

  m_Views->Connect( wxEVT_COMMAND_TOOL_CLICKED, wxCommandEventHandler( VisualizationPanel::OnViewClicked ), NULL, this );

  m_Displays->Connect( wxEVT_COMMAND_CHECKLISTBOX_TOGGLED, wxCommandEventHandler( VisualizationPanel::OnDisplayToggled ), NULL, this );
  m_Displays->Connect( wxEVT_COMMAND_LISTBOX_SELECTED, wxCommandEventHandler( VisualizationPanel::OnDisplaySelected ), NULL, this );

  m_RenderPanel->Connect( wxEVT_LEFT_DOWN, wxMouseEventHandler( VisualizationPanel::OnRenderWindowMouseEvents ), NULL, this );
  m_RenderPanel->Connect( wxEVT_MIDDLE_DOWN, wxMouseEventHandler( VisualizationPanel::OnRenderWindowMouseEvents ), NULL, this );
  m_RenderPanel->Connect( wxEVT_RIGHT_DOWN, wxMouseEventHandler( VisualizationPanel::OnRenderWindowMouseEvents ), NULL, this );
  m_RenderPanel->Connect( wxEVT_MOTION, wxMouseEventHandler( VisualizationPanel::OnRenderWindowMouseEvents ), NULL, this );
  m_RenderPanel->Connect( wxEVT_LEFT_UP, wxMouseEventHandler( VisualizationPanel::OnRenderWindowMouseEvents ), NULL, this );
  m_RenderPanel->Connect( wxEVT_MIDDLE_UP, wxMouseEventHandler( VisualizationPanel::OnRenderWindowMouseEvents ), NULL, this );
  m_RenderPanel->Connect( wxEVT_RIGHT_UP, wxMouseEventHandler( VisualizationPanel::OnRenderWindowMouseEvents ), NULL, this );

  // HACK HACK HACK until single threaded ROS
  m_RenderPanel->SetPreRenderCallback( new functor<VisualizationPanel>( this, &VisualizationPanel::LockRender ) );
  m_RenderPanel->SetPostRenderCallback( new functor<VisualizationPanel>( this, &VisualizationPanel::UnlockRender ) );

  m_UpdateTimer = new wxTimer( this );
  m_UpdateTimer->Start( 100 );
  m_UpdateStopwatch.Start();
  Connect( m_UpdateTimer->GetId(), wxEVT_TIMER, wxTimerEventHandler( VisualizationPanel::OnUpdate ), NULL, this );
}

VisualizationPanel::~VisualizationPanel()
{
  m_Views->Disconnect( wxEVT_COMMAND_TOOL_CLICKED, wxCommandEventHandler( VisualizationPanel::OnViewClicked ), NULL, this );

  m_Displays->Disconnect( wxEVT_COMMAND_CHECKLISTBOX_TOGGLED, wxCommandEventHandler( VisualizationPanel::OnDisplayToggled ), NULL, this );
  m_Displays->Disconnect( wxEVT_COMMAND_LISTBOX_SELECTED, wxCommandEventHandler( VisualizationPanel::OnDisplaySelected ), NULL, this );

  m_RenderPanel->Disconnect( wxEVT_LEFT_DOWN, wxMouseEventHandler( VisualizationPanel::OnRenderWindowMouseEvents ), NULL, this );
  m_RenderPanel->Disconnect( wxEVT_MIDDLE_DOWN, wxMouseEventHandler( VisualizationPanel::OnRenderWindowMouseEvents ), NULL, this );
  m_RenderPanel->Disconnect( wxEVT_RIGHT_DOWN, wxMouseEventHandler( VisualizationPanel::OnRenderWindowMouseEvents ), NULL, this );
  m_RenderPanel->Disconnect( wxEVT_MOTION, wxMouseEventHandler( VisualizationPanel::OnRenderWindowMouseEvents ), NULL, this );
  m_RenderPanel->Disconnect( wxEVT_LEFT_UP, wxMouseEventHandler( VisualizationPanel::OnRenderWindowMouseEvents ), NULL, this );
  m_RenderPanel->Disconnect( wxEVT_MIDDLE_UP, wxMouseEventHandler( VisualizationPanel::OnRenderWindowMouseEvents ), NULL, this );
  m_RenderPanel->Disconnect( wxEVT_RIGHT_UP, wxMouseEventHandler( VisualizationPanel::OnRenderWindowMouseEvents ), NULL, this );

  Disconnect( wxEVT_TIMER, m_UpdateTimer->GetId(), wxTimerEventHandler( VisualizationPanel::OnUpdate ), NULL, this );
  delete m_UpdateTimer;

  V_Visualizer::iterator visIt = m_Visualizers.begin();
  V_Visualizer::iterator visEnd = m_Visualizers.end();
  for ( ; visIt != visEnd; ++visIt )
  {
    VisualizerBase* visualizer = *visIt;
    delete visualizer;
  }
  m_Visualizers.clear();

  m_RenderPanel->Destroy();

  if ( m_CurrentOptionsPanel )
  {
    m_CurrentOptionsPanel->Destroy();
  }

  delete m_FPSCamera;
  delete m_OrbitCamera;

  m_OgreRoot->destroySceneManager( m_SceneManager );

  m_ROSNode->shutdown();
  delete m_ROSNode;
}

void VisualizationPanel::Render()
{
  m_RenderPanel->Refresh();
}

void VisualizationPanel::OnViewClicked( wxCommandEvent& event )
{
  switch ( event.GetId() )
  {
  case ViewTools::FPS:
    {
      m_CurrentCamera = m_FPSCamera;
    }
    break;

  case ViewTools::Orbit:
    {
      m_CurrentCamera = m_OrbitCamera;
    }
    break;
  }

  m_RenderPanel->SetCamera( m_CurrentCamera->GetOgreCamera() );
}

void VisualizationPanel::OnDisplayToggled( wxCommandEvent& event )
{
  int selectionIndex = event.GetSelection();

  VisualizerBase* visualizer = (VisualizerBase*)m_Displays->GetClientData( selectionIndex );

  if ( m_Displays->IsChecked( selectionIndex ) )
  {
    visualizer->Enable();
  }
  else
  {
    visualizer->Disable();
  }

  Render();
}

void VisualizationPanel::OnDisplaySelected( wxCommandEvent& event )
{
  int selectionIndex = event.GetSelection();

  VisualizerBase* visualizer = (VisualizerBase*)m_Displays->GetClientData( selectionIndex );

  if ( m_CurrentOptionsPanel )
  {
    m_PropertiesPanelSizer->Remove( m_CurrentOptionsPanel );
    m_CurrentOptionsPanel->Destroy();
  }

  m_CurrentOptionsPanel = visualizer->GetOptionsPanel( m_PropertiesPanel );

  if ( m_CurrentOptionsPanel )
  {
    m_CurrentOptionsPanel->SetSize( m_PropertiesPanel->GetSize() );
    m_PropertiesPanelSizer->Add( m_CurrentOptionsPanel, 1, wxEXPAND, 5 );
  }

  m_RenderPanel->Refresh();
}

void VisualizationPanel::OnUpdate( wxTimerEvent& event )
{
  long millis = m_UpdateStopwatch.Time();
  float dt = millis / 1000.0f;

  m_UpdateStopwatch.Start();

  V_Visualizer::iterator visIt = m_Visualizers.begin();
  V_Visualizer::iterator visEnd = m_Visualizers.end();
  for ( ; visIt != visEnd; ++visIt )
  {
    VisualizerBase* visualizer = *visIt;

    if ( visualizer->IsEnabled() )
    {
      visualizer->Update( dt );
    }
  }
}

void VisualizationPanel::AddVisualizer( VisualizerBase* visualizer )
{
  m_Visualizers.push_back( visualizer );

  m_Displays->Append( wxString::FromAscii( visualizer->GetName().c_str() ), visualizer );
  m_Displays->Check( m_Displays->GetCount() - 1, visualizer->IsEnabled() );

  visualizer->SetRenderCallback( new functor<VisualizationPanel>( this, &VisualizationPanel::Render ) );
  visualizer->SetLockRenderCallback( new functor<VisualizationPanel>( this, &VisualizationPanel::LockRender ) );
  visualizer->SetUnlockRenderCallback( new functor<VisualizationPanel>( this, &VisualizationPanel::UnlockRender ) );
}

void VisualizationPanel::OnRenderWindowMouseEvents( wxMouseEvent& event )
{
  int lastX = m_MouseX;
  int lastY = m_MouseY;

  m_MouseX = event.GetX();
  m_MouseY = event.GetY();

  if ( event.LeftDown() )
  {
    m_LeftMouseDown = true;
    m_MiddleMouseDown = false;
    m_RightMouseDown = false;
  }
  else if ( event.MiddleDown() )
  {
    m_LeftMouseDown = false;
    m_MiddleMouseDown = true;
    m_RightMouseDown = false;
  }
  else if ( event.RightDown() )
  {
    m_LeftMouseDown = false;
    m_MiddleMouseDown = false;
    m_RightMouseDown = true;
  }
  else if ( event.LeftUp() )
  {
    m_LeftMouseDown = false;
  }
  else if ( event.MiddleUp() )
  {
    m_MiddleMouseDown = false;
  }
  else if ( event.RightUp() )
  {
    m_RightMouseDown = false;
  }
  else if ( event.Dragging() )
  {
    int32_t diffX = m_MouseX - lastX;
    int32_t diffY = m_MouseY - lastY;

    bool handled = false;
    if ( m_LeftMouseDown )
    {
      m_CurrentCamera->MouseLeftDrag( diffX, diffY );

      handled = true;
    }
    else if ( m_MiddleMouseDown )
    {
      m_CurrentCamera->MouseMiddleDrag( diffX, diffY );

      handled = true;
    }
    else if ( m_RightMouseDown )
    {
      m_CurrentCamera->MouseRightDrag( diffX, diffY );

      handled = true;
    }

    if ( handled )
    {
      Render();
    }
  }
}

} // namespace ogre_vis
