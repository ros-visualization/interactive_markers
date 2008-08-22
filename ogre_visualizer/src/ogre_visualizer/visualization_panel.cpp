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
#include <wx/propgrid/propgrid.h>

namespace ogre_vis
{

namespace IDs
{
enum ID
{
  FPS = wxID_HIGHEST + 1,
  Orbit,
};
}
typedef IDs::ID ID;

BEGIN_DECLARE_EVENT_TYPES()
DECLARE_EVENT_TYPE(EVT_RENDER, wxID_ANY)
END_DECLARE_EVENT_TYPES()

DEFINE_EVENT_TYPE(EVT_RENDER)

VisualizationPanel::VisualizationPanel( wxWindow* parent, Ogre::Root* root )
    : VisualizationPanelGenerated( parent )
    , ogre_root_( root )
    , selected_visualizer_( NULL )
    , left_mouse_down_( false )
    , middle_mouse_down_( false )
    , right_mouse_down_( false )
    , mouse_x_( 0 )
    , mouse_y_( 0 )
{
  initializeCommon();

  static int count = 0;
  std::stringstream ss;
  ss << "VisualizationPanelNode" << count++;
  ros_node_ = new ros::node( ss.str() );
  tf_client_ = new rosTFClient( *ros_node_ );

  scene_manager_ = ogre_root_->createSceneManager( Ogre::ST_GENERIC );

  render_panel_ = new ogre_tools::wxOgreRenderWindow( ogre_root_, m_3DPanel );
  m_3DSizer->Add( render_panel_, 1, wxALL|wxEXPAND, 0 );

  fps_camera_ = new ogre_tools::FPSCamera( scene_manager_ );
  fps_camera_->getOgreCamera()->setNearClipDistance( 0.1f );
  fps_camera_->setPosition( 0, 0, 15 );

  orbit_camera_ = new ogre_tools::OrbitCamera( scene_manager_ );
  orbit_camera_->getOgreCamera()->setNearClipDistance( 0.1f );
  orbit_camera_->setPosition( 0, 0, 15 );

  Ogre::Light* directionalLight = scene_manager_->createLight( "MainDirectional" );
  directionalLight->setType( Ogre::Light::LT_DIRECTIONAL );
  directionalLight->setDirection( Ogre::Vector3( 0, -1, 1 ) );
  directionalLight->setDiffuseColour( Ogre::ColourValue( 1.0f, 1.0f, 1.0f ) );

  current_camera_ = orbit_camera_;

  render_panel_->getViewport()->setCamera( current_camera_->getOgreCamera() );

  m_Views->AddRadioTool( IDs::Orbit, wxT("Orbit"), wxNullBitmap, wxNullBitmap, wxT("Orbit Camera Controls") );
  m_Views->AddRadioTool( IDs::FPS, wxT("FPS"), wxNullBitmap, wxNullBitmap, wxT("FPS Camera Controls") );

  m_Views->Connect( wxEVT_COMMAND_TOOL_CLICKED, wxCommandEventHandler( VisualizationPanel::onViewClicked ), NULL, this );

  m_Displays->Connect( wxEVT_COMMAND_CHECKLISTBOX_TOGGLED, wxCommandEventHandler( VisualizationPanel::onDisplayToggled ), NULL, this );
  m_Displays->Connect( wxEVT_COMMAND_LISTBOX_SELECTED, wxCommandEventHandler( VisualizationPanel::onDisplaySelected ), NULL, this );

  render_panel_->Connect( wxEVT_LEFT_DOWN, wxMouseEventHandler( VisualizationPanel::onRenderWindowMouseEvents ), NULL, this );
  render_panel_->Connect( wxEVT_MIDDLE_DOWN, wxMouseEventHandler( VisualizationPanel::onRenderWindowMouseEvents ), NULL, this );
  render_panel_->Connect( wxEVT_RIGHT_DOWN, wxMouseEventHandler( VisualizationPanel::onRenderWindowMouseEvents ), NULL, this );
  render_panel_->Connect( wxEVT_MOTION, wxMouseEventHandler( VisualizationPanel::onRenderWindowMouseEvents ), NULL, this );
  render_panel_->Connect( wxEVT_LEFT_UP, wxMouseEventHandler( VisualizationPanel::onRenderWindowMouseEvents ), NULL, this );
  render_panel_->Connect( wxEVT_MIDDLE_UP, wxMouseEventHandler( VisualizationPanel::onRenderWindowMouseEvents ), NULL, this );
  render_panel_->Connect( wxEVT_RIGHT_UP, wxMouseEventHandler( VisualizationPanel::onRenderWindowMouseEvents ), NULL, this );
  render_panel_->Connect( wxEVT_MOUSEWHEEL, wxMouseEventHandler( VisualizationPanel::onRenderWindowMouseEvents ), NULL, this );

  render_panel_->setPreRenderCallback( new functor<VisualizationPanel>( this, &VisualizationPanel::lockRender ) );
  render_panel_->setPostRenderCallback( new functor<VisualizationPanel>( this, &VisualizationPanel::unlockRender ) );

  update_timer_ = new wxTimer( this );
  update_timer_->Start( 10 );
  update_stopwatch_.Start();
  Connect( update_timer_->GetId(), wxEVT_TIMER, wxTimerEventHandler( VisualizationPanel::onUpdate ), NULL, this );

  Connect( EVT_RENDER, wxCommandEventHandler( VisualizationPanel::onRender ), NULL, this );

  property_grid_ = new wxPropertyGrid( m_PropertiesPanel, wxID_ANY, wxDefaultPosition, wxDefaultSize, wxPG_SPLITTER_AUTO_CENTER | wxTAB_TRAVERSAL | wxPG_DEFAULT_STYLE );
  m_PropertiesPanelSizer->Add( property_grid_, 1, wxEXPAND, 5 );

  property_grid_->SetExtraStyle( wxPG_EX_HELP_AS_TOOLTIPS );

  property_grid_->Connect( wxEVT_PG_CHANGING, wxPropertyGridEventHandler( VisualizationPanel::onPropertyChanging ), NULL, this );
  property_grid_->Connect( wxEVT_PG_CHANGED, wxPropertyGridEventHandler( VisualizationPanel::onpropertyChanged ), NULL, this );
}

VisualizationPanel::~VisualizationPanel()
{
  m_Views->Disconnect( wxEVT_COMMAND_TOOL_CLICKED, wxCommandEventHandler( VisualizationPanel::onViewClicked ), NULL, this );

  m_Displays->Disconnect( wxEVT_COMMAND_CHECKLISTBOX_TOGGLED, wxCommandEventHandler( VisualizationPanel::onDisplayToggled ), NULL, this );
  m_Displays->Disconnect( wxEVT_COMMAND_LISTBOX_SELECTED, wxCommandEventHandler( VisualizationPanel::onDisplaySelected ), NULL, this );

  render_panel_->Disconnect( wxEVT_LEFT_DOWN, wxMouseEventHandler( VisualizationPanel::onRenderWindowMouseEvents ), NULL, this );
  render_panel_->Disconnect( wxEVT_MIDDLE_DOWN, wxMouseEventHandler( VisualizationPanel::onRenderWindowMouseEvents ), NULL, this );
  render_panel_->Disconnect( wxEVT_RIGHT_DOWN, wxMouseEventHandler( VisualizationPanel::onRenderWindowMouseEvents ), NULL, this );
  render_panel_->Disconnect( wxEVT_MOTION, wxMouseEventHandler( VisualizationPanel::onRenderWindowMouseEvents ), NULL, this );
  render_panel_->Disconnect( wxEVT_LEFT_UP, wxMouseEventHandler( VisualizationPanel::onRenderWindowMouseEvents ), NULL, this );
  render_panel_->Disconnect( wxEVT_MIDDLE_UP, wxMouseEventHandler( VisualizationPanel::onRenderWindowMouseEvents ), NULL, this );
  render_panel_->Disconnect( wxEVT_RIGHT_UP, wxMouseEventHandler( VisualizationPanel::onRenderWindowMouseEvents ), NULL, this );
  render_panel_->Disconnect( wxEVT_MOUSEWHEEL, wxMouseEventHandler( VisualizationPanel::onRenderWindowMouseEvents ), NULL, this );

  Disconnect( wxEVT_TIMER, update_timer_->GetId(), wxTimerEventHandler( VisualizationPanel::onUpdate ), NULL, this );
  delete update_timer_;

  Disconnect( EVT_RENDER, wxCommandEventHandler( VisualizationPanel::onRender ), NULL, this );

  property_grid_->Disconnect( wxEVT_PG_CHANGING, wxPropertyGridEventHandler( VisualizationPanel::onPropertyChanging ), NULL, this );
  property_grid_->Disconnect( wxEVT_PG_CHANGED, wxPropertyGridEventHandler( VisualizationPanel::onpropertyChanged ), NULL, this );
  property_grid_->Destroy();

  V_Visualizer::iterator visIt = visualizers_.begin();
  V_Visualizer::iterator visEnd = visualizers_.end();
  for ( ; visIt != visEnd; ++visIt )
  {
    VisualizerBase* visualizer = *visIt;
    delete visualizer;
  }
  visualizers_.clear();

  render_panel_->Destroy();

  delete fps_camera_;
  delete orbit_camera_;

  ogre_root_->destroySceneManager( scene_manager_ );

  ros_node_->shutdown();
  delete ros_node_;
}

void VisualizationPanel::render()
{
  wxCommandEvent event( EVT_RENDER, GetId() );
  wxPostEvent( this, event );
}

void VisualizationPanel::onRender( wxCommandEvent& event )
{
  render_panel_->Refresh();
}

void VisualizationPanel::onViewClicked( wxCommandEvent& event )
{
  ogre_tools::CameraBase* prevCamera = current_camera_;

  switch ( event.GetId() )
  {
  case IDs::FPS:
    {
      current_camera_ = fps_camera_;
    }
    break;

  case IDs::Orbit:
    {
      current_camera_ = orbit_camera_;
    }
    break;
  }

  current_camera_->setFrom( prevCamera );

  render_panel_->setCamera( current_camera_->getOgreCamera() );
}

void VisualizationPanel::onDisplayToggled( wxCommandEvent& event )
{
  int selectionIndex = event.GetSelection();

  VisualizerBase* visualizer = (VisualizerBase*)m_Displays->GetClientData( selectionIndex );

  if ( m_Displays->IsChecked( selectionIndex ) )
  {
    visualizer->enable();
  }
  else
  {
    visualizer->disable();
  }

  render();
}

void VisualizationPanel::onDisplaySelected( wxCommandEvent& event )
{
  int selectionIndex = event.GetSelection();

  selected_visualizer_ = (VisualizerBase*)m_Displays->GetClientData( selectionIndex );

  property_grid_->Freeze();
  property_grid_->Clear();
  selected_visualizer_->fillPropertyGrid( property_grid_ );
  property_grid_->Refresh();
  property_grid_->Thaw();

  render_panel_->Refresh();
}

void VisualizationPanel::onUpdate( wxTimerEvent& event )
{
  long millis = update_stopwatch_.Time();
  float dt = millis / 1000.0f;

  update_stopwatch_.Start();

  V_Visualizer::iterator visIt = visualizers_.begin();
  V_Visualizer::iterator visEnd = visualizers_.end();
  for ( ; visIt != visEnd; ++visIt )
  {
    VisualizerBase* visualizer = *visIt;

    if ( visualizer->isEnabled() )
    {
      visualizer->update( dt );
    }
  }
}

void VisualizationPanel::onPropertyChanging( wxPropertyGridEvent& event )
{
  if ( selected_visualizer_ )
  {
    wxPGProperty* property = event.GetProperty();

    if ( !property )
    {
      return;
    }

    selected_visualizer_->propertyChanging( event );
  }
}

void VisualizationPanel::onpropertyChanged( wxPropertyGridEvent& event )
{
  if ( selected_visualizer_ )
  {
    wxPGProperty* property = event.GetProperty();

    if ( !property )
    {
      return;
    }

    selected_visualizer_->propertyChanged( event );
  }
}

void VisualizationPanel::addVisualizer( VisualizerBase* visualizer )
{
  visualizers_.push_back( visualizer );

  m_Displays->Append( wxString::FromAscii( visualizer->getName().c_str() ), visualizer );
  m_Displays->Check( m_Displays->GetCount() - 1, visualizer->isEnabled() );

  visualizer->setRenderCallback( new functor<VisualizationPanel>( this, &VisualizationPanel::render ) );
  visualizer->setLockRenderCallback( new functor<VisualizationPanel>( this, &VisualizationPanel::lockRender ) );
  visualizer->setUnlockRenderCallback( new functor<VisualizationPanel>( this, &VisualizationPanel::unlockRender ) );
}

void VisualizationPanel::onRenderWindowMouseEvents( wxMouseEvent& event )
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
      current_camera_->mouseLeftDrag( diff_x, diff_y );

      handled = true;
    }
    else if ( middle_mouse_down_ )
    {
      current_camera_->mouseMiddleDrag( diff_x, diff_y );

      handled = true;
    }
    else if ( right_mouse_down_ )
    {
      current_camera_->mouseRightDrag( diff_x, diff_y );

      handled = true;
    }

    if ( handled )
    {
      render();
    }
  }

  if ( event.GetWheelRotation() != 0 )
  {
    current_camera_->scrollWheel( event.GetWheelRotation() );

    render();
  }
}

} // namespace ogre_vis
