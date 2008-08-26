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

#include <boost/bind.hpp>

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

  render_panel_ = new ogre_tools::wxOgreRenderWindow( ogre_root_, VisualizationPanelGenerated::render_panel_ );
  render_sizer_->Add( render_panel_, 1, wxALL|wxEXPAND, 0 );

  fps_camera_ = new ogre_tools::FPSCamera( scene_manager_ );
  fps_camera_->getOgreCamera()->setNearClipDistance( 0.1f );
  fps_camera_->setPosition( 0, 0, 15 );

  orbit_camera_ = new ogre_tools::OrbitCamera( scene_manager_ );
  orbit_camera_->getOgreCamera()->setNearClipDistance( 0.1f );
  orbit_camera_->setPosition( 0, 0, 15 );

  Ogre::Light* directional_light = scene_manager_->createLight( "MainDirectional" );
  directional_light->setType( Ogre::Light::LT_DIRECTIONAL );
  directional_light->setDirection( Ogre::Vector3( 0, -1, 1 ) );
  directional_light->setDiffuseColour( Ogre::ColourValue( 1.0f, 1.0f, 1.0f ) );

  current_camera_ = orbit_camera_;

  render_panel_->getViewport()->setCamera( current_camera_->getOgreCamera() );

  views_->AddRadioTool( IDs::Orbit, wxT("Orbit"), wxNullBitmap, wxNullBitmap, wxT("Orbit Camera Controls") );
  views_->AddRadioTool( IDs::FPS, wxT("FPS"), wxNullBitmap, wxNullBitmap, wxT("FPS Camera Controls") );

  views_->Connect( wxEVT_COMMAND_TOOL_CLICKED, wxCommandEventHandler( VisualizationPanel::onViewClicked ), NULL, this );

  displays_->Connect( wxEVT_COMMAND_CHECKLISTBOX_TOGGLED, wxCommandEventHandler( VisualizationPanel::onDisplayToggled ), NULL, this );
  displays_->Connect( wxEVT_COMMAND_LISTBOX_SELECTED, wxCommandEventHandler( VisualizationPanel::onDisplaySelected ), NULL, this );

  render_panel_->Connect( wxEVT_LEFT_DOWN, wxMouseEventHandler( VisualizationPanel::onRenderWindowMouseEvents ), NULL, this );
  render_panel_->Connect( wxEVT_MIDDLE_DOWN, wxMouseEventHandler( VisualizationPanel::onRenderWindowMouseEvents ), NULL, this );
  render_panel_->Connect( wxEVT_RIGHT_DOWN, wxMouseEventHandler( VisualizationPanel::onRenderWindowMouseEvents ), NULL, this );
  render_panel_->Connect( wxEVT_MOTION, wxMouseEventHandler( VisualizationPanel::onRenderWindowMouseEvents ), NULL, this );
  render_panel_->Connect( wxEVT_LEFT_UP, wxMouseEventHandler( VisualizationPanel::onRenderWindowMouseEvents ), NULL, this );
  render_panel_->Connect( wxEVT_MIDDLE_UP, wxMouseEventHandler( VisualizationPanel::onRenderWindowMouseEvents ), NULL, this );
  render_panel_->Connect( wxEVT_RIGHT_UP, wxMouseEventHandler( VisualizationPanel::onRenderWindowMouseEvents ), NULL, this );
  render_panel_->Connect( wxEVT_MOUSEWHEEL, wxMouseEventHandler( VisualizationPanel::onRenderWindowMouseEvents ), NULL, this );

  render_panel_->setPreRenderCallback( boost::bind( &VisualizationPanel::lockRender, this ) );
  render_panel_->setPostRenderCallback( boost::bind( &VisualizationPanel::unlockRender, this ) );

  update_timer_ = new wxTimer( this );
  update_timer_->Start( 10 );
  update_stopwatch_.Start();
  Connect( update_timer_->GetId(), wxEVT_TIMER, wxTimerEventHandler( VisualizationPanel::onUpdate ), NULL, this );

  Connect( EVT_RENDER, wxCommandEventHandler( VisualizationPanel::onRender ), NULL, this );

  property_grid_ = new wxPropertyGrid( properties_panel_, wxID_ANY, wxDefaultPosition, wxDefaultSize, wxPG_SPLITTER_AUTO_CENTER | wxTAB_TRAVERSAL | wxPG_DEFAULT_STYLE );
  properties_panel_sizer_->Add( property_grid_, 1, wxEXPAND, 5 );

  property_grid_->SetExtraStyle( wxPG_EX_HELP_AS_TOOLTIPS );

  property_grid_->Connect( wxEVT_PG_CHANGING, wxPropertyGridEventHandler( VisualizationPanel::onPropertyChanging ), NULL, this );
  property_grid_->Connect( wxEVT_PG_CHANGED, wxPropertyGridEventHandler( VisualizationPanel::onpropertyChanged ), NULL, this );
}

VisualizationPanel::~VisualizationPanel()
{
  views_->Disconnect( wxEVT_COMMAND_TOOL_CLICKED, wxCommandEventHandler( VisualizationPanel::onViewClicked ), NULL, this );

  displays_->Disconnect( wxEVT_COMMAND_CHECKLISTBOX_TOGGLED, wxCommandEventHandler( VisualizationPanel::onDisplayToggled ), NULL, this );
  displays_->Disconnect( wxEVT_COMMAND_LISTBOX_SELECTED, wxCommandEventHandler( VisualizationPanel::onDisplaySelected ), NULL, this );

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

  V_Visualizer::iterator vis_it = visualizers_.begin();
  V_Visualizer::iterator vis_end = visualizers_.end();
  for ( ; vis_it != vis_end; ++vis_it )
  {
    VisualizerBase* visualizer = *vis_it;
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
  ogre_tools::CameraBase* prev_camera = current_camera_;

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

  current_camera_->setFrom( prev_camera );

  render_panel_->setCamera( current_camera_->getOgreCamera() );
}

void VisualizationPanel::onDisplayToggled( wxCommandEvent& event )
{
  int selectionIndex = event.GetSelection();

  VisualizerBase* visualizer = (VisualizerBase*)displays_->GetClientData( selectionIndex );

  if ( displays_->IsChecked( selectionIndex ) )
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

  selected_visualizer_ = (VisualizerBase*)displays_->GetClientData( selectionIndex );

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

  V_Visualizer::iterator vis_it = visualizers_.begin();
  V_Visualizer::iterator vis_end = visualizers_.end();
  for ( ; vis_it != vis_end; ++vis_it )
  {
    VisualizerBase* visualizer = *vis_it;

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

  displays_->Append( wxString::FromAscii( visualizer->getName().c_str() ), visualizer );
  displays_->Check( displays_->GetCount() - 1, visualizer->isEnabled() );

  visualizer->setRenderCallback( boost::bind( &VisualizationPanel::render, this ) );
  visualizer->setLockRenderCallback( boost::bind( &VisualizationPanel::lockRender, this ) );
  visualizer->setUnlockRenderCallback( boost::bind( &VisualizationPanel::unlockRender, this ) );
}

void VisualizationPanel::onRenderWindowMouseEvents( wxMouseEvent& event )
{
  int last_x = mouse_x_;
  int last_y = mouse_y_;

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
    int32_t diff_x = mouse_x_ - last_x;
    int32_t diff_y = mouse_y_ - last_y;

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
