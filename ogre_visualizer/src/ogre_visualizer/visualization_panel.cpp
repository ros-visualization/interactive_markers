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
#include "visualization_manager.h"
#include "visualizer_base.h"
#include "new_display_dialog.h"
#include "properties/property.h"
#include "properties/property_manager.h"

#include "ogre_tools/wx_ogre_render_window.h"
#include "ogre_tools/fps_camera.h"
#include "ogre_tools/orbit_camera.h"
#include "ogre_tools/ortho_camera.h"

#include <wx/propgrid/propgrid.h>
#include <wx/confbase.h>

#include <boost/bind.hpp>

#include <OgreRoot.h>
#include <OgreViewport.h>

namespace ogre_vis
{

namespace IDs
{
enum ID
{
  FPS = wxID_HIGHEST + 1,
  Orbit,
  TopDownOrtho,
};
}
typedef IDs::ID ID;

BEGIN_DECLARE_EVENT_TYPES()
DECLARE_EVENT_TYPE(EVT_RENDER, wxID_ANY)
END_DECLARE_EVENT_TYPES()

DEFINE_EVENT_TYPE(EVT_RENDER)

VisualizationPanel::VisualizationPanel( wxWindow* parent )
: VisualizationPanelGenerated( parent )
, left_mouse_down_( false )
, middle_mouse_down_( false )
, right_mouse_down_( false )
, mouse_x_( 0 )
, mouse_y_( 0 )
, selected_visualizer_( NULL )
{
  render_panel_ = new ogre_tools::wxOgreRenderWindow( Ogre::Root::getSingletonPtr(), VisualizationPanelGenerated::render_panel_ );
  render_sizer_->Add( render_panel_, 1, wxALL|wxEXPAND, 0 );

  views_->AddRadioTool( IDs::Orbit, wxT("Orbit"), wxNullBitmap, wxNullBitmap, wxT("Orbit Camera Controls") );
  views_->AddRadioTool( IDs::FPS, wxT("FPS"), wxNullBitmap, wxNullBitmap, wxT("FPS Camera Controls") );
  views_->AddRadioTool( IDs::TopDownOrtho, wxT("Top-down Ortho"), wxNullBitmap, wxNullBitmap, wxT("Top-down Orthographic Camera") );

  views_->Connect( wxEVT_COMMAND_TOOL_CLICKED, wxCommandEventHandler( VisualizationPanel::onViewClicked ), NULL, this );

  render_panel_->Connect( wxEVT_LEFT_DOWN, wxMouseEventHandler( VisualizationPanel::onRenderWindowMouseEvents ), NULL, this );
  render_panel_->Connect( wxEVT_MIDDLE_DOWN, wxMouseEventHandler( VisualizationPanel::onRenderWindowMouseEvents ), NULL, this );
  render_panel_->Connect( wxEVT_RIGHT_DOWN, wxMouseEventHandler( VisualizationPanel::onRenderWindowMouseEvents ), NULL, this );
  render_panel_->Connect( wxEVT_MOTION, wxMouseEventHandler( VisualizationPanel::onRenderWindowMouseEvents ), NULL, this );
  render_panel_->Connect( wxEVT_LEFT_UP, wxMouseEventHandler( VisualizationPanel::onRenderWindowMouseEvents ), NULL, this );
  render_panel_->Connect( wxEVT_MIDDLE_UP, wxMouseEventHandler( VisualizationPanel::onRenderWindowMouseEvents ), NULL, this );
  render_panel_->Connect( wxEVT_RIGHT_UP, wxMouseEventHandler( VisualizationPanel::onRenderWindowMouseEvents ), NULL, this );
  render_panel_->Connect( wxEVT_MOUSEWHEEL, wxMouseEventHandler( VisualizationPanel::onRenderWindowMouseEvents ), NULL, this );
  render_panel_->Connect( wxEVT_LEFT_DCLICK, wxMouseEventHandler( VisualizationPanel::onRenderWindowMouseEvents ), NULL, this );

  render_panel_->setPreRenderCallback( boost::bind( &VisualizationPanel::lockRender, this ) );
  render_panel_->setPostRenderCallback( boost::bind( &VisualizationPanel::unlockRender, this ) );

  Connect( EVT_RENDER, wxCommandEventHandler( VisualizationPanel::onRender ), NULL, this );

  property_grid_ = new wxPropertyGrid( properties_panel_, wxID_ANY, wxDefaultPosition, wxDefaultSize, wxPG_SPLITTER_AUTO_CENTER | wxTAB_TRAVERSAL | wxPG_DEFAULT_STYLE );
  properties_panel_sizer_->Add( property_grid_, 1, wxEXPAND, 5 );

  property_grid_->SetExtraStyle( wxPG_EX_HELP_AS_TOOLTIPS );

  property_grid_->Connect( wxEVT_PG_CHANGING, wxPropertyGridEventHandler( VisualizationPanel::onPropertyChanging ), NULL, this );
  property_grid_->Connect( wxEVT_PG_CHANGED, wxPropertyGridEventHandler( VisualizationPanel::onPropertyChanged ), NULL, this );
  property_grid_->Connect( wxEVT_PG_SELECTED, wxPropertyGridEventHandler( VisualizationPanel::onPropertySelected ), NULL, this );

  property_grid_->SetCaptionBackgroundColour( wxColour( 2, 0, 174 ) );
  property_grid_->SetCaptionForegroundColour( *wxLIGHT_GREY );

  delete_display_->Enable( false );

  manager_ = new VisualizationManager( this );
  manager_->getVisualizerStateSignal().connect( boost::bind( &VisualizationPanel::onVisualizerStateChanged, this, _1 ) );

  fps_camera_ = new ogre_tools::FPSCamera( manager_->getSceneManager() );
  fps_camera_->getOgreCamera()->setNearClipDistance( 0.1f );
  fps_camera_->setPosition( 0, 0, 15 );
  fps_camera_->setRelativeNode( manager_->getTargetRelativeNode() );

  orbit_camera_ = new ogre_tools::OrbitCamera( manager_->getSceneManager() );
  orbit_camera_->getOgreCamera()->setNearClipDistance( 0.1f );
  orbit_camera_->setPosition( 0, 0, 15 );
  orbit_camera_->setRelativeNode( manager_->getTargetRelativeNode() );

  top_down_ortho_ = new ogre_tools::OrthoCamera( render_panel_, manager_->getSceneManager() );
  top_down_ortho_->setPosition( 0, 30, 0 );
  top_down_ortho_->pitch( -Ogre::Math::HALF_PI );
  top_down_ortho_->setRelativeNode( manager_->getTargetRelativeNode() );


  current_camera_ = orbit_camera_;

  render_panel_->getViewport()->setCamera( current_camera_->getOgreCamera() );
}

VisualizationPanel::~VisualizationPanel()
{
  delete fps_camera_;
  delete orbit_camera_;
  delete top_down_ortho_;

  delete manager_;

  views_->Disconnect( wxEVT_COMMAND_TOOL_CLICKED, wxCommandEventHandler( VisualizationPanel::onViewClicked ), NULL, this );

  render_panel_->Disconnect( wxEVT_LEFT_DOWN, wxMouseEventHandler( VisualizationPanel::onRenderWindowMouseEvents ), NULL, this );
  render_panel_->Disconnect( wxEVT_MIDDLE_DOWN, wxMouseEventHandler( VisualizationPanel::onRenderWindowMouseEvents ), NULL, this );
  render_panel_->Disconnect( wxEVT_RIGHT_DOWN, wxMouseEventHandler( VisualizationPanel::onRenderWindowMouseEvents ), NULL, this );
  render_panel_->Disconnect( wxEVT_MOTION, wxMouseEventHandler( VisualizationPanel::onRenderWindowMouseEvents ), NULL, this );
  render_panel_->Disconnect( wxEVT_LEFT_UP, wxMouseEventHandler( VisualizationPanel::onRenderWindowMouseEvents ), NULL, this );
  render_panel_->Disconnect( wxEVT_MIDDLE_UP, wxMouseEventHandler( VisualizationPanel::onRenderWindowMouseEvents ), NULL, this );
  render_panel_->Disconnect( wxEVT_RIGHT_UP, wxMouseEventHandler( VisualizationPanel::onRenderWindowMouseEvents ), NULL, this );
  render_panel_->Disconnect( wxEVT_MOUSEWHEEL, wxMouseEventHandler( VisualizationPanel::onRenderWindowMouseEvents ), NULL, this );
  render_panel_->Disconnect( wxEVT_LEFT_DCLICK, wxMouseEventHandler( VisualizationPanel::onRenderWindowMouseEvents ), NULL, this );

  Disconnect( EVT_RENDER, wxCommandEventHandler( VisualizationPanel::onRender ), NULL, this );

  property_grid_->Disconnect( wxEVT_PG_CHANGING, wxPropertyGridEventHandler( VisualizationPanel::onPropertyChanging ), NULL, this );
  property_grid_->Disconnect( wxEVT_PG_CHANGED, wxPropertyGridEventHandler( VisualizationPanel::onPropertyChanged ), NULL, this );
  property_grid_->Disconnect( wxEVT_PG_SELECTED, wxPropertyGridEventHandler( VisualizationPanel::onPropertySelected ), NULL, this );
  property_grid_->Destroy();

  render_panel_->Destroy();
}

void VisualizationPanel::queueRender()
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

  bool set_from_old = false;

  switch ( event.GetId() )
  {
  case IDs::FPS:
    {
      current_camera_ = fps_camera_;

      if ( current_camera_ == orbit_camera_ )
      {
        set_from_old = true;
      }
    }
    break;

  case IDs::Orbit:
    {
      current_camera_ = orbit_camera_;
      if ( current_camera_ == fps_camera_ )
      {
        set_from_old = true;
      }
    }
    break;

  case IDs::TopDownOrtho:
    {
      current_camera_ = top_down_ortho_;
    }
    break;
  }

  if ( set_from_old )
  {
    current_camera_->setFrom( prev_camera );
  }

  render_panel_->setCamera( current_camera_->getOgreCamera() );
}

void VisualizationPanel::onPropertyChanging( wxPropertyGridEvent& event )
{
  wxPGProperty* property = event.GetProperty();

  if ( !property )
  {
    return;
  }

  manager_->getPropertyManager()->propertyChanging( event );
}

void VisualizationPanel::onPropertyChanged( wxPropertyGridEvent& event )
{
  wxPGProperty* property = event.GetProperty();

  if ( !property )
  {
    return;
  }

  manager_->getPropertyManager()->propertyChanged( event );

  queueRender();
}

void VisualizationPanel::onPropertySelected( wxPropertyGridEvent& event )
{
  delete_display_->Enable( false );

  wxPGProperty* pg_property = event.GetProperty();

  if ( !pg_property )
  {
    return;
  }

  void* client_data = pg_property->GetClientData();
  if ( client_data )
  {
    PropertyBase* property = reinterpret_cast<PropertyBase*>(client_data);

    void* user_data = property->getUserData();
    if ( user_data )
    {
      VisualizerBase* visualizer = reinterpret_cast<VisualizerBase*>(user_data);

      if ( manager_->isValidVisualizer( visualizer ) )
      {
        selected_visualizer_ = visualizer;

        if ( manager_->isDeletionAllowed( visualizer ) )
        {
          delete_display_->Enable( true );
        }
      }
    }
  }
}

void VisualizationPanel::onRenderWindowMouseEvents( wxMouseEvent& event )
{
  int last_x = mouse_x_;
  int last_y = mouse_y_;

  mouse_x_ = event.GetX();
  mouse_y_ = event.GetY();

  if ( event.LeftDown() )
  {
    if ( event.ControlDown() )
    {
      manager_->pick( mouse_x_, mouse_y_ );
    }
    else
    {
      left_mouse_down_ = true;
      middle_mouse_down_ = false;
      right_mouse_down_ = false;

      current_camera_->mouseLeftDown( mouse_x_, mouse_y_ );
    }
  }
  else if ( event.MiddleDown() )
  {
    left_mouse_down_ = false;
    middle_mouse_down_ = true;
    right_mouse_down_ = false;

    current_camera_->mouseMiddleDown( mouse_x_, mouse_y_ );
  }
  else if ( event.RightDown() )
  {
    left_mouse_down_ = false;
    middle_mouse_down_ = false;
    right_mouse_down_ = true;

    current_camera_->mouseRightDown( mouse_x_, mouse_y_ );
  }
  else if ( event.LeftUp() )
  {
    left_mouse_down_ = false;

    current_camera_->mouseLeftUp( mouse_x_, mouse_y_ );
  }
  else if ( event.MiddleUp() )
  {
    middle_mouse_down_ = false;

    current_camera_->mouseMiddleUp( mouse_x_, mouse_y_ );
  }
  else if ( event.RightUp() )
  {
    right_mouse_down_ = false;

    current_camera_->mouseRightUp( mouse_x_, mouse_y_ );
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
      queueRender();
    }
  }

  if ( event.GetWheelRotation() != 0 )
  {
    current_camera_->scrollWheel( event.GetWheelRotation() );

    queueRender();
  }
}

void VisualizationPanel::onNewDisplay( wxCommandEvent& event )
{
  std::vector<std::string> types;
  manager_->getRegisteredTypes( types );

  NewDisplayDialog dialog( this, types );
  while (1)
  {
    if ( dialog.ShowModal() == wxOK )
    {
      std::string type = dialog.getTypeName();
      std::string name = dialog.getVisualizerName();

      if ( manager_->getVisualizer( name ) != NULL )
      {
        wxMessageBox( wxT("A visualizer with that name already exists!"), wxT("Invalid name"), wxICON_ERROR | wxOK, this );
        continue;
      }

      VisualizerBase* visualizer = manager_->createVisualizer( type, name, true, true );
      ROS_ASSERT(visualizer);
      (void)visualizer;

      break;
    }
    else
    {
      break;
    }
  }
}

void VisualizationPanel::onDeleteDisplay( wxCommandEvent& event )
{
  if ( !selected_visualizer_ )
  {
    return;
  }

  manager_->removeVisualizer( selected_visualizer_ );
  selected_visualizer_ = NULL;
}

void VisualizationPanel::onVisualizerStateChanged( VisualizerBase* visualizer )
{
  wxPGProperty* property = property_grid_->GetProperty( wxString::FromAscii( visualizer->getName().c_str() ) );
  ROS_ASSERT( property );

  wxPGCell* cell = new wxPGCell( wxString::FromAscii( visualizer->getName().c_str() ), wxNullBitmap, *wxLIGHT_GREY, *wxGREEN );
  property->SetCell( 0, cell );
  ROS_ASSERT( cell );

  if ( visualizer->isEnabled() )
  {
    cell->SetBgCol( wxColour( 32, 116, 38 ) );
  }
  else
  {
    cell->SetBgCol( wxColour( 151, 24, 41 ) );
  }
}

void VisualizationPanel::loadConfig( wxConfigBase* config )
{
  manager_->loadConfig( config );
}

void VisualizationPanel::saveConfig( wxConfigBase* config )
{
  manager_->saveConfig( config );
}

} // namespace ogre_vis
