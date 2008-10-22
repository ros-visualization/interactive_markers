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


#include "CameraPanel.h"
#include "CameraSetupDialog.h"

// ROS includes
#include "ros/node.h"

// wx includes
#include <wx/image.h>
#include <wx/mstream.h>
#include <wx/dcclient.h>

// Standard includes
#include <sstream>

// jfaust TODO: some of these values should be read from a config somewhere (min/max mostly)
#define PAN_SCALE (0.1f)

#define TILT_SCALE (0.1f)

#define ZOOM_SCALE (10.0f)

#define POSITION_TICK_LENGTH (20)
#define POSITION_TICK_WIDTH (5)

#define BAR_WINDOW_PROPORTION (0.9f)
#define BAR_WIDTH (10)

#define TARGET_PAN_EPSILON (1.0f)
#define TARGET_TILT_EPSILON (1.0f)
#define TARGET_ZOOM_EPSILON (10.0f)

#define SCROLL_WAIT_INTERVAL ( 300 )
#define ZOOM_SCROLL_STEPS (20.0f)

BEGIN_DECLARE_EVENT_TYPES()
DECLARE_EVENT_TYPE(EVT_FAKE_REFRESH, wxID_ANY)
END_DECLARE_EVENT_TYPES()

DEFINE_EVENT_TYPE(EVT_FAKE_REFRESH)

CameraPanel::CameraPanel(wxWindow* parent)
: CameraPanelBase( parent )
, enabled_( false )
, ptz_enabled_( false )
, image_data_( NULL )
, image_( NULL )
, image_codec_( &image_message_ )
, recreate_bitmap_( false )
, current_pan_( 0.0f )
, current_tilt_( 0.0f )
, current_zoom_( 0.0f )
, has_pan_target_( false )
, pan_target_( 0.0f )
, has_tilt_target_( false )
, tilt_target_( 0.0f )
, has_zoom_target_( false )
, zoom_target_( 0.0f )
, left_mouse_down_( false )
, right_mouse_down_( false )
, start_mouse_x_( 0 )
, start_mouse_y_( 0 )
, current_mouse_x_( 0 )
, current_mouse_y_( 0 )
, pan_min_( -169.0f )
, pan_max_( 169.0f )
, tilt_min_( -10.0f )
, tilt_max_( 90.0f )
, zoom_min_( 1.0f )
, zoom_max_( 9999.0f )
, zoom_scroll_timer_( this )
, minimal_( false )
{
  wxInitAllImageHandlers();

  Connect( wxEVT_TIMER, wxTimerEventHandler(CameraPanel::onScrollComplete), NULL, this);

  ros_node_ = ros::node::instance();

  /// @todo This should go away once creation of the ros::node is more well-defined
  if (!ros_node_)
  {
    int argc = 0;
    ros::init( argc, 0 );
    ros_node_ = new ros::node( "CameraPanel", ros::node::DONT_HANDLE_SIGINT );
  }
  ROS_ASSERT( ros_node_ );

  image_panel_->Connect( EVT_FAKE_REFRESH, wxCommandEventHandler( CameraPanel::onFakeRefresh ), NULL, this );
}

CameraPanel::~CameraPanel()
{
  image_panel_->Disconnect( EVT_FAKE_REFRESH, wxCommandEventHandler( CameraPanel::onFakeRefresh ), NULL, this );

  setEnabled( false );

  delete [] image_data_;
  delete image_;
}

void CameraPanel::setName( const std::string& name )
{
  name_ = name;

  setImageSubscription( name + "/image" );
  setPTZStateSubscription( name + "/ptz_state" );
  setPTZControlCommand( name + "/ptz_cmd" );
}

void CameraPanel::setImageSubscription( const std::string& subscription )
{
  if ( image_topic_ == subscription )
  {
    return;
  }

  // if we already have a subscription, unsubscribe
  if ( !image_topic_.empty() )
  {
    unsubscribeImage();
  }

  image_topic_ = subscription;

  if ( !image_topic_.empty() )
  {
    subscribeImage();
  }
}

void CameraPanel::setPTZStateSubscription( const std::string& subscription )
{
  if ( ptz_state_topic_ == subscription )
  {
    return;
  }

  // if we already have a subscription, unsubscribe
  if ( !ptz_state_topic_.empty() )
  {
    unsubscribePTZState();
  }

  ptz_state_topic_ = subscription;

  if ( !ptz_state_topic_.empty() )
  {
    subscribePTZState();
  }
}

void CameraPanel::setPTZControlCommand( const std::string& command )
{
  if ( ptz_control_topic_ == command )
  {
    return;
  }

  // if we already have a command we're advertising, stop
  if ( !ptz_control_topic_.empty() )
  {
    stopPTZControl();
  }

  ptz_control_topic_ = command;

  if ( !ptz_control_topic_.empty() )
  {
    advertisePTZControl();
  }
}

void CameraPanel::setEnabled( bool enabled )
{
  if ( enabled_ == enabled )
  {
    return;
  }

  if ( enabled )
  {
    enabled_ = true;
    startAll();
  }
  else
  {
    stopAll();
    enabled_ = false;
  }

  enable_->SetValue( enabled );
}

void CameraPanel::setPTZEnabled( bool enabled )
{
  if ( ptz_enabled_ == enabled )
  {
    return;
  }

  ptz_enabled_ = enabled;

  if ( enabled )
  {
    subscribePTZState();
    advertisePTZControl();
  }
  else
  {
    unsubscribePTZState();
    stopPTZControl();
  }
}

void CameraPanel::setPanLimits( float min, float max )
{
  pan_min_ = min;
  pan_max_ = max;
}

void CameraPanel::setTiltLimits( float min, float max )
{
  tilt_min_ = min;
  tilt_max_ = max;
}

void CameraPanel::setZoomLimits( float min, float max )
{
  zoom_min_ = min;
  zoom_max_ = max;
}

void CameraPanel::setMinimal( bool minimal )
{
  if ( minimal )
  {
    setup_->Hide();
    enable_->Hide();
  }
  else
  {
    setup_->Show();
    enable_->Show();
  }

  Layout();
  Fit();
}

void CameraPanel::startAll()
{
  subscribeImage();
  subscribePTZState();
  advertisePTZControl();
}

void CameraPanel::stopAll()
{
  unsubscribeImage();
  unsubscribePTZState();
  stopPTZControl();
}

void CameraPanel::subscribeImage()
{
  if ( isImageEnabled() )
  {
    ros_node_->subscribe( image_topic_, image_message_, &CameraPanel::incomingImage, this, 1 );
  }
}

void CameraPanel::unsubscribeImage()
{
  ros_node_->unsubscribe( image_topic_, &CameraPanel::incomingImage, this );
}

void CameraPanel::subscribePTZState()
{
  if ( isPTZStateEnabled() )
  {
    ros_node_->subscribe( ptz_state_topic_, ptz_state_message_, &CameraPanel::incomingPTZState, this, 0 );
  }
}

void CameraPanel::unsubscribePTZState()
{
  ros_node_->unsubscribe( ptz_state_topic_, &CameraPanel::incomingPTZState, this );
}

void CameraPanel::advertisePTZControl()
{
  if ( isPTZControlEnabled() )
  {
    ros_node_->advertise<axis_cam::PTZActuatorCmd>(ptz_control_topic_, 0);
  }
}

void CameraPanel::stopPTZControl()
{
  ros_node_->unadvertise(ptz_control_topic_);
}

void CameraPanel::incomingPTZState()
{
  if ( ptz_state_message_.pan.pos_valid )
  {
    current_pan_ = ptz_state_message_.pan.pos;
  }

  if ( ptz_state_message_.tilt.pos_valid )
  {
    current_tilt_ = ptz_state_message_.tilt.pos;
  }

  if ( ptz_state_message_.zoom.pos_valid )
  {
    current_zoom_ = ptz_state_message_.zoom.pos;
  }

  if ( has_pan_target_ && abs( current_pan_ - pan_target_ ) < TARGET_PAN_EPSILON )
  {
    has_pan_target_ = false;
  }

  if ( has_tilt_target_ && abs( current_tilt_ - tilt_target_ ) < TARGET_TILT_EPSILON )
  {
    has_tilt_target_ = false;
  }

  if ( has_zoom_target_ && abs( current_zoom_ - zoom_target_ ) < TARGET_ZOOM_EPSILON )
  {
    has_zoom_target_ = false;
  }

  // wx really doesn't like a Refresh call coming from a separate thread
  // send a fake refresh event, so that the call to Refresh comes from the main thread
  wxCommandEvent evt( EVT_FAKE_REFRESH, image_panel_->GetId() );
  wxPostEvent( image_panel_, evt );
}

void CameraPanel::incomingImage()
{
  image_mutex_.lock();

  // if this image is raw, compress it as jpeg since wx doesn't support the raw format
  if (image_message_.compression == "raw")
  {
    image_codec_.inflate();
    image_message_.compression = "jpeg";

    if (!image_codec_.deflate(100))
      return;
  }


  delete [] image_data_;
  const uint32_t dataSize = image_message_.get_data_size();
  image_data_ = new uint8_t[ dataSize ];
  memcpy( image_data_, &image_message_.data[0], dataSize );

  delete image_;
  wxMemoryInputStream memoryStream( image_data_, dataSize );
  image_ = new wxImage( memoryStream, wxBITMAP_TYPE_ANY, -1 );

  recreate_bitmap_ = true;

  // wx really doesn't like a Refresh call coming from a separate thread
  // send a fake refresh event, so that the call to Refresh comes from the main thread
  wxCommandEvent evt( EVT_FAKE_REFRESH, image_panel_->GetId() );
  wxPostEvent( image_panel_, evt );

  image_mutex_.unlock();
}

void CameraPanel::onFakeRefresh( wxCommandEvent& event )
{
  image_panel_->Refresh();
}

void CameraPanel::onImageSize( wxSizeEvent& event )
{
  if ( image_ )
  {
    recreate_bitmap_ = true;
  }

  image_panel_->Refresh();

  event.Skip();
}

void CameraPanel::drawPan( wxDC& dc, wxPen& pen, float pan )
{
  wxSize panelSize = image_panel_->GetSize();
  int panelWidth = panelSize.GetWidth();
  int panelHeight = panelSize.GetHeight();

  int adjustedWidth = panelWidth * BAR_WINDOW_PROPORTION;

  int padWidth = (panelWidth - adjustedWidth) / 2;

  // Normalize pan to the measure bar
  pan -= pan_min_;
  pan /= (pan_max_ - pan_min_);
  pan *= adjustedWidth;

  dc.SetPen( pen );
  // draw pan tick
  dc.DrawLine( padWidth + (int)pan, panelHeight, padWidth + (int)pan, panelHeight - POSITION_TICK_LENGTH );
}

void CameraPanel::drawTilt( wxDC& dc, wxPen& pen, float tilt )
{
  wxSize panelSize = image_panel_->GetSize();
  int panelWidth = panelSize.GetWidth();
  int panelHeight = panelSize.GetHeight();

  int adjustedHeight = panelHeight * BAR_WINDOW_PROPORTION;

  int padHeight = (panelHeight - adjustedHeight) / 2;

  // Normalize tilt to the measure bar
  tilt -= tilt_min_;
  tilt /= (tilt_max_ - tilt_min_);
  tilt *= adjustedHeight;

  dc.SetPen( pen );
  // draw tilt tick
  dc.DrawLine( panelWidth, adjustedHeight + padHeight - (int)tilt,
               panelWidth - POSITION_TICK_LENGTH, adjustedHeight + padHeight - (int)tilt );
}

void CameraPanel::drawNewPanTiltLocations( wxDC& dc )
{
	float newPan, newTilt;
	if( left_mouse_down_ )
	{
	  newPan = computeNewPan( start_mouse_x_, current_mouse_x_ );
	  newTilt = computeNewTilt( start_mouse_y_, current_mouse_y_ );
	}
	else
	{
		newPan = pan_target_;
		newTilt = tilt_target_;
	}

  wxPen pen( *wxRED_PEN );
  pen.SetWidth( POSITION_TICK_WIDTH );

  drawPan( dc, pen, newPan );
  drawTilt( dc, pen, newTilt );
}

void CameraPanel::drawCurrentPanTiltLocations( wxDC& dc )
{
  wxPen pen( *wxWHITE_PEN );
  pen.SetWidth( POSITION_TICK_WIDTH );

  drawPan( dc, pen, current_pan_ );
  drawTilt( dc, pen, current_tilt_ );
}

void CameraPanel::drawZoom( wxDC& dc, wxPen& pen, float zoom )
{
  wxSize panelSize = image_panel_->GetSize();
  int panelHeight = panelSize.GetHeight();

  int adjustedHeight = panelHeight * BAR_WINDOW_PROPORTION;

  int padHeight = (panelHeight - adjustedHeight) / 2;

  // Normalize zoom to the window
  zoom -= zoom_min_;
  zoom /= (zoom_max_ - zoom_min_);
  zoom *= adjustedHeight;

  dc.SetPen( pen );

  // draw zoom bar
  dc.DrawLine( 0, adjustedHeight + padHeight - (int)zoom, POSITION_TICK_LENGTH, adjustedHeight + padHeight - (int)zoom );
}

void CameraPanel::drawNewZoomLocation( wxDC& dc )
{
	float newZoom;
	if( right_mouse_down_ )
	{
		newZoom = computeNewZoom( start_mouse_y_, current_mouse_y_ );
	}
	else
	{
		newZoom = zoom_target_;
	}
  wxPen pen( *wxRED_PEN );
  pen.SetWidth( POSITION_TICK_WIDTH );

  drawZoom( dc, pen, newZoom );
}

void CameraPanel::drawCurrentZoomLocation( wxDC& dc )
{
  wxPen pen( *wxWHITE_PEN );
  pen.SetWidth( POSITION_TICK_WIDTH );

  drawZoom( dc, pen, current_zoom_ );
}

void CameraPanel::drawPanBar( wxDC& dc )
{
  wxSize panelSize = image_panel_->GetSize();
  int panelWidth = panelSize.GetWidth();
  int panelHeight = panelSize.GetHeight();

  int adjustedWidth = panelWidth * BAR_WINDOW_PROPORTION;

  int padWidth = (panelWidth - adjustedWidth) / 2;

  wxBrush brush( *wxBLACK_BRUSH );
  dc.SetBrush( brush );
  wxPen pen( *wxWHITE_PEN );
  dc.SetPen( pen );

  dc.DrawRectangle( padWidth, panelHeight - BAR_WIDTH, adjustedWidth, BAR_WIDTH );
}

void CameraPanel::drawTiltBar( wxDC& dc )
{
  wxSize panelSize = image_panel_->GetSize();
  int panelWidth = panelSize.GetWidth();
  int panelHeight = panelSize.GetHeight();

  int adjustedHeight = panelHeight * BAR_WINDOW_PROPORTION;

  int padHeight = (panelHeight - adjustedHeight) / 2;

  wxBrush brush( *wxBLACK_BRUSH );
  dc.SetBrush( brush );
  wxPen pen( *wxWHITE_PEN );
  dc.SetPen( pen );

  dc.DrawRectangle( panelWidth - BAR_WIDTH, padHeight, BAR_WIDTH, adjustedHeight );
}

void CameraPanel::drawZoomBar( wxDC& dc )
{
  wxSize panelSize = image_panel_->GetSize();
  int panelHeight = panelSize.GetHeight();

  int adjustedHeight = panelHeight * BAR_WINDOW_PROPORTION;

  int padHeight = (panelHeight - adjustedHeight) / 2;

  wxBrush brush( *wxBLACK_BRUSH );
  dc.SetBrush( brush );
  wxPen pen( *wxWHITE_PEN );
  dc.SetPen( pen );

  dc.DrawRectangle( 0, padHeight, BAR_WIDTH, adjustedHeight );
}

void CameraPanel::onImagePaint( wxPaintEvent& event )
{
  wxPaintDC dc( image_panel_ );

  // Draw the image as the background
  if ( image_ )
  {
    image_mutex_.lock();

    if ( recreate_bitmap_ )
    {
      wxSize scale = image_panel_->GetSize();
      bitmap_ = image_->Scale( scale.GetWidth(), scale.GetHeight() );

      recreate_bitmap_ = false;
    }

    dc.DrawBitmap( bitmap_, 0, 0, false );

    image_mutex_.unlock();
  }
  else
  {
    dc.DrawText( wxT( "No image to display" ), 0, 0 );
  }

  bool ptzStateEnabled = isPTZStateEnabled();
  bool ptzControlEnabled = isPTZControlEnabled();

  if ( ptzStateEnabled || ptzControlEnabled )
  {
    drawPanBar( dc );
    drawTiltBar( dc );
    drawZoomBar( dc );

	if ( !( left_mouse_down_ && right_mouse_down_ ) && ptzControlEnabled )
	{
		// Now draw pan/tilt if necessary
		if ( left_mouse_down_ || has_pan_target_ || has_tilt_target_ )
		{
		  drawNewPanTiltLocations( dc );
		}

		if ( right_mouse_down_ || has_zoom_target_ )
		{
		  drawNewZoomLocation( dc );
		}
	}

	if ( ptzStateEnabled )
	{
		drawCurrentPanTiltLocations( dc );
		drawCurrentZoomLocation( dc );
	}
  }
}

void CameraPanel::onSetup( wxCommandEvent& event )
{
  CameraSetupDialog dialog( this, ros_node_, name_, pan_min_, pan_max_, tilt_min_, tilt_max_, zoom_min_, zoom_max_, ptz_enabled_ );

  if (dialog.ShowModal() == wxOK)
  {
    setName( dialog.getName() );
    setPTZEnabled( dialog.getPTZEnabled() );
    pan_min_ = dialog.getPanMin();
    pan_max_ = dialog.getPanMax();
    tilt_min_ = dialog.getTiltMin();
    tilt_max_ = dialog.getTiltMax();
    zoom_min_ = dialog.getZoomMin();
    zoom_max_ = dialog.getZoomMax();
  }
  image_panel_->Refresh();
}

void CameraPanel::onEnable( wxCommandEvent& event )
{
  setEnabled( event.IsChecked() );
}

void CameraPanel::onLeftMouseDown( wxMouseEvent& event )
{
  if ( !isPTZControlEnabled() )
  {
    return;
  }

  left_mouse_down_ = true;

  start_mouse_x_ = current_mouse_x_ = event.GetX();
  start_mouse_y_ = current_mouse_y_ = event.GetY();
}

void CameraPanel::onLeftMouseUp( wxMouseEvent& event )
{
  if ( !left_mouse_down_ || !isPTZControlEnabled() )
  {
    left_mouse_down_ = false;
    return;
  }

  float newPan = computeNewPan( start_mouse_x_, event.GetX() );
  float newTilt = computeNewTilt( start_mouse_y_, event.GetY() );

  ptz_command_message_.pan.valid = 1;
  ptz_command_message_.pan.cmd = newPan;
  ptz_command_message_.tilt.valid = 1;
  ptz_command_message_.tilt.cmd = newTilt;
  ptz_command_message_.zoom.valid = 0;

  ros_node_->publish(ptz_control_topic_, ptz_command_message_);

  left_mouse_down_ = false;

  has_pan_target_ = true;
  has_tilt_target_ = true;
  pan_target_ = newPan;
  tilt_target_ = newTilt;

  start_mouse_x_ = current_mouse_x_ = event.GetX();
  start_mouse_y_ = current_mouse_y_ = event.GetY();

  image_panel_->Refresh();
}

void CameraPanel::onMiddleMouseUp( wxMouseEvent& event )
{
	if ( !isPTZControlEnabled() )
		return;

	wxSize scale = image_panel_->GetSize();

	float pan_mid = ((float)scale.GetWidth())/2.0f;
	float tilt_mid = ((float)scale.GetHeight())/2.0f;

	float pan_change = (event.m_x - pan_mid)/pan_mid*(21.0f-(current_zoom_)/500.0f);
	float tilt_change = -(event.m_y - tilt_mid)/tilt_mid*(15.0f-(current_zoom_)/700.0f);

	if( has_pan_target_ )
		pan_target_ = pan_target_ + pan_change;
	else
		pan_target_ = current_pan_ + pan_change;

	if( has_tilt_target_ )
		tilt_target_ = tilt_target_ + tilt_change;
	else
		tilt_target_ = current_tilt_ + tilt_change;

	has_pan_target_ = true;
	has_tilt_target_ = true;

	ptz_command_message_.pan.valid = 1;
	ptz_command_message_.pan.cmd = std::min(std::max(pan_target_, pan_min_), pan_max_);
	ptz_command_message_.tilt.valid = 1;
	ptz_command_message_.tilt.cmd = std::min(std::max(tilt_target_, tilt_min_), tilt_max_);
	ptz_command_message_.zoom.valid = 0;

	ros_node_->publish(ptz_control_topic_, ptz_command_message_);

	image_panel_->Refresh();
}

void CameraPanel::onMouseWheel( wxMouseEvent& event )
{
	if( !isPTZControlEnabled() )
	{
		return;
	}

	float zoom = has_zoom_target_ ? zoom_target_ : current_zoom_;
	has_zoom_target_ = true;
	zoom = std::min(std::max(zoom + (float)event.GetWheelRotation()*(zoom_max_-zoom_min_)/ZOOM_SCROLL_STEPS/(float)event.GetWheelDelta(), zoom_min_), zoom_max_);
	if(zoom != zoom_target_)
	{
		zoom_target_ = zoom;
		zoom_scroll_timer_.Start(SCROLL_WAIT_INTERVAL,true);
	}
}

void CameraPanel::onScrollComplete( wxTimerEvent& event )
{
  ptz_command_message_.pan.valid = 0;
  ptz_command_message_.tilt.valid = 0;
  ptz_command_message_.zoom.valid = 1;
  ptz_command_message_.zoom.cmd = zoom_target_;

  ros_node_->publish(ptz_control_topic_, ptz_command_message_);
}

void CameraPanel::onRightMouseDown( wxMouseEvent& event )
{
  if ( !isPTZControlEnabled() )
  {
    return;
  }

  right_mouse_down_ = true;

  start_mouse_x_ = current_mouse_x_ = event.GetX();
  start_mouse_y_ = current_mouse_y_ = event.GetY();
}

void CameraPanel::onRightMouseUp( wxMouseEvent& event )
{
  if ( !right_mouse_down_ || !isPTZControlEnabled() )
  {
    right_mouse_down_ = false;
    return;
  }

  float newZoom = computeNewZoom( start_mouse_y_, event.GetY() );

  ptz_command_message_.pan.valid = 0;
  ptz_command_message_.tilt.valid = 0;
  ptz_command_message_.zoom.valid = 1;
  ptz_command_message_.zoom.cmd = newZoom;

  ros_node_->publish(ptz_control_topic_, ptz_command_message_);

  right_mouse_down_ = false;

  has_zoom_target_ = true;
  zoom_target_ = newZoom;

  start_mouse_x_ = current_mouse_x_ = event.GetX();
  start_mouse_y_ = current_mouse_y_ = event.GetY();

  image_panel_->Refresh();
}

void CameraPanel::onMouseMotion( wxMouseEvent& event )
{
  if ( left_mouse_down_ && right_mouse_down_
       || !(left_mouse_down_ || right_mouse_down_) )
  {
    return;
  }

  current_mouse_x_ = event.GetX();
  current_mouse_y_ = event.GetY();

  image_panel_->Refresh();
}

float CameraPanel::computeNewPan( int32_t startX, int32_t endX )
{
  float pan = has_pan_target_ ? pan_target_ : current_pan_;
  int32_t diffX = endX - startX;

  return std::min(std::max(pan  + PAN_SCALE*(float)diffX, pan_min_), pan_max_);
}

float CameraPanel::computeNewTilt( int32_t startY, int32_t endY )
{
  float tilt = has_tilt_target_ ? tilt_target_ : current_tilt_;
  int32_t diffY = startY - endY;

  return std::min(std::max(tilt + TILT_SCALE*(float)diffY, tilt_min_), tilt_max_);
}

float CameraPanel::computeNewZoom( int32_t startY, int32_t endY )
{
  float zoom = has_zoom_target_ ? zoom_target_ : current_zoom_;
  int32_t diffY = startY - endY;

  return std::min(std::max(zoom + ZOOM_SCALE*(float)diffY, zoom_min_), zoom_max_);
}
