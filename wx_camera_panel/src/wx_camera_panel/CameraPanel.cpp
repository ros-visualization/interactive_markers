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
, m_Enabled( false )
, m_ImageData( NULL )
, m_Image( NULL )
, m_ImageCodec( &m_ImageMessage )
, m_RecreateBitmap( false )
, m_CurrentPan( 0.0f )
, m_CurrentTilt( 0.0f )
, m_CurrentZoom( 0.0f )
, m_HasPanTarget( false )
, m_PanTarget( 0.0f )
, m_HasTiltTarget( false )
, m_TiltTarget( 0.0f )
, m_HasZoomTarget( false )
, m_ZoomTarget( 0.0f )
, m_LeftMouseDown( false )
, m_RightMouseDown( false )
, m_StartMouseX( 0 )
, m_StartMouseY( 0 )
, m_CurrentMouseX( 0 )
, m_CurrentMouseY( 0 )
, m_PanMin( -169.0f )
, m_PanMax( 169.0f )
, m_TiltMin( -10.0f )
, m_TiltMax( 90.0f )
, m_ZoomMin( 1.0f )
, m_ZoomMax( 9999.0f )
, m_ZoomScrollTimer( this )
{
  wxInitAllImageHandlers();
  
  Connect( wxEVT_TIMER, wxTimerEventHandler(CameraPanel::OnScrollComplete), NULL, this);
  
  // ensure a unique node name
  static uint32_t count = 0;
  std::stringstream ss;
  ss << "CameraPanelNode" << count++;

  // jfaust TODO: rosnode should be passed in once STROS is here and it supports multiple subscribers to the same topic
  m_ROSNode = new ros::node( ss.str() );

  m_ImagePanel->Connect( EVT_FAKE_REFRESH, wxCommandEventHandler( CameraPanel::OnFakeRefresh ), NULL, this );
}

CameraPanel::~CameraPanel()
{
  m_ImagePanel->Disconnect( EVT_FAKE_REFRESH, wxCommandEventHandler( CameraPanel::OnFakeRefresh ), NULL, this );

  SetEnabled( false );

  m_ROSNode->shutdown();
  delete m_ROSNode;

  delete [] m_ImageData;
  delete m_Image;
}

void CameraPanel::SetImageSubscription( const std::string& subscription )
{
  if ( m_ImageTopic == subscription )
  {
    return;
  }

  // if we already have a subscription, unsubscribe
  if ( !m_ImageTopic.empty() )
  {
    UnsubscribeImage();
  }

  m_ImageTopic = subscription;

  if ( !m_ImageTopic.empty() )
  {
    SubscribeImage();
  }
}

void CameraPanel::SetPTZStateSubscription( const std::string& subscription )
{
  if ( m_PTZStateTopic == subscription )
  {
    return;
  }

  // if we already have a subscription, unsubscribe
  if ( !m_PTZStateTopic.empty() )
  {
    UnsubscribePTZState();
  }

  m_PTZStateTopic = subscription;

  if ( !m_PTZStateTopic.empty() )
  {
    SubscribePTZState();
  }
}

void CameraPanel::SetPTZControlCommand( const std::string& command )
{
  if ( m_PTZControlTopic == command )
  {
    return;
  }

  // if we already have a command we're advertising, stop
  if ( !m_PTZControlTopic.empty() )
  {
    StopPTZControl();
  }

  m_PTZControlTopic = command;

  if ( !m_PTZControlTopic.empty() )
  {
    AdvertisePTZControl();
  }
}

void CameraPanel::SetEnabled( bool enabled )
{
  if ( m_Enabled == enabled )
  {
    return;
  }

  if ( enabled )
  {
    m_Enabled = true;
    StartAll();
  }
  else
  {
    StopAll();
    m_Enabled = false;
  }

  m_Enable->SetValue( enabled );
}

void CameraPanel::SetPanLimits( float min, float max )
{
  m_PanMin = min;
  m_PanMax = max;
}

void CameraPanel::SetTiltLimits( float min, float max )
{
  m_TiltMin = min;
  m_TiltMax = max;
}

void CameraPanel::SetZoomLimits( float min, float max )
{
  m_ZoomMin = min;
  m_ZoomMax = max;
}

void CameraPanel::StartAll()
{
  SubscribeImage();
  SubscribePTZState();
  AdvertisePTZControl();
}

void CameraPanel::StopAll()
{
  UnsubscribeImage();
  UnsubscribePTZState();
  StopPTZControl();
}

void CameraPanel::SubscribeImage()
{
  if ( IsImageEnabled() )
  {
    m_ROSNode->subscribe( m_ImageTopic, m_ImageMessage, &CameraPanel::IncomingImage, this );
  }
}

void CameraPanel::UnsubscribeImage()
{
  if ( IsImageEnabled() )
  {
    m_ROSNode->unsubscribe( m_ImageTopic );
  }
}

void CameraPanel::SubscribePTZState()
{
  if ( IsPTZStateEnabled() )
  {
    m_ROSNode->subscribe( m_PTZStateTopic, m_PTZStateMessage, &CameraPanel::IncomingPTZState, this );
  }
}

void CameraPanel::UnsubscribePTZState()
{
  if ( IsPTZStateEnabled() )
  {
    m_ROSNode->unsubscribe( m_PTZStateTopic );
  }
}

void CameraPanel::AdvertisePTZControl()
{
  if ( IsPTZControlEnabled() )
  {
    m_ROSNode->advertise<std_msgs::PTZActuatorCmd>(m_PTZControlTopic);
  }
}

void CameraPanel::StopPTZControl()
{
  if ( IsPTZControlEnabled() )
  {
    // jfaust TODO: once there is support for stopping an advertisement
  }
}

void CameraPanel::IncomingPTZState()
{
  if ( m_PTZStateMessage.pan.pos_valid )
  {
    m_CurrentPan = m_PTZStateMessage.pan.pos;
  }

  if ( m_PTZStateMessage.tilt.pos_valid )
  {
    m_CurrentTilt = m_PTZStateMessage.tilt.pos;
  }

  if ( m_PTZStateMessage.zoom.pos_valid )
  {
    m_CurrentZoom = m_PTZStateMessage.zoom.pos;
  }

  if ( m_HasPanTarget && abs( m_CurrentPan - m_PanTarget ) < TARGET_PAN_EPSILON )
  {
    m_HasPanTarget = false;
  }

  if ( m_HasTiltTarget && abs( m_CurrentTilt - m_TiltTarget ) < TARGET_TILT_EPSILON )
  {
    m_HasTiltTarget = false;
  }

  if ( m_HasZoomTarget && abs( m_CurrentZoom - m_ZoomTarget ) < TARGET_ZOOM_EPSILON )
  {
    m_HasZoomTarget = false;
  }

  // wx really doesn't like a Refresh call coming from a separate thread
  // send a fake refresh event, so that the call to Refresh comes from the main thread
  wxCommandEvent evt( EVT_FAKE_REFRESH, m_ImagePanel->GetId() );
  wxPostEvent( m_ImagePanel, evt );
}

void CameraPanel::IncomingImage()
{
  m_ImageMutex.lock();

  // if this image is raw, compress it as jpeg since wx doesn't support the raw format
  if (m_ImageMessage.compression == "raw")
  {
    m_ImageCodec.inflate();
    m_ImageMessage.compression = "jpeg";

    if (!m_ImageCodec.deflate(100))
      return;
  }

  
  delete [] m_ImageData;
  const uint32_t dataSize = m_ImageMessage.get_data_size();
  m_ImageData = new uint8_t[ dataSize ];
  memcpy( m_ImageData, m_ImageMessage.data, dataSize );

  delete m_Image;
  wxMemoryInputStream memoryStream( m_ImageData, dataSize );
  m_Image = new wxImage( memoryStream, wxBITMAP_TYPE_ANY, -1 );

  m_RecreateBitmap = true;

  // wx really doesn't like a Refresh call coming from a separate thread
  // send a fake refresh event, so that the call to Refresh comes from the main thread
  wxCommandEvent evt( EVT_FAKE_REFRESH, m_ImagePanel->GetId() );
  wxPostEvent( m_ImagePanel, evt );

  m_ImageMutex.unlock();
}

void CameraPanel::OnFakeRefresh( wxCommandEvent& event )
{
  m_ImagePanel->Refresh();
}

void CameraPanel::OnImageSize( wxSizeEvent& event )
{
  if ( m_Image )
  {
    m_RecreateBitmap = true;

    m_ImagePanel->Refresh();
  }

  event.Skip();
}

void CameraPanel::DrawPan( wxDC& dc, wxPen& pen, float pan )
{
  wxSize panelSize = m_ImagePanel->GetSize();
  int panelWidth = panelSize.GetWidth();
  int panelHeight = panelSize.GetHeight();

  int adjustedWidth = panelWidth * BAR_WINDOW_PROPORTION;

  int padWidth = (panelWidth - adjustedWidth) / 2;

  // Normalize pan to the measure bar
  pan -= m_PanMin;
  pan /= (m_PanMax - m_PanMin);
  pan *= adjustedWidth;

  dc.SetPen( pen );
  // draw pan tick
  dc.DrawLine( padWidth + (int)pan, panelHeight, padWidth + (int)pan, panelHeight - POSITION_TICK_LENGTH );
}

void CameraPanel::DrawTilt( wxDC& dc, wxPen& pen, float tilt )
{
  wxSize panelSize = m_ImagePanel->GetSize();
  int panelWidth = panelSize.GetWidth();
  int panelHeight = panelSize.GetHeight();

  int adjustedHeight = panelHeight * BAR_WINDOW_PROPORTION;

  int padHeight = (panelHeight - adjustedHeight) / 2;

  // Normalize tilt to the measure bar
  tilt -= m_TiltMin;
  tilt /= (m_TiltMax - m_TiltMin);
  tilt *= adjustedHeight;

  dc.SetPen( pen );
  // draw tilt tick
  dc.DrawLine( panelWidth, adjustedHeight + padHeight - (int)tilt, 
               panelWidth - POSITION_TICK_LENGTH, adjustedHeight + padHeight - (int)tilt );
}

void CameraPanel::DrawNewPanTiltLocations( wxDC& dc )
{
	float newPan, newTilt;
	if( m_LeftMouseDown )
	{
	  newPan = ComputeNewPan( m_StartMouseX, m_CurrentMouseX );
	  newTilt = ComputeNewTilt( m_StartMouseY, m_CurrentMouseY );
	}
	else
	{
		newPan = m_PanTarget;
		newTilt = m_TiltTarget;
	}

  wxPen pen( *wxRED_PEN );
  pen.SetWidth( POSITION_TICK_WIDTH );

  DrawPan( dc, pen, newPan );
  DrawTilt( dc, pen, newTilt );
}

void CameraPanel::DrawCurrentPanTiltLocations( wxDC& dc )
{
  wxPen pen( *wxWHITE_PEN );
  pen.SetWidth( POSITION_TICK_WIDTH );

  DrawPan( dc, pen, m_CurrentPan );
  DrawTilt( dc, pen, m_CurrentTilt );
}

void CameraPanel::DrawZoom( wxDC& dc, wxPen& pen, float zoom )
{
  wxSize panelSize = m_ImagePanel->GetSize();
  int panelHeight = panelSize.GetHeight();

  int adjustedHeight = panelHeight * BAR_WINDOW_PROPORTION;

  int padHeight = (panelHeight - adjustedHeight) / 2;

  // Normalize zoom to the window
  zoom -= m_ZoomMin;
  zoom /= (m_ZoomMax - m_ZoomMin);
  zoom *= adjustedHeight;

  dc.SetPen( pen );

  // draw zoom bar
  dc.DrawLine( 0, adjustedHeight + padHeight - (int)zoom, POSITION_TICK_LENGTH, adjustedHeight + padHeight - (int)zoom );
}

void CameraPanel::DrawNewZoomLocation( wxDC& dc )
{
	float newZoom;
	if( m_RightMouseDown )
	{
		newZoom = ComputeNewZoom( m_StartMouseY, m_CurrentMouseY );
	}
	else
	{
		newZoom = m_ZoomTarget;
	}
  wxPen pen( *wxRED_PEN );
  pen.SetWidth( POSITION_TICK_WIDTH );

  DrawZoom( dc, pen, newZoom );
}

void CameraPanel::DrawCurrentZoomLocation( wxDC& dc )
{
  wxPen pen( *wxWHITE_PEN );
  pen.SetWidth( POSITION_TICK_WIDTH );

  DrawZoom( dc, pen, m_CurrentZoom );
}

void CameraPanel::DrawPanBar( wxDC& dc )
{
  wxSize panelSize = m_ImagePanel->GetSize();
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

void CameraPanel::DrawTiltBar( wxDC& dc )
{
  wxSize panelSize = m_ImagePanel->GetSize();
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

void CameraPanel::DrawZoomBar( wxDC& dc )
{
  wxSize panelSize = m_ImagePanel->GetSize();
  int panelHeight = panelSize.GetHeight();

  int adjustedHeight = panelHeight * BAR_WINDOW_PROPORTION;

  int padHeight = (panelHeight - adjustedHeight) / 2;

  wxBrush brush( *wxBLACK_BRUSH );
  dc.SetBrush( brush );
  wxPen pen( *wxWHITE_PEN );
  dc.SetPen( pen );

  dc.DrawRectangle( 0, padHeight, BAR_WIDTH, adjustedHeight );
}

void CameraPanel::OnImagePaint( wxPaintEvent& event )
{
  wxPaintDC dc( m_ImagePanel );

  // Draw the image as the background
  if ( m_Image )
  {
    m_ImageMutex.lock();

    if ( m_RecreateBitmap )
    {
      wxSize scale = m_ImagePanel->GetSize();
      m_Bitmap = m_Image->Scale( scale.GetWidth(), scale.GetHeight() );

      m_RecreateBitmap = false;
    }

    dc.DrawBitmap( m_Bitmap, 0, 0, false );

    m_ImageMutex.unlock();
  }
  else
  {
    dc.DrawText( wxT( "No image to display" ), 0, 0 );
  }

  bool ptzStateEnabled = IsPTZStateEnabled();
  bool ptzControlEnabled = IsPTZControlEnabled();

  if ( ptzStateEnabled || ptzControlEnabled )
  {
    DrawPanBar( dc );
    DrawTiltBar( dc );
    DrawZoomBar( dc );

	if ( !( m_LeftMouseDown && m_RightMouseDown ) && ptzControlEnabled )
	{
		// Now draw pan/tilt if necessary
		if ( m_LeftMouseDown || m_HasPanTarget || m_HasTiltTarget )
		{
		  DrawNewPanTiltLocations( dc );
		}

		if ( m_RightMouseDown || m_HasZoomTarget )
		{
		  DrawNewZoomLocation( dc );
		}
	}

	if ( ptzStateEnabled )
	{
		DrawCurrentPanTiltLocations( dc );
		DrawCurrentZoomLocation( dc );
	}
  }
}

void CameraPanel::OnSetup( wxCommandEvent& event )
{
  CameraSetupDialog dialog( this, m_ROSNode, m_ImageTopic, m_PTZStateTopic, m_PTZControlTopic, m_PanMin, m_PanMax, m_TiltMin, m_TiltMax, m_ZoomMin, m_ZoomMax );

  if (dialog.ShowModal() == wxID_OK)
  {
    SetImageSubscription( dialog.GetImageSubscription() );
    SetPTZStateSubscription( dialog.GetPTZStateSubscription() );
    SetPTZControlCommand( dialog.GetPTZControlCommand() );
    m_PanMin = dialog.GetPanMin();
    m_PanMax = dialog.GetPanMax();
    m_TiltMin = dialog.GetTiltMin();
    m_TiltMax = dialog.GetTiltMax();
    m_ZoomMin = dialog.GetZoomMin();
    m_ZoomMax = dialog.GetZoomMax();
  }
  m_ImagePanel->Refresh();
}

void CameraPanel::OnEnable( wxCommandEvent& event )
{
  SetEnabled( event.IsChecked() );
}

void CameraPanel::OnLeftMouseDown( wxMouseEvent& event )
{
  if ( !IsPTZControlEnabled() )
  {
    return;
  }

  m_LeftMouseDown = true;

  m_StartMouseX = m_CurrentMouseX = event.GetX();
  m_StartMouseY = m_CurrentMouseY = event.GetY();
}

void CameraPanel::OnLeftMouseUp( wxMouseEvent& event )
{
  if ( !m_LeftMouseDown || !IsPTZControlEnabled() )
  {
    m_LeftMouseDown = false;
    return;
  }

  float newPan = ComputeNewPan( m_StartMouseX, event.GetX() );
  float newTilt = ComputeNewTilt( m_StartMouseY, event.GetY() );

  m_PTZControlMessage.pan.valid = 1;
  m_PTZControlMessage.pan.cmd = newPan;
  m_PTZControlMessage.tilt.valid = 1;
  m_PTZControlMessage.tilt.cmd = newTilt;
  m_PTZControlMessage.zoom.valid = 0;

  m_ROSNode->publish(m_PTZControlTopic, m_PTZControlMessage);

  m_LeftMouseDown = false;

  m_HasPanTarget = true;
  m_HasTiltTarget = true;
  m_PanTarget = newPan;
  m_TiltTarget = newTilt;

  m_StartMouseX = m_CurrentMouseX = event.GetX();
  m_StartMouseY = m_CurrentMouseY = event.GetY();

  m_ImagePanel->Refresh();
}

void CameraPanel::OnMiddleMouseUp( wxMouseEvent& event )
{
	if ( !IsPTZControlEnabled() )
		return;
	
	wxSize scale = m_ImagePanel->GetSize();
	
	float pan_mid = ((float)scale.GetWidth())/2.0f;
	float tilt_mid = ((float)scale.GetHeight())/2.0f;
	
	float pan_change = (event.m_x - pan_mid)/pan_mid*(21.0f-(m_CurrentZoom)/500.0f);
	float tilt_change = -(event.m_y - tilt_mid)/tilt_mid*(15.0f-(m_CurrentZoom)/700.0f);
	
	if( m_HasPanTarget )
		m_PanTarget = m_PanTarget + pan_change;
	else
		m_PanTarget = m_CurrentPan + pan_change;
		
	if( m_HasTiltTarget )
		m_TiltTarget = m_TiltTarget + tilt_change;
	else
		m_TiltTarget = m_CurrentTilt + tilt_change;
	
	m_HasPanTarget = true;
	m_HasTiltTarget = true;
	
	m_PTZControlMessage.pan.valid = 1;
	m_PTZControlMessage.pan.cmd = std::min(std::max(m_PanTarget, m_PanMin), m_PanMax);
	m_PTZControlMessage.tilt.valid = 1;
	m_PTZControlMessage.tilt.cmd = std::min(std::max(m_TiltTarget, m_TiltMin), m_TiltMax);
	m_PTZControlMessage.zoom.valid = 0;

	m_ROSNode->publish(m_PTZControlTopic, m_PTZControlMessage);
	
	m_ImagePanel->Refresh();
}

void CameraPanel::OnMouseWheel( wxMouseEvent& event )
{
	if( !IsPTZControlEnabled() )
	{
		return;
	}
	
	float zoom = m_HasZoomTarget ? m_ZoomTarget : m_CurrentZoom;
	m_HasZoomTarget = true;
	zoom = std::min(std::max(zoom + (float)event.GetWheelRotation()*(m_ZoomMax-m_ZoomMin)/ZOOM_SCROLL_STEPS/(float)event.GetWheelDelta(), m_ZoomMin), m_ZoomMax);
	if(zoom != m_ZoomTarget)
	{
		m_ZoomTarget = zoom;
		m_ZoomScrollTimer.Start(SCROLL_WAIT_INTERVAL,true);
	}
}

void CameraPanel::OnScrollComplete( wxTimerEvent& event )
{
  m_PTZControlMessage.pan.valid = 0;
  m_PTZControlMessage.tilt.valid = 0;
  m_PTZControlMessage.zoom.valid = 1;
  m_PTZControlMessage.zoom.cmd = m_ZoomTarget;

  m_ROSNode->publish(m_PTZControlTopic, m_PTZControlMessage);
}
	
void CameraPanel::OnRightMouseDown( wxMouseEvent& event )
{
  if ( !IsPTZControlEnabled() )
  {
    return;
  }

  m_RightMouseDown = true;

  m_StartMouseX = m_CurrentMouseX = event.GetX();
  m_StartMouseY = m_CurrentMouseY = event.GetY();
}

void CameraPanel::OnRightMouseUp( wxMouseEvent& event )
{
  if ( !m_RightMouseDown || !IsPTZControlEnabled() )
  {
    m_RightMouseDown = false;
    return;
  }

  float newZoom = ComputeNewZoom( m_StartMouseY, event.GetY() );

  m_PTZControlMessage.pan.valid = 0;
  m_PTZControlMessage.tilt.valid = 0;
  m_PTZControlMessage.zoom.valid = 1;
  m_PTZControlMessage.zoom.cmd = newZoom;

  m_ROSNode->publish(m_PTZControlTopic, m_PTZControlMessage);

  m_RightMouseDown = false;

  m_HasZoomTarget = true;
  m_ZoomTarget = newZoom;

  m_StartMouseX = m_CurrentMouseX = event.GetX();
  m_StartMouseY = m_CurrentMouseY = event.GetY();

  m_ImagePanel->Refresh();
}

void CameraPanel::OnMouseMotion( wxMouseEvent& event )
{
  if ( m_LeftMouseDown && m_RightMouseDown
       || !(m_LeftMouseDown || m_RightMouseDown) )
  {
    return;
  }

  m_CurrentMouseX = event.GetX();
  m_CurrentMouseY = event.GetY();

  m_ImagePanel->Refresh();
}

float CameraPanel::ComputeNewPan( int32_t startX, int32_t endX )
{
  float pan = m_HasPanTarget ? m_PanTarget : m_CurrentPan;
  int32_t diffX = endX - startX;

  return std::min(std::max(pan  + PAN_SCALE*(float)diffX, m_PanMin), m_PanMax);
}

float CameraPanel::ComputeNewTilt( int32_t startY, int32_t endY )
{
  float tilt = m_HasTiltTarget ? m_TiltTarget : m_CurrentTilt;
  int32_t diffY = startY - endY;

  return std::min(std::max(tilt + TILT_SCALE*(float)diffY, m_TiltMin), m_TiltMax);
}

float CameraPanel::ComputeNewZoom( int32_t startY, int32_t endY )
{
  float zoom = m_HasZoomTarget ? m_ZoomTarget : m_CurrentZoom;
  int32_t diffY = startY - endY;

  return std::min(std::max(zoom + ZOOM_SCALE*(float)diffY, m_ZoomMin), m_ZoomMax);
}
