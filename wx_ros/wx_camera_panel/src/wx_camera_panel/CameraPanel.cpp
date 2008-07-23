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
#define PAN_MIN (-169.0f)
#define PAN_MAX (169.0f)

#define TILT_SCALE (0.1f)
#define TILT_MIN (-10.0f)
#define TILT_MAX (90.0f)

#define ZOOM_SCALE (10.0f)
#define ZOOM_MIN (0.0f)
#define ZOOM_MAX (10000.0F)

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
, m_LeftMouseDown( false )
, m_RightMouseDown( false )
, m_StartMouseX( 0 )
, m_StartMouseY( 0 )
, m_CurrentMouseX( 0 )
, m_CurrentMouseY( 0 )
{
	wxInitAllImageHandlers();

	// ensure a unique node name
	static uint32_t count = 0;
	std::stringstream ss;
	ss << "CameraPanelNode" << count++;
	 
	// jfaust TODO: rosnode should be passed in once STROS is here and it supports multiple subscribers to the same topic
	m_ROSNode = new ros::node( ss.str() );
}

CameraPanel::~CameraPanel()
{
	SetEnabled( false );

	m_ROSNode->shutdown();
	delete m_ROSNode;

	delete [] m_ImageData;
	delete m_Image;
}

void CameraPanel::SetImageSubscription( const std::string& subscription )
{
	if ( m_ImageSubscription == subscription )
	{
		return;
	}

	// if we already have a subscription, unsubscribe
	if ( !m_ImageSubscription.empty() )
	{
		UnsubscribeImage();
	}

	m_ImageSubscription = subscription;

	if ( !m_ImageSubscription.empty() )
	{
		SubscribeImage();
	}
}

void CameraPanel::SetPTZStateSubscription( const std::string& subscription )
{
	if ( m_PTZStateSubscription == subscription )
	{
		return;
	}

	// if we already have a subscription, unsubscribe
	if ( !m_PTZStateSubscription.empty() )
	{
		UnsubscribePTZState();
	}

	m_PTZStateSubscription = subscription;

	if ( !m_PTZStateSubscription.empty() )
	{
		SubscribePTZState();
	}
}

void CameraPanel::SetPTZControlCommand( const std::string& command )
{
	if ( m_PTZControlCommand == command )
	{
		return;
	}

	// if we already have a command we're advertising, stop
	if ( !m_PTZControlCommand.empty() )
	{
		StopPTZControl();
	}

	m_PTZControlCommand = command;

	if ( !m_PTZControlCommand.empty() )
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
		EnableAll();
	}
	else
	{
		DisableAll();
		m_Enabled = false;
	}

	m_Enable->SetValue( enabled );
}

void CameraPanel::EnableAll()
{
	SubscribeImage();
	SubscribePTZState();
	AdvertisePTZControl();
}

void CameraPanel::DisableAll()
{
	UnsubscribeImage();
	UnsubscribePTZState();
	StopPTZControl();
}

void CameraPanel::SubscribeImage()
{
	if ( m_Enabled && !m_ImageSubscription.empty() )
	{
		m_ROSNode->subscribe( m_ImageSubscription, m_ImageMessage, &CameraPanel::IncomingImage, this );
	}
}

void CameraPanel::UnsubscribeImage()
{
	if ( m_Enabled && !m_ImageSubscription.empty() )
	{
		m_ROSNode->unsubscribe( m_ImageSubscription );
	}
}

void CameraPanel::SubscribePTZState()
{
	if ( m_Enabled && !m_PTZStateSubscription.empty() )
	{
		m_ROSNode->subscribe( m_PTZStateSubscription, m_PTZStateMessage, &CameraPanel::IncomingPTZState, this );
	}
}

void CameraPanel::UnsubscribePTZState()
{
	if ( m_Enabled && !m_PTZStateSubscription.empty() )
	{
		m_ROSNode->unsubscribe( m_PTZStateSubscription );
	}
}

void CameraPanel::AdvertisePTZControl()
{
	if ( m_Enabled && !m_PTZControlCommand.empty() )
	{
		m_ROSNode->advertise<std_msgs::PTZActuatorCmd>(m_PTZControlCommand);
	}
}

void CameraPanel::StopPTZControl()
{
	if ( m_Enabled && !m_PTZControlCommand.empty() )
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

	//printf( "Pan: %.2f    Tilt: %.2f    Zoom: %.2f\n", m_CurrentPan, m_CurrentTilt, m_CurrentZoom );
}

void CameraPanel::IncomingImage()
{
	m_ImageMutex.lock();

	if(m_ImageMessage.compression == "raw")
	{
		m_ImageCodec.inflate();
		m_ImageMessage.compression = "jpeg";

		if(!m_ImageCodec.deflate(100))
			return;
	}

	delete [] m_ImageData;
	const uint32_t dataSize = m_ImageMessage.get_data_size();
	m_ImageData = new uint8_t[ dataSize ];
	memcpy( m_ImageData, m_ImageMessage.data, dataSize );
	
	delete m_Image;
	wxMemoryInputStream memoryStream( m_ImageData, dataSize );
	m_Image = new wxImage( memoryStream, wxBITMAP_TYPE_ANY, -1 );

	// wx really doesn't like a Refresh coming from a separate thread
	// force a size event instead, which will also cause a refresh
	wxCommandEvent evt( wxEVT_SIZE, m_ImagePanel->GetId() );
	wxPostEvent( this, evt );

	m_ImageMutex.unlock();
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
		}

		dc.DrawBitmap( m_Bitmap, 0, 0, false );

		m_ImageMutex.unlock();
	}
	else
	{
		dc.DrawText( wxT( "No image to display" ), 0, 0 );
	}

	if ( !( m_LeftMouseDown && m_RightMouseDown ) )
	{
		// Now draw pan/tilt if necessary
		if ( m_LeftMouseDown )
		{
			float newPan = ComputeNewPan( m_StartMouseX, m_CurrentMouseX, m_CurrentPan );
			float newTilt = ComputeNewTilt( m_StartMouseY, m_CurrentMouseY, m_CurrentTilt );

			wxSize panelSize = m_ImagePanel->GetSize();
			int panelWidth = panelSize.GetWidth();
			int panelHeight = panelSize.GetHeight();
			
			// Normalize pan to the window
			newPan -= PAN_MIN;
			newPan /= (PAN_MAX - PAN_MIN);
			newPan *= panelWidth;

			// Normalize tilt to the window
			newTilt -= TILT_MIN;
			newTilt /= (TILT_MAX - TILT_MIN);
			newTilt *= panelHeight;

			wxPen pen( *wxRED_PEN );
			pen.SetWidth( 5 );
			dc.SetPen( pen );
			// draw pan bar
			dc.DrawLine( (int)newPan, panelHeight, (int)newPan, panelHeight - 20 );
			// draw tilt bar
			dc.DrawLine( panelWidth, panelHeight - (int)newTilt, panelWidth - 20, panelHeight - (int)newTilt );
		}
		else if ( m_RightMouseDown )
		{
			float newZoom = ComputeNewZoom( m_StartMouseY, m_CurrentMouseY, m_CurrentZoom );

			wxSize panelSize = m_ImagePanel->GetSize();
			int panelWidth = panelSize.GetWidth();
			int panelHeight = panelSize.GetHeight();

			// Normalize zoom to the window
			newZoom -= ZOOM_MIN;
			newZoom /= (ZOOM_MAX - ZOOM_MIN);
			newZoom *= panelHeight;

			wxPen pen( *wxCYAN_PEN );
			pen.SetWidth( 5 );
			dc.SetPen( pen );

			// draw zoom bar
			dc.DrawLine( panelWidth, panelHeight - (int)newZoom, panelWidth - 20, panelHeight - (int)newZoom );
		}
	}
}

void CameraPanel::OnSetup( wxCommandEvent& event )
{
	CameraSetupDialog dialog( this, m_ROSNode, m_ImageSubscription, m_PTZStateSubscription, m_PTZControlCommand );

	if (dialog.ShowModal() == wxID_OK)
	{
		SetImageSubscription( dialog.GetImageSubscription() );
		SetPTZStateSubscription( dialog.GetPTZStateSubscription() );
		SetPTZControlCommand( dialog.GetPTZControlCommand() );
	}
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

	float newPan = ComputeNewPan( m_StartMouseX, event.GetX(), m_CurrentPan );
	float newTilt = ComputeNewTilt( m_StartMouseY, event.GetY(), m_CurrentTilt );

	m_PTZControlMessage.pan.valid = 1;
	m_PTZControlMessage.pan.cmd = newPan;
	m_PTZControlMessage.tilt.valid = 1;
	m_PTZControlMessage.tilt.cmd = newTilt;
	m_PTZControlMessage.zoom.valid = 0;

	m_ROSNode->publish(m_PTZControlCommand, m_PTZControlMessage);

	m_LeftMouseDown = false;
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

	float newZoom = ComputeNewZoom( m_StartMouseY, event.GetY(), m_CurrentZoom );

	m_PTZControlMessage.pan.valid = 0;
	m_PTZControlMessage.tilt.valid = 0;
	m_PTZControlMessage.zoom.valid = 1;
	m_PTZControlMessage.zoom.cmd = newZoom;

	m_ROSNode->publish(m_PTZControlCommand, m_PTZControlMessage);

	m_RightMouseDown = false;
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

float CameraPanel::ComputeNewPan( int32_t startX, int32_t endX, float currentPan )
{
	int32_t diffX = endX - startX;

	return std::min(std::max(currentPan  + PAN_SCALE*(float)diffX, PAN_MIN), PAN_MAX);
}

float CameraPanel::ComputeNewTilt( int32_t startY, int32_t endY, float currentTilt )
{
	int32_t diffY = startY - endY;

	return std::min(std::max(currentTilt + TILT_SCALE*(float)diffY, TILT_MIN), TILT_MAX);
}

float CameraPanel::ComputeNewZoom( int32_t startY, int32_t endY, float currentZoom )
{
	int32_t diffY = startY - endY;

	return std::min(std::max(currentZoom + ZOOM_SCALE*(float)diffY, ZOOM_MIN), ZOOM_MAX);
}
