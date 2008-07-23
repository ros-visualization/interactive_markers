#pragma once

#include "CameraPanelsGenerated.h"

// ROS includes
#include <rosthread/mutex.h>
#include <std_msgs/Image.h>
#include <std_msgs/PTZActuatorState.h>
#include <std_msgs/PTZActuatorCmd.h>

#include "image_utils/image_codec.h"

// wx includes
#include <wx/bitmap.h>

namespace ros
{
	class node;
}

class CameraPanel : public CameraPanelBase
{
public:
	CameraPanel( wxWindow* parent );
	~CameraPanel();

	void SetEnabled( bool enabled );

	void SetImageSubscription( const std::string& subscription );
	void IncomingImage();

	void SetPTZStateSubscription( const std::string& subscription );
	void IncomingPTZState();

	void SetPTZControlCommand( const std::string& command );

	bool IsPTZStateEnabled() { return m_Enabled && !m_PTZStateSubscription.empty(); }
	bool IsPTZControlEnabled() { return m_Enabled && !m_PTZControlCommand.empty(); }

private:
	void SubscribeImage();
	void UnsubscribeImage();

	void SubscribePTZState();
	void UnsubscribePTZState();

	void AdvertisePTZControl();
	void StopPTZControl();

	void EnableAll();
	void DisableAll();

	float ComputeNewPan( int32_t startX, int32_t endX, float currentPan );
	float ComputeNewTilt( int32_t startY, int32_t endY, float currentTilt );
	float ComputeNewZoom( int32_t startY, int32_t endY, float currentZoom );

	void DrawNewPanTiltLocations( wxDC& dc );
	void DrawCurrentPanTiltLocations( wxDC& dc );

	void DrawNewZoomLocation( wxDC& dc );
	void DrawCurrentZoomLocation( wxDC& dc );

	// custom draw functions
	void DrawPan( wxDC& dc, wxPen& pen, float pan );
	void DrawTilt( wxDC& dc, wxPen& pen, float tilt );
	void DrawZoom( wxDC& dc, wxPen& pen, float zoom );

	void DrawPanBar( wxDC& dc );
	void DrawTiltBar( wxDC& dc );
	void DrawZoomBar( wxDC& dc );

	// wx callbacks
	void OnSetup( wxCommandEvent& event );
	void OnEnable( wxCommandEvent& event );
	void OnImagePaint( wxPaintEvent& event );
	void OnImageSize( wxSizeEvent& event );

	void OnRightMouseDown( wxMouseEvent& event );
	void OnRightMouseUp( wxMouseEvent& event );
	void OnLeftMouseDown( wxMouseEvent& event );
	void OnLeftMouseUp( wxMouseEvent& event );

	void OnMouseMotion( wxMouseEvent& event );

	void OnFakeRefresh( wxCommandEvent& event );
	

	// private variables
	bool m_Enabled;

	std::string m_ImageSubscription;
	std::string m_PTZStateSubscription;
	std::string m_PTZControlCommand;

	ros::node* m_ROSNode;

	std_msgs::Image 	m_ImageMessage;
	uint8_t*			m_ImageData;
	wxImage*			m_Image;
	ros::thread::mutex 	m_ImageMutex;
	ImageCodec<std_msgs::Image> m_ImageCodec;
	wxBitmap			m_Bitmap;
	bool				m_RecreateBitmap;

	std_msgs::PTZActuatorState m_PTZStateMessage;
	std_msgs::PTZActuatorCmd   m_PTZControlMessage;

	float m_CurrentPan;
	float m_CurrentTilt;
	float m_CurrentZoom;


	// Mouse handling
	bool m_LeftMouseDown;
	bool m_RightMouseDown;

	int32_t m_StartMouseX;
	int32_t m_StartMouseY;
	int32_t m_CurrentMouseX;
	int32_t m_CurrentMouseY;
};
