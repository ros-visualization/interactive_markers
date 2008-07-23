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


#ifndef WX_CAMERA_PANEL_CAMERA_PANEL_H
#define WX_CAMERA_PANEL_CAMERA_PANEL_H

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

/** Standalone camera panel that can be used anywhere wx is
 *  being used.  Can be used by either assigning a blank
 *  panel/frame as its parent, or by adding it to a sizer */
class CameraPanel : public CameraPanelBase
{
public:
  CameraPanel( wxWindow* parent );
  ~CameraPanel();

  /// Set whether this panel is enabled or not.  Subscribes/unsubscribes from all messages, and disables PTZ control
  void SetEnabled( bool enabled );

  /// Set the subscription topic for the image display
  void SetImageSubscription( const std::string& topic );

  /// Set the subscription topic for the PTZ state display
  void SetPTZStateSubscription( const std::string& topic );

  /// Set the advertised topic for the PTZ control commands
  void SetPTZControlCommand( const std::string& topic );

private:
  /// Subscribes to the current image topic, unless disabled
  void SubscribeImage();
  /// Unsubscribes from the current image topic, unless disabled
  void UnsubscribeImage();

  /// Subscribes to the current PTZ state topic, unless disabled
  void SubscribePTZState();
  /// Unsubscribes from the current PTZ state topic, unless disabled
  void UnsubscribePTZState();

  /// Begins advertising on the current PTZ control topic, unless disabled
  void AdvertisePTZControl();
  /// Stops advertising on the current PTZ control topic, unless disabled
  void StopPTZControl();

  /// Returns whether or not the image message + display are enabled
  bool IsImageEnabled() { return m_Enabled && !m_ImageTopic.empty();}
  /// Returns whether or not the state message + display are enabled
  bool IsPTZStateEnabled() { return m_Enabled && !m_PTZStateTopic.empty();}
  /// Returns whether or not the control message + display are enabled
  bool IsPTZControlEnabled() { return m_Enabled && !m_PTZControlTopic.empty();}

  /// Start all enabled subscriptions and advertisements
  void StartAll();
  /// Stop all enabled subscriptions and advertizements
  void StopAll();

  /// ROS callback for an incoming image message
  void IncomingImage();
  /// ROS callback for an incoming PTZ state message
  void IncomingPTZState();

  /// Compute a new pan value, given a mouse start/end position
  float ComputeNewPan( int32_t startX, int32_t endX, float currentPan );
  /// Compute a new tilt value, given a mouse start/end position
  float ComputeNewTilt( int32_t startY, int32_t endY, float currentTilt );
  /// Compute a new zoom value, given a mouse start/end position
  float ComputeNewZoom( int32_t startY, int32_t endY, float currentZoom );

  // custom draw functions

  /// Draws the new pan/tilt locations (when dragging the mouse)
  void DrawNewPanTiltLocations( wxDC& dc );
  /// Draws the current pan/tilt locations (current meaning the latest data from the camera)
  void DrawCurrentPanTiltLocations( wxDC& dc );

  /// Draws the new zoom location (when dragging the mouse)
  void DrawNewZoomLocation( wxDC& dc );
  /// Draws the current zoom location (latest data from the camera)
  void DrawCurrentZoomLocation( wxDC& dc );

  /// Draw a pan tickmark
  void DrawPan( wxDC& dc, wxPen& pen, float pan );
  /// Draw a tilt tickmark
  void DrawTilt( wxDC& dc, wxPen& pen, float tilt );
  /// Draw a zoom tickmark
  void DrawZoom( wxDC& dc, wxPen& pen, float zoom );

  /// Draw the pan measurement bar
  void DrawPanBar( wxDC& dc );
  /// Draw the tilt measurement bar
  void DrawTiltBar( wxDC& dc );
  /// Draw the zoom measurement bar
  void DrawZoomBar( wxDC& dc );

  // wx callbacks
  /// wx callback, called when the setup button is pressed
  void OnSetup( wxCommandEvent& event );
  /// wx callback, called when the enable checkbox changes
  void OnEnable( wxCommandEvent& event );
  /// wx callback, called to paint the image panel
  void OnImagePaint( wxPaintEvent& event );
  /// wx callback, called when the image panel is resized
  void OnImageSize( wxSizeEvent& event );

  /// wx callback, called when the right mouse button is pressed on the image panel
  void OnRightMouseDown( wxMouseEvent& event );
  /// wx callback, called when the right mouse button is released on the image panel
  void OnRightMouseUp( wxMouseEvent& event );
  /// wx callback, called when the left mouse button is pressed on the image panel
  void OnLeftMouseDown( wxMouseEvent& event );
  /// wx callback, called when the left mouse button is released on the image panel
  void OnLeftMouseUp( wxMouseEvent& event );

  /// wx callback, called when the mouse moves over the image panel
  void OnMouseMotion( wxMouseEvent& event );

  /// wx callback for a custom event, called to force a refresh from another thread
  void OnFakeRefresh( wxCommandEvent& event );


  // private variables

  /// Are we enabled?
  bool m_Enabled;

  /// ROS topic for the incoming image
  std::string m_ImageTopic;
  /// ROS topic for the incoming PTZ state
  std::string m_PTZStateTopic;
  /// ROS topic for the outgoing PTZ control
  std::string m_PTZControlTopic;

  /// Our ROS node
  ros::node* m_ROSNode;

  std_msgs::Image   m_ImageMessage;
  uint8_t*      m_ImageData;
  wxImage*      m_Image;
  ros::thread::mutex  m_ImageMutex;
  ImageCodec<std_msgs::Image> m_ImageCodec;
  wxBitmap      m_Bitmap;
  bool        m_RecreateBitmap;

  std_msgs::PTZActuatorState m_PTZStateMessage;
  std_msgs::PTZActuatorCmd   m_PTZControlMessage;

  /// Latest pan received from ROS
  float m_CurrentPan;
  /// Latest tilt received from ROS
  float m_CurrentTilt;
  /// Latest zoom received from ROS
  float m_CurrentZoom;

  // Mouse handling
  /// Is the left mouse button currently pressed?
  bool m_LeftMouseDown;
  /// Is the right mouse button currently pressed?
  bool m_RightMouseDown;

  /// X pixel location at which the mouse was pressed (relative to the image panel)
  int32_t m_StartMouseX;
  /// Y pixel location at which the mouse was pressed (relative to the image panel)
  int32_t m_StartMouseY;
  /// X pixel location of the mouse right now (relative to the image panel)
  int32_t m_CurrentMouseX;
  /// Y pixel location of the mouse right now (relative to the image panel)
  int32_t m_CurrentMouseY;
};

#endif
