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
#include <std_msgs/Image.h>
#include <axis_cam/PTZActuatorState.h>
#include <axis_cam/PTZActuatorCmd.h>

#include "image_utils/image_codec.h"

#include "boost/thread/mutex.hpp"


// wx includes
#include <wx/bitmap.h>
#include <wx/timer.h>

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
  void setEnabled( bool enabled );

  /// Set whether the PTZ functionality is enabled
  void setPTZEnabled( bool enabled );

  /// Set the name of the camera this panel is connected to
  void setName( const std::string& name );

  /// Set the pan limits for this camera.  Note that the values are arbitrary, and are not in any defined units -- they just need
  /// to match what the camera is expecting.
  void setPanLimits( float min, float max );

  /// Set the tilt limits for this camera.  Note that the values are arbitrary, and are not in any defined units -- they just need
  /// to match what the camera is expecting.
  void setTiltLimits( float min, float max );

  /// Set the zoom limits for this camera.  Note that the values are arbitrary, and are not in any defined units -- they just need
  /// to match what the camera is expecting.
  void setZoomLimits( float min, float max );

  /**
   * \brief Set this panel to be "minimal", ie hide the Setup/Enabled buttons
   * @param minimal If true, sets the panel to be minimal
   */
  void setMinimal( bool minimal );

private:
  /// Set the subscription topic for the image display
  void setImageSubscription( const std::string& topic );

  /// Set the subscription topic for the PTZ state display
  void setPTZStateSubscription( const std::string& topic );

  /// Set the advertised topic for the PTZ control commands
  void setPTZControlCommand( const std::string& topic );

  /// Subscribes to the current image topic, unless disabled
  void subscribeImage();
  /// Unsubscribes from the current image topic, unless disabled
  void unsubscribeImage();

  /// Subscribes to the current PTZ state topic, unless disabled
  void subscribePTZState();
  /// Unsubscribes from the current PTZ state topic, unless disabled
  void unsubscribePTZState();

  /// Begins advertising on the current PTZ control topic, unless disabled
  void advertisePTZControl();
  /// Stops advertising on the current PTZ control topic, unless disabled
  void stopPTZControl();

  /// Returns whether or not the image message + display are enabled
  bool isImageEnabled() { return enabled_ && !image_topic_.empty();}
  /// Returns whether or not the state message + display are enabled
  bool isPTZStateEnabled() { return enabled_ && ptz_enabled_ && !ptz_state_topic_.empty();}
  /// Returns whether or not the control message + display are enabled
  bool isPTZControlEnabled() { return enabled_ && ptz_enabled_ && !ptz_control_topic_.empty();}

  /// Start all enabled subscriptions and advertisements
  void startAll();
  /// Stop all enabled subscriptions and advertizements
  void stopAll();

  /// ROS callback for an incoming image message
  void incomingImage();
  /// ROS callback for an incoming PTZ state message
  void incomingPTZState();

  /// Compute a new pan value, given a mouse start/end position
  float computeNewPan( int32_t startX, int32_t endX );
  /// Compute a new tilt value, given a mouse start/end position
  float computeNewTilt( int32_t startY, int32_t endY );
  /// Compute a new zoom value, given a mouse start/end position
  float computeNewZoom( int32_t startY, int32_t endY );

  // custom draw functions

  /// Draws the new pan/tilt locations (when dragging the mouse)
  void drawNewPanTiltLocations( wxDC& dc );
  /// Draws the current pan/tilt locations (current meaning the latest data from the camera)
  void drawCurrentPanTiltLocations( wxDC& dc );

  /// Draws the new zoom location (when dragging the mouse)
  void drawNewZoomLocation( wxDC& dc );
  /// Draws the current zoom location (latest data from the camera)
  void drawCurrentZoomLocation( wxDC& dc );
  /// Draw a pan tickmark
  void drawPan( wxDC& dc, wxPen& pen, float pan );
  /// Draw a tilt tickmark
  void drawTilt( wxDC& dc, wxPen& pen, float tilt );
  /// Draw a zoom tickmark
  void drawZoom( wxDC& dc, wxPen& pen, float zoom );

  /// Draw the pan measurement bar
  void drawPanBar( wxDC& dc );
  /// Draw the tilt measurement bar
  void drawTiltBar( wxDC& dc );
  /// Draw the zoom measurement bar
  void drawZoomBar( wxDC& dc );

  // wx callbacks
  /// wx callback, called when the setup button is pressed
  void onSetup( wxCommandEvent& event );
  /// wx callback, called when the enable checkbox changes
  void onEnable( wxCommandEvent& event );
  /// wx callback, called to paint the image panel
  void onImagePaint( wxPaintEvent& event );
  /// wx callback, called when the image panel is resized
  void onImageSize( wxSizeEvent& event );

  /// wx callback, called when the right mouse button is pressed on the image panel
  void onRightMouseDown( wxMouseEvent& event );
  /// wx callback, called when the right mouse button is released on the image panel
  void onRightMouseUp( wxMouseEvent& event );
  /// wx callback, called when the middle mouse button is released on the image panel
  void onMiddleMouseUp( wxMouseEvent& event );
  /// wx callback, called when the scroll wheel is spun on the image panel
  void onMouseWheel(wxMouseEvent& event);
  /// wx callback, called when the left mouse button is pressed on the image panel
  void onLeftMouseDown( wxMouseEvent& event );
  /// wx callback, called when the left mouse button is released on the image panel
  void onLeftMouseUp( wxMouseEvent& event );

  /// wx callback, called when the mouse moves over the image panel
  void onMouseMotion( wxMouseEvent& event );

  /// wx callback for a custom event, called to force a refresh from another thread
  void onFakeRefresh( wxCommandEvent& event );

  void onScrollComplete( wxTimerEvent& event );
  void onSize( wxSizeEvent& event );


  // private variables

  /// Are we enabled?
  bool enabled_;
  /// Is the PTZ functionality enabled?
  bool ptz_enabled_;

  /// Name of the camera we're listening to
  std::string name_;

  /// ROS topic for the incoming image
  std::string image_topic_;
  /// ROS topic for the incoming PTZ state
  std::string ptz_state_topic_;
  /// ROS topic for the outgoing PTZ control
  std::string ptz_control_topic_;

  /// Our ROS node
  ros::node* ros_node_;

  std_msgs::Image   image_message_;
  uint8_t*      image_data_;
  wxImage*      image_;
  boost::mutex  image_mutex_;
  ImageCodec<std_msgs::Image> image_codec_;
  wxBitmap      bitmap_;
  bool        recreate_bitmap_;

  axis_cam::PTZActuatorState ptz_state_message_;
  axis_cam::PTZActuatorCmd   ptz_command_message_;

  /// Latest pan received from ROS
  float current_pan_;
  /// Latest tilt received from ROS
  float current_tilt_;
  /// Latest zoom received from ROS
  float current_zoom_;

  bool has_pan_target_;
  float pan_target_;

  bool has_tilt_target_;
  float tilt_target_;

  bool has_zoom_target_;
  float zoom_target_;

  // Mouse handling
  /// Is the left mouse button currently pressed?
  bool left_mouse_down_;
  /// Is the right mouse button currently pressed?
  bool right_mouse_down_;

  /// X pixel location at which the mouse was pressed (relative to the image panel)
  int32_t start_mouse_x_;
  /// Y pixel location at which the mouse was pressed (relative to the image panel)
  int32_t start_mouse_y_;
  /// X pixel location of the mouse right now (relative to the image panel)
  int32_t current_mouse_x_;
  /// Y pixel location of the mouse right now (relative to the image panel)
  int32_t current_mouse_y_;

  float pan_min_;
  float pan_max_;

  float tilt_min_;
  float tilt_max_;

  float zoom_min_;
  float zoom_max_;

  wxTimer zoom_scroll_timer_;

  bool minimal_;
};

#endif
