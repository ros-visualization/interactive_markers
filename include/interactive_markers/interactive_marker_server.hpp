/*
 * Copyright (c) 2011, Willow Garage, Inc.
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
 *
 * Author: David Gossow
 */

#ifndef INTERACTIVE_MARKER_SERVER
#define INTERACTIVE_MARKER_SERVER

#include <visualization_msgs/msg/interactive_marker_init.hpp>
#include <visualization_msgs/msg/interactive_marker_feedback.hpp>
#include <visualization_msgs/msg/interactive_marker_update.hpp>

#include <functional>
#include <mutex>
#include <thread>
#include <unordered_map>

#include <rclcpp/rclcpp.hpp>

// #include <boost/scoped_ptr.hpp>
// #include <boost/thread/thread.hpp>
// #include <boost/thread/recursive_mutex.hpp>

// #include <ros/ros.h>
// #include <ros/callback_queue.h>
//
//
// #include <boost/function.hpp>
// #include <boost/unordered_map.hpp>

namespace interactive_markers
{

/// Acts as a server to one or many GUIs (e.g. rviz) displaying a set of interactive markers
///
/// Note: Keep in mind that changes made by calling insert(), erase(), setCallback() etc.
///       are not applied until calling applyChanges().
class InteractiveMarkerServer
{
public:
  typedef visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr FeedbackConstPtr;
  typedef std::function<void ( FeedbackConstPtr )> FeedbackCallback;

  static const uint8_t DEFAULT_FEEDBACK_CB = 255;

  /// @param topic_ns      The interface will use the topics topic_ns/update and
  ///                      topic_ns/feedback for communication.
  /// @param server_id     If you run multiple servers on the same topic from
  ///                      within the same node, you will need to assign different names to them.
  ///                      Otherwise, leave this empty.
  InteractiveMarkerServer(
    const std::string & topic_ns,
    rclcpp::node_interfaces::NodeBaseInterface::SharedPtr base_interface,
    rclcpp::node_interfaces::NodeClockInterface::SharedPtr clock_interface,
    rclcpp::node_interfaces::NodeLoggingInterface::SharedPtr logging_interface,
    rclcpp::node_interfaces::NodeTimersInterface::SharedPtr timers_interface,
    rclcpp::node_interfaces::NodeTopicsInterface::SharedPtr topics_interface,
    const std::string server_id);

  template<typename NodePtr>
  InteractiveMarkerServer(
    const std::string & topic_ns, NodePtr node,
    const std::string & server_id = "")
  : InteractiveMarkerServer(
      topic_ns,
      node->get_node_base_interface(),
      node->get_node_clock_interface(),
      node->get_node_logging_interface(),
      node->get_node_timers_interface(),
      node->get_node_topics_interface(),
      server_id)
  {
  }

  /// Destruction of the interface will lead to all managed markers being cleared.
  ~InteractiveMarkerServer();

  /// Add or replace a marker without changing its callback functions.
  /// Note: Changes to the marker will not take effect until you call applyChanges().
  /// The callback changes immediately.
  /// @param int_marker     The marker to be added or replaced
  void insert(const visualization_msgs::msg::InteractiveMarker & int_marker);

  /// Add or replace a marker and its callback functions
  /// Note: Changes to the marker will not take effect until you call applyChanges().
  /// The callback changes immediately.
  /// @param int_marker     The marker to be added or replaced
  /// @param feedback_cb    Function to call on the arrival of a feedback message.
  /// @param feedback_type  Type of feedback for which to call the feedback.
  void insert(
    const visualization_msgs::msg::InteractiveMarker & int_marker,
    FeedbackCallback feedback_cb,
    uint8_t feedback_type = DEFAULT_FEEDBACK_CB);

  /// Update the pose of a marker with the specified name
  /// Note: This change will not take effect until you call applyChanges()
  /// @return true if a marker with that name exists
  /// @param name    Name of the interactive marker
  /// @param pose    The new pose
  /// @param header  Header replacement. Leave this empty to use the previous one.
  bool setPose(
    const std::string & name,
    const geometry_msgs::msg::Pose & pose,
    const std_msgs::msg::Header & header = std_msgs::msg::Header() );

  /// Erase the marker with the specified name
  /// Note: This change will not take effect until you call applyChanges().
  /// @return true if a marker with that name exists
  /// @param name  Name of the interactive marker
  bool erase(const std::string & name);

  /// Clear all markers.
  /// Note: This change will not take effect until you call applyChanges().
  void clear();

  /// Return whether the server contains any markers.
  /// Note: Does not include markers inserted since the last applyChanges().
  /// @return true if the server contains no markers
  bool empty() const;

  /// Return the number of markers contained in the server
  /// Note: Does not include markers inserted since the last applyChanges().
  /// @return The number of markers contained in the server
  std::size_t size() const;

  /// Add or replace a callback function for the specified marker.
  /// Note: This change will not take effect until you call applyChanges().
  /// The server will try to call any type-specific callback first.
  /// If none is set, it will call the default callback.
  /// If a callback for the given type already exists, it will be replaced.
  /// To unset a type-specific callback, pass in an empty one.
  /// @param name           Name of the interactive marker
  /// @param feedback_cb    Function to call on the arrival of a feedback message.
  /// @param feedback_type  Type of feedback for which to call the feedback.
  ///                       Leave this empty to make this the default callback.
  bool setCallback(
    const std::string & name, FeedbackCallback feedback_cb,
    uint8_t feedback_type = DEFAULT_FEEDBACK_CB);

  /// Apply changes made since the last call to this method &
  /// broadcast an update to all clients.
  void applyChanges();

  /// Get marker by name
  /// @param name             Name of the interactive marker
  /// @param[out] int_marker  Output message
  /// @return true if a marker with that name exists
  bool get(std::string name, visualization_msgs::msg::InteractiveMarker & int_marker) const;

private:
  // Disable copying
  InteractiveMarkerServer(const InteractiveMarkerServer &) = delete;
  InteractiveMarkerServer & operator=(const InteractiveMarkerServer &) = delete;

  struct MarkerContext
  {
    rclcpp::Time last_feedback;
    std::string last_client_id;
    FeedbackCallback default_feedback_cb;
    std::unordered_map<uint8_t, FeedbackCallback> feedback_cbs;
    visualization_msgs::msg::InteractiveMarker int_marker;
  };

  typedef std::unordered_map<std::string, MarkerContext> M_MarkerContext;

  // represents an update to a single marker
  struct UpdateContext
  {
    enum
    {
      FULL_UPDATE,
      POSE_UPDATE,
      ERASE
    } update_type;
    visualization_msgs::msg::InteractiveMarker int_marker;
    FeedbackCallback default_feedback_cb;
    std::unordered_map<uint8_t, FeedbackCallback> feedback_cbs;
  };

  typedef std::unordered_map<std::string, UpdateContext> M_UpdateContext;

  // update marker pose & call user callback
  void processFeedback(visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr feedback);

  // send an empty update to keep the client GUIs happy
  void keepAlive();

  // increase sequence number & publish an update
  void publish(visualization_msgs::msg::InteractiveMarkerUpdate & update);

  // publish the current complete state to the latched "init" topic.
  void publishInit();

  // Update pose, schedule update without locking
  void doSetPose(
    M_UpdateContext::iterator update_it,
    const std::string & name,
    const geometry_msgs::msg::Pose & pose,
    const std_msgs::msg::Header & header);

  // contains the current state of all markers
  M_MarkerContext marker_contexts_;

  // updates that have to be sent on the next publish
  M_UpdateContext pending_updates_;

  // topic namespace to use
  std::string topic_ns_;

  mutable std::recursive_mutex mutex_;

  // this is needed when running in non-threaded mode
  rclcpp::TimerBase::SharedPtr keep_alive_timer_;

  rclcpp::Publisher<visualization_msgs::msg::InteractiveMarkerInit>::SharedPtr init_pub_;
  rclcpp::Publisher<visualization_msgs::msg::InteractiveMarkerUpdate>::SharedPtr update_pub_;
  rclcpp::Subscription<visualization_msgs::msg::InteractiveMarkerFeedback>::SharedPtr feedback_sub_;

  rclcpp::Context::SharedPtr context_;
  rclcpp::Clock::SharedPtr clock_;
  rclcpp::Logger logger_;

  uint64_t seq_num_;

  std::string server_id_;
};

}

#endif
