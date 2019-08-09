// Copyright (c) 2011, Willow Garage, Inc.
// Copyright (c) 2019, Open Source Robotics Foundation, Inc.
// All rights reserved.
//
// Software License Agreement (BSD License 2.0)
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions
// are met:
//
//  * Redistributions of source code must retain the above copyright
//    notice, this list of conditions and the following disclaimer.
//  * Redistributions in binary form must reproduce the above
//    copyright notice, this list of conditions and the following
//    disclaimer in the documentation and/or other materials provided
//    with the distribution.
//  * Neither the name of Willow Garage, Inc. nor the names of its
//    contributors may be used to endorse or promote products derived
//    from this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
// FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
// COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
// INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
// BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
// CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
// LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
// ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

// Author: David Gossow

#ifndef INTERACTIVE_MARKERS__INTERACTIVE_MARKER_CLIENT_HPP_
#define INTERACTIVE_MARKERS__INTERACTIVE_MARKER_CLIENT_HPP_

#include <deque>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <unordered_map>

#include "rclcpp/logger.hpp"
#include "rclcpp/rclcpp.hpp"

#include "visualization_msgs/msg/interactive_marker_update.hpp"
#include "visualization_msgs/srv/get_interactive_markers.hpp"

#include "detail/message_context.hpp"
#include "detail/state_machine.hpp"

namespace tf2
{
class BufferCoreInterface;
}

namespace interactive_markers
{

/// Acts as a client to one or multiple Interactive Marker servers.
/**
 * Handles topic subscription, error detection and tf transformations.
 *
 * After connecting to a provided namespace, the client sends a service request
 * to an available interactive marker server to get an initial set of markers.
 * Once initialized, feedback messages may be sent from the client to the server
 * as well as update messages received from the server.
 *
 * In case of an error (e.g. update message loss or tf failure), the connection
 * to the server is reset.
 *
 * All timestamped messages are being transformed into the target frame,
 * while for non-timestamped messages it is ensured that the necessary
 * tf transformation will be available.
 */
class InteractiveMarkerClient
{
public:
  enum Status
  {
    DEBUG = 0,
    INFO,
    WARN,
    ERROR
  };

  enum State
  {
    IDLE,
    INITIALIZE,
    RUNNING
  };

  typedef std::function<void (visualization_msgs::msg::InteractiveMarkerUpdate::SharedPtr)>
    UpdateCallback;
  typedef std::function<void (visualization_msgs::srv::GetInteractiveMarkers::Response::SharedPtr)>
    InitializeCallback;
  typedef std::function<void (const std::string &)> ResetCallback;
  typedef std::function<void (const Status, const std::string &)> StatusCallback;

  /// Constructor.
  /**
   * \param node_base_interface The node base interface for creating the service client.
   * \param topics_interface The node topics interface for creating publishers and subscriptions.
   * \param graph_interface The node graph interface for querying the ROS graph.
   * \param logging_interface The node logging interface for logging messages.
   * \param tf_buffer_core The tf transformer to use.
   * \param target_frame The tf frame to transform timestamped messages into.
   * \param topic_namespace The interactive marker topic namespace.
   *   This is the namespace used for the underlying ROS service and topics for communication
   *   between the client and server.
   *   If the namespace is not empty, then connect() is called.
   */
  InteractiveMarkerClient(
    rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node_base_interface,
    rclcpp::node_interfaces::NodeTopicsInterface::SharedPtr topics_interface,
    rclcpp::node_interfaces::NodeServicesInterface::SharedPtr services_interface,
    rclcpp::node_interfaces::NodeGraphInterface::SharedPtr graph_interface,
    rclcpp::node_interfaces::NodeLoggingInterface::SharedPtr logging_interface,
    std::shared_ptr<tf2::BufferCoreInterface> tf_buffer_core,
    const std::string & target_frame = "",
    const std::string & topic_namespace = "");

  /// Constructor.
  /**
   * \param node The object from which to get the desired node interfaces.
   * \param tf_buffer_core The tf transformer to use.
   * \param target_frame The tf frame to transform timestamped messages into.
   * \param topic_namespace The interactive marker topic namespace.
   *   This is the namespace used for the underlying ROS service and topics for communication
   *   between the client and server.
   *   If the namespace is not empty, then connect() is called.
   */
  template<typename NodePtr>
  InteractiveMarkerClient(
    NodePtr node,
    std::shared_ptr<tf2::BufferCoreInterface> tf_buffer_core,
    const std::string & target_frame = "",
    const std::string & topic_namespace = "")
  : InteractiveMarkerClient(
      node->get_node_base_interface(),
      node->get_node_topics_interface(),
      node->get_node_services_interface(),
      node->get_node_graph_interface(),
      node->get_node_logging_interface(),
      tf_buffer_core,
      target_frame,
      topic_namespace)
  {
  }

  /// Destructor.
  /**
   * Calls reset().
   */
  ~InteractiveMarkerClient();

  /// Connect to a server in a given namespace.
  void connect(std::string topic_namespace);

  /// Disconnect from a server and clear the update queue.
  void disconnect();

  /// Update the internal state and call registered callbacks.
  void update();

  /// Change the target frame.
  /**
   * This resets the connection.
   */
  void setTargetFrame(std::string target_frame);

  /// Set the initialization callback.
  /**
   * The registered function is called when the client successfully initializes with a connected
   * server.
   */
  void setInitializeCallback(const InitializeCallback & cb);

  /// Set the callback for update messages.
  /**
   * If the client is connected and initialized, the registered function is called whenever an
   * update message is received.
   */
  void setUpdateCallback(const UpdateCallback & cb);

  /// Set the reset callback.
  /**
   * The registered function is called whenver the connection is reset.
   */
  void setResetCallback(const ResetCallback & cb);

  /// Set the callback for status updates.
  /**
   * The registered function is called whenever there is a status message.
   */
  void setStatusCallback(const StatusCallback & cb);

  inline void setEnableAutocompleteTransparency(bool enable)
  {
    enable_autocomplete_transparency_ = enable;
  }

  inline State getState() const
  {
    return static_cast<State>(state_);
  }

private:
  typedef MessageContext<visualization_msgs::srv::GetInteractiveMarkers::Response>
    InitialMessageContext;
  typedef MessageContext<visualization_msgs::msg::InteractiveMarkerUpdate> UpdateMessageContext;

  // Disable copying
  InteractiveMarkerClient(const InteractiveMarkerClient &) = delete;
  InteractiveMarkerClient & operator=(const InteractiveMarkerClient &) = delete;

  // Clear messages from the update queue
  void reset();

  // Make a service request to get interactive markers from a server
  void requestInteractiveMarkers();

  void processInitialMessage(
    rclcpp::Client<visualization_msgs::srv::GetInteractiveMarkers>::SharedFuture future);

  void processUpdate(
    visualization_msgs::msg::InteractiveMarkerUpdate::SharedPtr msg);

  bool transformInitialMessage();

  bool transformUpdateMessages();

  bool checkInitializeFinished();

  void pushUpdates();

  void changeState(const State & new_state);

  void updateStatus(const Status status, const std::string & msg);

  // Node interfaces
  rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node_base_interface_;
  rclcpp::node_interfaces::NodeTopicsInterface::SharedPtr topics_interface_;
  rclcpp::node_interfaces::NodeServicesInterface::SharedPtr services_interface_;
  rclcpp::node_interfaces::NodeGraphInterface::SharedPtr graph_interface_;

  rclcpp::Logger logger_;

  StateMachine<State> state_;

  rclcpp::Client<visualization_msgs::srv::GetInteractiveMarkers>::SharedPtr
    get_interactive_markers_client_;

  rclcpp::SubscriptionBase::SharedPtr update_sub_;

  std::shared_ptr<tf2::BufferCoreInterface> tf_buffer_core_;

  std::string target_frame_;

  std::string topic_namespace_;

  std::recursive_mutex mutex_;

  // The response message from the request to get interactive markers
  std::shared_ptr<InitialMessageContext> initial_response_msg_;

  // Queue of update messages from the server
  std::deque<UpdateMessageContext> update_queue_;

  // true if no updates have been received since the last response message
  bool first_update_;

  // Sequence number last update message received
  uint64_t last_update_sequence_number_;

  // if false, auto-completed markers will have alpha = 1.0
  bool enable_autocomplete_transparency_;

  // User callbacks
  InitializeCallback initialize_callback_;
  UpdateCallback update_callback_;
  ResetCallback reset_callback_;
  StatusCallback status_callback_;
};  // class InteractiveMarkerClient

}  // namespace interactive_markers

#endif  // INTERACTIVE_MARKERS__INTERACTIVE_MARKER_CLIENT_HPP_
