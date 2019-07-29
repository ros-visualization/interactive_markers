// Copyright (c) 2011, Willow Garage, Inc.
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

#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <unordered_map>

#include "rclcpp/logger.hpp"
#include "rclcpp/rclcpp.hpp"

#include "visualization_msgs/msg/interactive_marker_update.hpp"
#include "visualization_msgs/srv/get_interactive_markers.hpp"

#include "detail/state_machine.hpp"

namespace tf2
{
class BufferCoreInterface;
}

namespace interactive_markers
{

class SingleClient;

/// Acts as a client to one or multiple Interactive Marker servers.
/// Handles topic subscription, error detection and tf transformations.
///
/// The output is an init message followed by a stream of updates
/// for each server. In case of an error (e.g. message loss, tf failure),
/// the connection to the sending server is reset.
///
/// All timestamped messages are being transformed into the target frame,
/// while for non-timestamped messages it is ensured that the necessary
/// tf transformation will be available.
class InteractiveMarkerClient
{
public:
  enum StatusT
  {
    OK = 0,
    WARN = 1,
    ERROR = 2
  };

  enum StateT
  {
    IDLE,
    INIT,
    RUNNING
  };

  typedef std::function<void (visualization_msgs::msg::InteractiveMarkerUpdate::SharedPtr)>
    UpdateCallback;
  typedef std::function<void (visualization_msgs::srv::GetInteractiveMarkers::Response::SharedPtr)>
    InitializeCallback;
  typedef std::function<void (const std::string &)> ResetCallback;
  typedef std::function<void (StatusT, const std::string &, const std::string &)> StatusCallback;

  /// @param tf           The tf transformer to use.
  /// @param target_frame tf frame to transform timestamped messages into.
  /// @param topic_ns     The topic namespace (will subscribe to topic_ns/update, topic_ns/init)
  InteractiveMarkerClient(
    rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node_base_interface,
    rclcpp::node_interfaces::NodeTopicsInterface::SharedPtr topics_interface,
    rclcpp::node_interfaces::NodeServicesInterface::SharedPtr services_interface,
    rclcpp::node_interfaces::NodeGraphInterface::SharedPtr graph_interface,
    rclcpp::node_interfaces::NodeLoggingInterface::SharedPtr logging_interface,
    std::shared_ptr<tf2::BufferCoreInterface> tf_buffer_core,
    const std::string & target_frame = "",
    const std::string & topic_ns = "");

  template<typename NodePtr>
  InteractiveMarkerClient(
    NodePtr node,
    std::shared_ptr<tf2::BufferCoreInterface> tf_buffer_core,
    const std::string & target_frame = "",
    const std::string & topic_ns = "")
  : InteractiveMarkerClient(
      node->get_node_base_interface(),
      node->get_node_topics_interface(),
      node->get_node_services_interface(),
      node->get_node_graph_interface(),
      node->get_node_logging_interface(),
      tf_buffer_core,
      target_frame,
      topic_ns)
  {
  }

  /// Will cause a 'reset' call for all server ids
  ~InteractiveMarkerClient();

  /// Subscribe to the topics topic_ns/update and topic_ns/init
  void subscribe(std::string topic_ns);

  /// Unsubscribe, clear queues & call reset callbacks
  void shutdown();

  /// Update tf info, call callbacks
  void update();

  /// Change the target frame and reset the connection
  void setTargetFrame(std::string target_frame);

  /// Set callback for init messages
  void setInitializeCallback(const InitializeCallback & cb);

  /// Set callback for update messages
  void setUpdateCallback(const UpdateCallback & cb);

  /// Set callback for resetting one server connection
  void setResetCallback(const ResetCallback & cb);

  /// Set callback for status updates
  void setStatusCallback(const StatusCallback & cb);

  inline void setEnableAutocompleteTransparency(bool enable)
  {
    enable_autocomplete_transparency_ = enable;
  }

  inline StateT getState() const
  {
    return static_cast<StateT>(state_);
  }

private:
  // Process message from the init or update channel
  template<class MsgSharedPtrT>
  void process(const MsgSharedPtrT msg);

  rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node_base_interface_;
  rclcpp::node_interfaces::NodeTopicsInterface::SharedPtr topics_interface_;
  rclcpp::node_interfaces::NodeServicesInterface::SharedPtr services_interface_;
  rclcpp::node_interfaces::NodeGraphInterface::SharedPtr graph_interface_;
  rclcpp::Logger logger_;

  StateMachine<StateT> state_;

  std::string topic_ns_;

  rclcpp::Client<visualization_msgs::srv::GetInteractiveMarkers>::SharedPtr
    get_interactive_markers_client_;
  rclcpp::SubscriptionBase::SharedPtr update_sub_;

  void requestInteractiveMarkers();

  // subscribe to the init channel
  void subscribeUpdate();

  void statusCallback(StatusT status, const std::string & server_id, const std::string & msg);

  typedef std::shared_ptr<SingleClient> SingleClientPtr;
  typedef std::unordered_map<std::string, SingleClientPtr> M_SingleClient;
  M_SingleClient publisher_contexts_;
  std::mutex publisher_contexts_mutex_;

  std::shared_ptr<tf2::BufferCoreInterface> tf_buffer_core_;
  std::string target_frame_;

// TODO(jacobperron): protected
public:
  // for internal usage
  struct Callbacks
  {
    void initializeCallback(
      visualization_msgs::srv::GetInteractiveMarkers::Response::SharedPtr response) const
    {
      if (initialize_callback_) {
        initialize_callback_(response);
      }
    }

    void updateCallback(visualization_msgs::msg::InteractiveMarkerUpdate::SharedPtr update) const
    {
      if (update_callback_) {
        update_callback_(update);
      }
    }

    void resetCallback(const std::string & message) const
    {
      if (reset_callback_) {
        reset_callback_(message);
      }
    }

    void statusCallback(StatusT status, const std::string & id, const std::string & message) const
    {
      if (status_callback_) {
        status_callback_(status, id, message);
      }
    }

    void setInitializeCallback(InitializeCallback initialize_callback)
    {
      initialize_callback_ = initialize_callback;
    }

    void setUpdateCallback(UpdateCallback update_callback)
    {
      update_callback_ = update_callback;
    }

    void setResetCallback(ResetCallback reset_callback)
    {
      reset_callback_ = reset_callback;
    }

    void setStatusCallback(StatusCallback status_callback)
    {
      status_callback_ = status_callback;
    }

private:
    InitializeCallback initialize_callback_;
    UpdateCallback update_callback_;
    ResetCallback reset_callback_;
    StatusCallback status_callback_;
  };  // struct Callbacks

  // handle update message
  void processUpdate(const visualization_msgs::msg::InteractiveMarkerUpdate::SharedPtr msg);

private:
  // Disable copying
  InteractiveMarkerClient(const InteractiveMarkerClient &) = delete;
  InteractiveMarkerClient & operator=(const InteractiveMarkerClient &) = delete;

  Callbacks callbacks_;

  // this is the real (external) status callback
  StatusCallback status_cb_;

  // this allows us to detect if a server died (in most cases)
  size_t last_num_publishers_;

  // if server is ready
  bool server_ready_;

  // if false, auto-completed markers will have alpha = 1.0
  bool enable_autocomplete_transparency_;
};  // class InteractiveMarkerClient

}  // namespace interactive_markers

#endif  // INTERACTIVE_MARKERS__INTERACTIVE_MARKER_CLIENT_HPP_
