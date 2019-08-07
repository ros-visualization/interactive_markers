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

#include <memory>
#include <mutex>
#include <string>
#include <utility>

#include "visualization_msgs/srv/get_interactive_markers.hpp"

#include "interactive_markers/interactive_marker_client.hpp"

using namespace std::placeholders;

namespace interactive_markers
{

const size_t MAX_UPDATE_QUEUE_SIZE = 100u;

InteractiveMarkerClient::InteractiveMarkerClient(
  rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node_base_interface,
  rclcpp::node_interfaces::NodeTopicsInterface::SharedPtr topics_interface,
  rclcpp::node_interfaces::NodeServicesInterface::SharedPtr services_interface,
  rclcpp::node_interfaces::NodeGraphInterface::SharedPtr graph_interface,
  rclcpp::node_interfaces::NodeLoggingInterface::SharedPtr logging_interface,
  std::shared_ptr<tf2::BufferCoreInterface> tf_buffer_core,
  const std::string & target_frame,
  const std::string & topic_namespace)
: node_base_interface_(node_base_interface),
  topics_interface_(topics_interface),
  services_interface_(services_interface),
  graph_interface_(graph_interface),
  logger_(logging_interface->get_logger()),
  state_("InteractiveMarkerClient", IDLE),
  tf_buffer_core_(tf_buffer_core),
  target_frame_(target_frame),
  topic_namespace_(topic_namespace),
  initial_response_msg_(0),
  first_update_(true),
  last_update_sequence_number_(0u),
  enable_autocomplete_transparency_(true)
{
  connect(topic_namespace_);
}

InteractiveMarkerClient::~InteractiveMarkerClient()
{
  disconnect();
}

void InteractiveMarkerClient::connect(std::string topic_namespace)
{
  changeState(IDLE);
  topic_namespace_ = topic_namespace;

  // Terminate any existing connection
  disconnect();

  // Don't do anything if no namespace is provided
  if (topic_namespace_.empty()) {
    return;
  }

  get_interactive_markers_client_ =
    rclcpp::create_client<visualization_msgs::srv::GetInteractiveMarkers>(
    node_base_interface_,
    graph_interface_,
    services_interface_,
    topic_namespace_ + "/get_interactive_markers",
    rmw_qos_profile_services_default,
    nullptr);

  try {
    rclcpp::QoS update_qos(rclcpp::KeepLast(100));
    update_sub_ = rclcpp::create_subscription<visualization_msgs::msg::InteractiveMarkerUpdate>(
      topics_interface_,
      topic_namespace_ + "/update",
      update_qos,
      std::bind(&InteractiveMarkerClient::processUpdate, this, _1));
  } catch (rclcpp::exceptions::InvalidNodeError & ex) {
    updateStatus(ERROR, "Error subscribing: " + std::string(ex.what()));
    return;
  } catch (rclcpp::exceptions::NameValidationError & ex) {
    updateStatus(ERROR, "Error subscribing: " + std::string(ex.what()));
    return;
  }

  updateStatus(INFO, "Connected on namespace: " + topic_namespace_);
}

void InteractiveMarkerClient::disconnect()
{
  get_interactive_markers_client_.reset();
  update_sub_.reset();
  reset();
}

void InteractiveMarkerClient::update()
{
  if (!get_interactive_markers_client_) {
    // Disconnected
    updateStatus(WARN, "Update called when disconnected");
    return;
  }

  const bool server_ready = get_interactive_markers_client_->service_is_ready();

  switch (state_) {
    case IDLE:
      if (server_ready) {
        changeState(INITIALIZE);
      }
      break;

    case INITIALIZE:
      if (!server_ready) {
        updateStatus(WARN, "Server not available during initialization, resetting");
        changeState(IDLE);
        break;
      }
      // If there's an unexpected error, reset
      if (!transformInitialMessage()) {
        changeState(IDLE);
        break;
      }
      if (checkInitializeFinished()) {
        changeState(RUNNING);
      }
      break;

    case RUNNING:
      if (!server_ready) {
        updateStatus(WARN, "Server not available while running, resetting");
        changeState(IDLE);
        break;
      }
      // If there's an unexpected error, reset
      if (!transformUpdateMessages()) {
        changeState(IDLE);
        break;
      }
      pushUpdates();
      break;

    default:
      updateStatus(ERROR, "Invalid state in update: " + std::to_string(getState()));
  }
}

void InteractiveMarkerClient::setTargetFrame(std::string target_frame)
{
  if (target_frame_ == target_frame) {
    return;
  }

  target_frame_ = target_frame;
  updateStatus(INFO, "Target frame is now " + target_frame_);

  // TODO(jacobperron): Maybe we can do better, efficiency-wise
  changeState(IDLE);
}

void InteractiveMarkerClient::setInitializeCallback(const InitializeCallback & cb)
{
  initialize_callback_ = cb;
}

void InteractiveMarkerClient::setUpdateCallback(const UpdateCallback & cb)
{
  update_callback_ = cb;
}

void InteractiveMarkerClient::setResetCallback(const ResetCallback & cb)
{
  reset_callback_ = cb;
}

void InteractiveMarkerClient::setStatusCallback(const StatusCallback & cb)
{
  status_callback_ = cb;
}

void InteractiveMarkerClient::reset()
{
  std::unique_lock<std::recursive_mutex> lock(mutex_);
  state_ = IDLE;
  first_update_ = true;
  initial_response_msg_.reset();
  update_queue_.clear();
}

void InteractiveMarkerClient::requestInteractiveMarkers()
{
  if (!get_interactive_markers_client_) {
    updateStatus(ERROR, "Interactive markers requested when client is disconnected");
    return;
  }
  if (!get_interactive_markers_client_->wait_for_service(std::chrono::seconds(1))) {
    updateStatus(WARN, "Service is not ready during request for interactive markers");
    return;
  }
  updateStatus(DEBUG, "Sending request for interactive markers");

  auto callback = std::bind(&InteractiveMarkerClient::processInitialMessage, this, _1);
  auto request = std::make_shared<visualization_msgs::srv::GetInteractiveMarkers::Request>();
  get_interactive_markers_client_->async_send_request(
    request,
    callback);
}

void InteractiveMarkerClient::processInitialMessage(
  rclcpp::Client<visualization_msgs::srv::GetInteractiveMarkers>::SharedFuture future)
{
  updateStatus(INFO, "Service response received for initialization");
  auto response = future.get();
  {
    std::unique_lock<std::recursive_mutex> lock(mutex_);
    initial_response_msg_ = std::make_shared<InitialMessageContext>(
      tf_buffer_core_, target_frame_, response, enable_autocomplete_transparency_);
  }
}

void InteractiveMarkerClient::processUpdate(
  visualization_msgs::msg::InteractiveMarkerUpdate::SharedPtr msg)
{
  // Ignore legacy "keep alive" messages
  if (msg->type == msg->KEEP_ALIVE) {
    RCLCPP_WARN_ONCE(
      logger_,
      "KEEP_ALIVE message ignored. "
      "Servers are no longer expected to publish this type of message.");
    return;
  }

  if (!first_update_ && msg->seq_num != last_update_sequence_number_ + 1) {
    std::ostringstream oss;
    oss << "Update sequence number is out of order. " << last_update_sequence_number_ + 1 <<
      " (expected) vs. " << msg->seq_num << " (received)";
    updateStatus(WARN, oss.str());
    // Change state to IDLE to cause reset
    changeState(IDLE);
    return;
  }

  updateStatus(DEBUG, "Received update with sequence number " + std::to_string(msg->seq_num));

  first_update_ = false;
  last_update_sequence_number_ = msg->seq_num;

  if (update_queue_.size() > MAX_UPDATE_QUEUE_SIZE) {
    updateStatus(
      WARN,
      "Update queue too large. Erasing message with sequence number " +
      std::to_string(update_queue_.begin()->msg->seq_num));
    update_queue_.pop_back();
  }

  update_queue_.push_front(UpdateMessageContext(
      tf_buffer_core_, target_frame_, msg, enable_autocomplete_transparency_));
}

bool InteractiveMarkerClient::transformInitialMessage()
{
  std::unique_lock<std::recursive_mutex> lock(mutex_);
  if (!initial_response_msg_) {
    // We haven't received a response yet
    return true;
  }

  try {
    initial_response_msg_->getTfTransforms();
    // TODO(jacobperron): Use custom exception
  } catch (std::runtime_error & e) {
    std::ostringstream oss;
    oss << "Resetting due to tf error: " << e.what();
    updateStatus(ERROR, oss.str());
    return false;
  } catch (...) {
    std::ostringstream oss;
    oss << "Resetting due to unknown exception";
    updateStatus(ERROR, oss.str());
    return false;
  }

  return true;
}

bool InteractiveMarkerClient::transformUpdateMessages()
{
  std::unique_lock<std::recursive_mutex> lock(mutex_);
  for (auto it = update_queue_.begin(); it != update_queue_.end(); ++it) {
    try {
      it->getTfTransforms();
      // TODO(jacobperron): Use custom exception
    } catch (std::runtime_error & e) {
      std::ostringstream oss;
      oss << "Resetting due to tf error: " << e.what();
      updateStatus(ERROR, oss.str());
      return false;
    } catch (...) {
      std::ostringstream oss;
      oss << "Resetting due to unknown exception";
      updateStatus(ERROR, oss.str());
      return false;
    }
  }
  return true;
}

bool InteractiveMarkerClient::checkInitializeFinished()
{
  std::unique_lock<std::recursive_mutex> lock(mutex_);
  if (!initial_response_msg_) {
    // We haven't received a response yet
    return false;
  }

  const uint64_t & response_sequence_number = initial_response_msg_->msg->sequence_number;
  if (!initial_response_msg_->isReady()) {
    updateStatus(DEBUG, "Initialization: Waiting for TF info");
    return false;
  }

  // Prune old update messages
  while (!update_queue_.empty() && update_queue_.back().msg->seq_num <= response_sequence_number) {
    updateStatus(
      DEBUG,
      "Omitting update with sequence number " + std::to_string(update_queue_.back().msg->seq_num));
    update_queue_.pop_back();
  }

  if (initialize_callback_) {
    initialize_callback_(initial_response_msg_->msg);
  }

  updateStatus(DEBUG, "Initialized");

  return true;
}

void InteractiveMarkerClient::pushUpdates()
{
  std::unique_lock<std::recursive_mutex> lock(mutex_);
  while (!update_queue_.empty() && update_queue_.back().isReady()) {
    visualization_msgs::msg::InteractiveMarkerUpdate::SharedPtr msg = update_queue_.back().msg;
    updateStatus(DEBUG, "Pushing update with sequence number " + std::to_string(msg->seq_num));
    if (update_callback_) {
      update_callback_(msg);
    }
    update_queue_.pop_back();
  }
}

void InteractiveMarkerClient::changeState(const State & new_state)
{
  if (state_ == new_state) {
    return;
  }

  updateStatus(DEBUG, "Change state to: " + std::to_string(new_state));

  switch (new_state) {
    case IDLE:
      reset();
      break;

    case INITIALIZE:
      requestInteractiveMarkers();
      break;

    case RUNNING:
      break;

    default:
      updateStatus(ERROR, "Invalid state when changing state: " + std::to_string(new_state));
      return;
  }
  state_ = new_state;
}

void InteractiveMarkerClient::updateStatus(const Status status, const std::string & msg)
{
  switch (status) {
    case DEBUG:
      RCLCPP_DEBUG(logger_, "%s", msg.c_str());
      break;
    case INFO:
      RCLCPP_INFO(logger_, "%s", msg.c_str());
      break;
    case WARN:
      RCLCPP_WARN(logger_, "%s", msg.c_str());
      break;
    case ERROR:
      RCLCPP_ERROR(logger_, "%s", msg.c_str());
      break;
  }

  if (status_callback_) {
    status_callback_(status, msg);
  }
}

}  // namespace interactive_markers
