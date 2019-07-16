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
#include <string>
#include <utility>

#include "interactive_markers/interactive_marker_client.hpp"
#include "interactive_markers/detail/single_client.hpp"

using namespace std::placeholders;

namespace interactive_markers
{

InteractiveMarkerClient::InteractiveMarkerClient(
  rclcpp::node_interfaces::NodeTopicsInterface::SharedPtr topics_interface,
  rclcpp::node_interfaces::NodeGraphInterface::SharedPtr graph_interface,
  rclcpp::node_interfaces::NodeLoggingInterface::SharedPtr logging_interface,
  tf2::BufferCore & tf_buffer,
  const std::string & target_frame,
  const std::string & topic_ns)
: state_("InteractiveMarkerClient", IDLE),
  topics_interface_(topics_interface),
  graph_interface_(graph_interface),
  logger_(logging_interface->get_logger()),
  tf_buffer_(tf_buffer),
  last_num_publishers_(0),
  enable_autocomplete_transparency_(true)
{
  target_frame_ = target_frame;
  if (!topic_ns.empty()) {
    subscribe(topic_ns);
  }
  callbacks_.setStatusCb(std::bind(&InteractiveMarkerClient::statusCb, this, _1, _2, _3));
}

InteractiveMarkerClient::~InteractiveMarkerClient()
{
  shutdown();
}

/// Subscribe to given topic
void InteractiveMarkerClient::subscribe(std::string topic_ns)
{
  topic_ns_ = topic_ns;
  subscribeUpdate();
  subscribeInit();
}

void InteractiveMarkerClient::setInitCb(const InitCallback & cb)
{
  callbacks_.setInitCb(cb);
}

void InteractiveMarkerClient::setUpdateCb(const UpdateCallback & cb)
{
  callbacks_.setUpdateCb(cb);
}

void InteractiveMarkerClient::setResetCb(const ResetCallback & cb)
{
  callbacks_.setResetCb(cb);
}

void InteractiveMarkerClient::setStatusCb(const StatusCallback & cb)
{
  status_cb_ = cb;
}

void InteractiveMarkerClient::setTargetFrame(std::string target_frame)
{
  target_frame_ = target_frame;
  RCLCPP_DEBUG(logger_, "Target frame is now %s", target_frame_.c_str());

  switch (state_) {
    case IDLE:
      break;

    case INIT:
    case RUNNING:
      shutdown();
      subscribeUpdate();
      subscribeInit();
      break;
  }
}

void InteractiveMarkerClient::shutdown()
{
  switch (state_) {
    case IDLE:
      break;

    case INIT:
    case RUNNING:
      init_sub_.reset();
      update_sub_.reset();
      std::lock_guard<std::mutex> lock(publisher_contexts_mutex_);
      publisher_contexts_.clear();
      last_num_publishers_ = 0;
      state_ = IDLE;
      break;
  }
}

void InteractiveMarkerClient::subscribeUpdate()
{
  if (!topic_ns_.empty()) {
    try {
      rclcpp::QoS update_qos(rclcpp::KeepLast(100));
      update_sub_ = rclcpp::create_subscription<visualization_msgs::msg::InteractiveMarkerUpdate>(
        topics_interface_,
        topic_ns_ + "/update",
        update_qos,
        std::bind(&InteractiveMarkerClient::processUpdate, this, _1));
      RCLCPP_DEBUG(logger_, "Subscribed to update topic: %s", (topic_ns_ + "/update").c_str());
    } catch (rclcpp::exceptions::InvalidNodeError & ex) {
      callbacks_.statusCb(ERROR, "General", "Error subscribing: " + std::string(ex.what()));
      return;
    } catch (rclcpp::exceptions::NameValidationError & ex) {
      callbacks_.statusCb(ERROR, "General", "Error subscribing: " + std::string(ex.what()));
      return;
    }
  }
  callbacks_.statusCb(OK, "General", "Waiting for messages.");
}

void InteractiveMarkerClient::subscribeInit()
{
  if (state_ != INIT && !topic_ns_.empty()) {
    try {
      rclcpp::QoS init_qos(rclcpp::KeepLast(100));
      // TODO(jacobperron): Do we need this?
      // init_qos.transient_local().reliable();
      init_sub_ = rclcpp::create_subscription<visualization_msgs::msg::InteractiveMarkerInit>(
        topics_interface_,
        topic_ns_ + "/update_full",
        init_qos,
        std::bind(&InteractiveMarkerClient::processInit, this, _1));
      RCLCPP_DEBUG(logger_, "Subscribed to init topic: %s", (topic_ns_ + "/update_full").c_str());
      state_ = INIT;
    } catch (rclcpp::exceptions::InvalidNodeError & ex) {
      callbacks_.statusCb(ERROR, "General", "Error subscribing: " + std::string(ex.what()));
    } catch (rclcpp::exceptions::NameValidationError & ex) {
      callbacks_.statusCb(ERROR, "General", "Error subscribing: " + std::string(ex.what()));
    }
  }
}

template<class MsgConstPtrT>
void InteractiveMarkerClient::process(const MsgConstPtrT & msg)
{
  callbacks_.statusCb(OK, "General", "Receiving messages.");

  // get caller ID of the sending entity
  if (msg->server_id.empty()) {
    callbacks_.statusCb(ERROR, "General", "Received message with empty server_id!");
    return;
  }

  SingleClientPtr client;
  {
    std::lock_guard<std::mutex> lock(publisher_contexts_mutex_);

    M_SingleClient::iterator context_it = publisher_contexts_.find(msg->server_id);

    // If we haven't seen this publisher before, we need to reset the
    // display and listen to the init topic, plus of course add this
    // publisher to our list.
    if (context_it == publisher_contexts_.end()) {
      RCLCPP_DEBUG(logger_, "New publisher detected: %s", msg->server_id.c_str());

      SingleClientPtr pc(new SingleClient(msg->server_id, tf_buffer_, target_frame_, callbacks_));
      context_it = publisher_contexts_.insert(std::make_pair(msg->server_id, pc)).first;
      client = pc;

      // we need to subscribe to the init topic again
      subscribeInit();
    }

    client = context_it->second;
  }

  // forward init/update to respective context
  client->process(msg, enable_autocomplete_transparency_);
}

void InteractiveMarkerClient::processInit(
  visualization_msgs::msg::InteractiveMarkerInit::SharedPtr msg)
{
  process<visualization_msgs::msg::InteractiveMarkerInit::SharedPtr>(msg);
}

void InteractiveMarkerClient::processUpdate(
  visualization_msgs::msg::InteractiveMarkerUpdate::SharedPtr msg)
{
  process<visualization_msgs::msg::InteractiveMarkerUpdate::SharedPtr>(msg);
}

void InteractiveMarkerClient::update()
{
  switch (state_) {
    case IDLE:
      break;

    case INIT:
    case RUNNING:
      {
        // check if one publisher has gone offline
        const size_t num_publishers = graph_interface_->count_publishers(
          update_sub_->get_topic_name());
        if (num_publishers < last_num_publishers_) {
          callbacks_.statusCb(ERROR, "General", "Server is offline. Resetting.");
          shutdown();
          subscribeUpdate();
          subscribeInit();
          return;
        }
        last_num_publishers_ = num_publishers;

        // check if all single clients are finished with the init channels
        bool initialized = true;
        std::lock_guard<std::mutex> lock(publisher_contexts_mutex_);
        M_SingleClient::iterator it;
        for (it = publisher_contexts_.begin(); it != publisher_contexts_.end(); ++it) {
          // Explicitly reference the pointer to the client here, because the client
          // might call user code, which might call shutdown(), which will delete
          // the publisher_contexts_ map...

          SingleClientPtr single_client = it->second;
          single_client->update();
          if (!single_client->isInitialized()) {
            initialized = false;
          }

          if (publisher_contexts_.empty()) {
            break;  // Yep, someone called shutdown()...
          }
        }
        if (state_ == INIT && initialized) {
          init_sub_.reset();
          state_ = RUNNING;
        }
        if (state_ == RUNNING && !initialized) {
          subscribeInit();
        }
        break;
      }
  }
}

void InteractiveMarkerClient::statusCb(
  StatusT status, const std::string & server_id,
  const std::string & msg)
{
  switch (status) {
    case OK:
      RCLCPP_DEBUG(logger_, "%s: %s (Status: OK)", server_id.c_str(), msg.c_str());
      break;
    case WARN:
      RCLCPP_DEBUG(logger_, "%s: %s (Status: WARNING)", server_id.c_str(), msg.c_str());
      break;
    case ERROR:
      RCLCPP_DEBUG(logger_, "%s: %s (Status: ERROR)", server_id.c_str(), msg.c_str());
      break;
  }

  if (status_cb_) {
    status_cb_(status, server_id, msg);
  }
}

}  // namespace interactive_markers
