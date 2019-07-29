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
#include <unordered_map>
#include <utility>

#include "interactive_markers/interactive_marker_server.hpp"

#include "rmw/rmw.h"
#include "rclcpp/qos.hpp"

using visualization_msgs::msg::InteractiveMarkerFeedback;
using visualization_msgs::msg::InteractiveMarkerUpdate;

namespace interactive_markers
{

InteractiveMarkerServer::InteractiveMarkerServer(
  const std::string & topic_ns,
  rclcpp::node_interfaces::NodeBaseInterface::SharedPtr base_interface,
  rclcpp::node_interfaces::NodeClockInterface::SharedPtr clock_interface,
  rclcpp::node_interfaces::NodeLoggingInterface::SharedPtr logging_interface,
  rclcpp::node_interfaces::NodeTimersInterface::SharedPtr timers_interface,
  rclcpp::node_interfaces::NodeTopicsInterface::SharedPtr topics_interface,
  rclcpp::node_interfaces::NodeServicesInterface::SharedPtr services_interface,
  const std::string server_id)
: topic_ns_(topic_ns),
  seq_num_(0),
  context_(base_interface->get_context()),
  clock_(clock_interface->get_clock()),
  logger_(logging_interface->get_logger().get_child(server_id))
{
  if (!server_id.empty()) {
    server_id_ = std::string(base_interface->get_fully_qualified_name()) + "/" + server_id;
  } else {
    server_id_ = base_interface->get_fully_qualified_name();
  }

  const std::string update_topic = topic_ns + "/update";
  const std::string feedback_topic = topic_ns + "/feedback";

  get_interactive_markers_service_ = rclcpp::create_service<
    visualization_msgs::srv::GetInteractiveMarkers>(
    base_interface,
    services_interface,
    topic_ns + "/get_interactive_markers",
    std::bind(
      &InteractiveMarkerServer::getInteractiveMarkersCallback,
      this,
      std::placeholders::_1,
      std::placeholders::_2,
      std::placeholders::_3),
    rmw_qos_profile_services_default,
    base_interface->get_default_callback_group());

  rclcpp::QoS update_qos(rclcpp::KeepLast(100));
  update_qos.durability_volatile();
  update_pub_ = rclcpp::create_publisher<InteractiveMarkerUpdate>(topics_interface, update_topic,
      update_qos);

  rclcpp::QoS feedback_qos(rclcpp::KeepLast(100));
  feedback_qos.durability_volatile();
  feedback_sub_ = rclcpp::create_subscription<InteractiveMarkerFeedback>(topics_interface,
      feedback_topic,
      feedback_qos,
      std::bind(&InteractiveMarkerServer::processFeedback, this, std::placeholders::_1));

  keep_alive_timer_ = rclcpp::GenericTimer<std::function<void()>>::make_shared(
    clock_,
    std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::milliseconds(500)),
    std::bind(&InteractiveMarkerServer::keepAlive, this),
    context_);

  rclcpp::callback_group::CallbackGroup::SharedPtr group{nullptr};
  timers_interface->add_timer(keep_alive_timer_, group);
}

InteractiveMarkerServer::~InteractiveMarkerServer()
{
  if (rclcpp::ok(context_)) {
    clear();
    applyChanges();
  }
}

void InteractiveMarkerServer::applyChanges()
{
  std::unique_lock<std::recursive_mutex> lock(mutex_);

  if (pending_updates_.empty()) {
    return;
  }

  visualization_msgs::msg::InteractiveMarkerUpdate update;
  update.type = visualization_msgs::msg::InteractiveMarkerUpdate::UPDATE;

  update.markers.reserve(marker_contexts_.size());
  update.poses.reserve(marker_contexts_.size());
  update.erases.reserve(marker_contexts_.size());

  for (auto update_it = pending_updates_.begin();
    update_it != pending_updates_.end();
    update_it++)
  {
    M_MarkerContext::iterator marker_context_it = marker_contexts_.find(update_it->first);

    switch (update_it->second.update_type) {
      case UpdateContext::FULL_UPDATE:
        {
          if (marker_context_it == marker_contexts_.end()) {
            RCLCPP_DEBUG(logger_, "Creating new context for %s", update_it->first.c_str());
            // create a new int_marker context
            marker_context_it =
              marker_contexts_.insert(std::make_pair(update_it->first, MarkerContext())).first;
            // copy feedback cbs, in case they have been set before the marker context was created
            marker_context_it->second.default_feedback_cb = update_it->second.default_feedback_cb;
            marker_context_it->second.feedback_cbs = update_it->second.feedback_cbs;
          }

          marker_context_it->second.int_marker = update_it->second.int_marker;

          update.markers.push_back(marker_context_it->second.int_marker);
          break;
        }

      case UpdateContext::POSE_UPDATE:
        {
          if (marker_context_it == marker_contexts_.end()) {
            RCLCPP_ERROR(
              logger_,
              "Pending pose update for non-existing marker found. This is a bug in "
              "InteractiveMarkerInterface.");
          } else {
            marker_context_it->second.int_marker.pose = update_it->second.int_marker.pose;
            marker_context_it->second.int_marker.header = update_it->second.int_marker.header;

            visualization_msgs::msg::InteractiveMarkerPose pose_update;
            pose_update.header = marker_context_it->second.int_marker.header;
            pose_update.pose = marker_context_it->second.int_marker.pose;
            pose_update.name = marker_context_it->second.int_marker.name;
            update.poses.push_back(pose_update);
          }
          break;
        }

      case UpdateContext::ERASE:
        {
          if (marker_context_it != marker_contexts_.end()) {
            marker_contexts_.erase(update_it->first);
            update.erases.push_back(update_it->first);
          }
          break;
        }
    }
  }

  seq_num_++;

  publish(update);
  pending_updates_.clear();
}

bool InteractiveMarkerServer::erase(const std::string & name)
{
  std::unique_lock<std::recursive_mutex> lock(mutex_);

  if (marker_contexts_.end() == marker_contexts_.find(name) &&
    pending_updates_.end() == pending_updates_.find(name))
  {
    return false;
  }
  pending_updates_[name].update_type = UpdateContext::ERASE;
  return true;
}

void InteractiveMarkerServer::clear()
{
  std::unique_lock<std::recursive_mutex> lock(mutex_);

  // erase all markers
  pending_updates_.clear();
  M_MarkerContext::iterator it;
  for (it = marker_contexts_.begin(); it != marker_contexts_.end(); it++) {
    pending_updates_[it->first].update_type = UpdateContext::ERASE;
  }
}

bool InteractiveMarkerServer::empty() const
{
  return marker_contexts_.empty();
}

std::size_t InteractiveMarkerServer::size() const
{
  return marker_contexts_.size();
}

bool InteractiveMarkerServer::setPose(
  const std::string & name,
  const geometry_msgs::msg::Pose & pose,
  const std_msgs::msg::Header & header)
{
  std::unique_lock<std::recursive_mutex> lock(mutex_);

  M_MarkerContext::iterator marker_context_it = marker_contexts_.find(name);
  M_UpdateContext::iterator update_it = pending_updates_.find(name);

  // if there's no marker and no pending addition for it, we can't update the pose
  if (marker_context_it == marker_contexts_.end() &&
    (update_it == pending_updates_.end() ||
    update_it->second.update_type != UpdateContext::FULL_UPDATE))
  {
    return false;
  }

  // keep the old header
  if (header.frame_id.empty()) {
    if (marker_context_it != marker_contexts_.end()) {
      doSetPose(update_it, name, pose, marker_context_it->second.int_marker.header);
    } else if (update_it != pending_updates_.end()) {
      doSetPose(update_it, name, pose, update_it->second.int_marker.header);
    } else {
      RCLCPP_WARN(logger_, "Marker does not exist and there is no pending creation.");
      return false;
    }
  } else {
    doSetPose(update_it, name, pose, header);
  }
  return true;
}

bool InteractiveMarkerServer::setCallback(
  const std::string & name,
  FeedbackCallback feedback_cb,
  uint8_t feedback_type)
{
  std::unique_lock<std::recursive_mutex> lock(mutex_);

  M_MarkerContext::iterator marker_context_it = marker_contexts_.find(name);
  M_UpdateContext::iterator update_it = pending_updates_.find(name);

  if (marker_context_it == marker_contexts_.end() && update_it == pending_updates_.end()) {
    return false;
  }

  // we need to overwrite both the callbacks for the actual marker
  // and the update, if there's any
  if (marker_context_it != marker_contexts_.end()) {
    // the marker exists, so we can just overwrite the existing callbacks
    if (feedback_type == DEFAULT_FEEDBACK_CB) {
      marker_context_it->second.default_feedback_cb = feedback_cb;
    } else {
      if (feedback_cb) {
        marker_context_it->second.feedback_cbs[feedback_type] = feedback_cb;
      } else {
        marker_context_it->second.feedback_cbs.erase(feedback_type);
      }
    }
  }

  if (update_it != pending_updates_.end()) {
    if (feedback_type == DEFAULT_FEEDBACK_CB) {
      update_it->second.default_feedback_cb = feedback_cb;
    } else {
      if (feedback_cb) {
        update_it->second.feedback_cbs[feedback_type] = feedback_cb;
      } else {
        update_it->second.feedback_cbs.erase(feedback_type);
      }
    }
  }
  return true;
}

void InteractiveMarkerServer::insert(const visualization_msgs::msg::InteractiveMarker & int_marker)
{
  std::unique_lock<std::recursive_mutex> lock(mutex_);

  M_UpdateContext::iterator update_it = pending_updates_.find(int_marker.name);
  if (update_it == pending_updates_.end()) {
    update_it = pending_updates_.insert(std::make_pair(int_marker.name, UpdateContext())).first;
  }

  update_it->second.update_type = UpdateContext::FULL_UPDATE;
  update_it->second.int_marker = int_marker;
}

void InteractiveMarkerServer::insert(
  const visualization_msgs::msg::InteractiveMarker & int_marker,
  FeedbackCallback feedback_cb,
  uint8_t feedback_type)
{
  insert(int_marker);

  setCallback(int_marker.name, feedback_cb, feedback_type);
}

bool InteractiveMarkerServer::get(
  std::string name,
  visualization_msgs::msg::InteractiveMarker & int_marker) const
{
  std::unique_lock<std::recursive_mutex> lock(mutex_);

  M_UpdateContext::const_iterator update_it = pending_updates_.find(name);

  if (update_it == pending_updates_.end()) {
    M_MarkerContext::const_iterator marker_context_it = marker_contexts_.find(name);
    if (marker_context_it == marker_contexts_.end()) {
      return false;
    }

    int_marker = marker_context_it->second.int_marker;
    return true;
  }

  // if there's an update pending, we'll have to account for that
  switch (update_it->second.update_type) {
    case UpdateContext::ERASE:
      return false;

    case UpdateContext::POSE_UPDATE:
      {
        M_MarkerContext::const_iterator marker_context_it = marker_contexts_.find(name);
        if (marker_context_it == marker_contexts_.end()) {
          return false;
        }
        int_marker = marker_context_it->second.int_marker;
        int_marker.pose = update_it->second.int_marker.pose;
        return true;
      }

    case UpdateContext::FULL_UPDATE:
      int_marker = update_it->second.int_marker;
      return true;
  }

  return false;
}

void InteractiveMarkerServer::getInteractiveMarkersCallback(
  const std::shared_ptr<rmw_request_id_t> request_header,
  const std::shared_ptr<visualization_msgs::srv::GetInteractiveMarkers::Request> request,
  std::shared_ptr<visualization_msgs::srv::GetInteractiveMarkers::Response> response)
{
  (void)request_header;

  RCLCPP_DEBUG(logger_, "Responding to request to get interactive markers");
  response->server_id = server_id_;
  response->sequence_number = seq_num_;
  response->markers.reserve(marker_contexts_.size());
  M_MarkerContext::iterator it;
  for (it = marker_contexts_.begin(); it != marker_contexts_.end(); it++) {
    RCLCPP_DEBUG(logger_, "Sending marker '%s'", it->second.int_marker.name.c_str());
    response->markers.push_back(it->second.int_marker);
  }
}

void InteractiveMarkerServer::processFeedback(
  visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr feedback)
{
  std::unique_lock<std::recursive_mutex> lock(mutex_);

  M_MarkerContext::iterator marker_context_it = marker_contexts_.find(feedback->marker_name);

  // ignore feedback for non-existing markers
  if (marker_context_it == marker_contexts_.end()) {
    return;
  }

  MarkerContext & marker_context = marker_context_it->second;

  // if two callers try to modify the same marker, reject (timeout= 1 sec)
  if (marker_context.last_client_id != feedback->client_id &&
    (clock_->now() - marker_context.last_feedback).seconds() < 1.0)
  {
    RCLCPP_DEBUG(logger_, "Rejecting feedback for %s: conflicting feedback from separate clients.",
      feedback->marker_name.c_str());
    return;
  }

  marker_context.last_feedback = clock_->now();
  marker_context.last_client_id = feedback->client_id;

  if (feedback->event_type == visualization_msgs::msg::InteractiveMarkerFeedback::POSE_UPDATE) {
    if (marker_context.int_marker.header.stamp == rclcpp::Time()) {
      // keep the old header
      doSetPose(pending_updates_.find(
          feedback->marker_name), feedback->marker_name, feedback->pose,
        marker_context.int_marker.header);
    } else {
      doSetPose(pending_updates_.find(
          feedback->marker_name), feedback->marker_name, feedback->pose, feedback->header);
    }
  }

  // call feedback handler
  std::unordered_map<uint8_t, FeedbackCallback>::iterator feedback_cb_it =
    marker_context.feedback_cbs.find(feedback->event_type);
  if (feedback_cb_it != marker_context.feedback_cbs.end() && feedback_cb_it->second) {
    // call type-specific callback
    feedback_cb_it->second(feedback);
  } else if (marker_context.default_feedback_cb) {
    // call default callback
    marker_context.default_feedback_cb(feedback);
  }
}

void InteractiveMarkerServer::keepAlive()
{
  visualization_msgs::msg::InteractiveMarkerUpdate empty_update;
  empty_update.type = visualization_msgs::msg::InteractiveMarkerUpdate::KEEP_ALIVE;
  publish(empty_update);
}

void InteractiveMarkerServer::publish(visualization_msgs::msg::InteractiveMarkerUpdate & update)
{
  update.server_id = server_id_;
  update.seq_num = seq_num_;
  update_pub_->publish(update);
}

void InteractiveMarkerServer::doSetPose(
  M_UpdateContext::iterator update_it,
  const std::string & name,
  const geometry_msgs::msg::Pose & pose,
  const std_msgs::msg::Header & header)
{
  if (update_it == pending_updates_.end()) {
    update_it = pending_updates_.insert(std::make_pair(name, UpdateContext())).first;
    update_it->second.update_type = UpdateContext::POSE_UPDATE;
  } else if (update_it->second.update_type != UpdateContext::FULL_UPDATE) {
    update_it->second.update_type = UpdateContext::POSE_UPDATE;
  }

  update_it->second.int_marker.pose = pose;
  update_it->second.int_marker.header = header;
  RCLCPP_DEBUG(logger_, "Marker '%s' is now at %f, %f, %f",
    update_it->first.c_str(), pose.position.x, pose.position.y, pose.position.z);
}

}  // namespace interactive_markers
