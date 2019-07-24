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

#include <functional>
#include <memory>
#include <string>

#include "interactive_markers/detail/single_client.hpp"

#define DBG_MSG(...) RCUTILS_LOG_DEBUG(__VA_ARGS__);

namespace interactive_markers
{

SingleClient::SingleClient(
  const std::string & server_id,
  tf2::BufferCore & tf,
  const std::string & target_frame,
  const InteractiveMarkerClient::CbCollection & callbacks
)
: state_(server_id, INIT),
  first_update_seq_num_(-1),
  last_update_seq_num_(-1),
  tf_(tf),
  target_frame_(target_frame),
  callbacks_(callbacks),
  server_id_(server_id),
  warn_keepalive_(false)
{
  callbacks_.statusCb(InteractiveMarkerClient::OK, server_id_, "Waiting for init message.");
}

SingleClient::~SingleClient()
{
  callbacks_.resetCb(server_id_);
}

void SingleClient::process(
  visualization_msgs::msg::InteractiveMarkerInit::SharedPtr msg,
  bool enable_autocomplete_transparency)
{
  DBG_MSG("%s: received init #%lu", server_id_.c_str(), msg->seq_num);

  switch (state_) {
    case INIT:
      if (init_queue_.size() > 5) {
        DBG_MSG("Init queue too large. Erasing init message with id %lu.",
          init_queue_.begin()->msg->seq_num);
        init_queue_.pop_back();
      }
      init_queue_.push_front(InitMessageContext(tf_, target_frame_, msg,
        enable_autocomplete_transparency));
      callbacks_.statusCb(InteractiveMarkerClient::OK, server_id_, "Init message received.");
      break;

    case RECEIVING:
    case TF_ERROR:
      break;
  }
}

void SingleClient::process(
  visualization_msgs::msg::InteractiveMarkerUpdate::SharedPtr msg,
  bool enable_autocomplete_transparency)
{
  if (first_update_seq_num_ == (uint64_t)-1) {
    first_update_seq_num_ = msg->seq_num;
  }

  last_update_time_ = rclcpp::Clock().now();

  if (msg->type == msg->KEEP_ALIVE) {
    DBG_MSG("%s: received keep-alive #%lu", server_id_.c_str(), msg->seq_num);
    if (last_update_seq_num_ != (uint64_t)-1 && msg->seq_num != last_update_seq_num_) {
      std::ostringstream s;
      s << "Sequence number of update is out of order. Expected: " << last_update_seq_num_ <<
        " Received: " << msg->seq_num;
      errorReset(s.str());
      return;
    }
    last_update_seq_num_ = msg->seq_num;
    return;
  } else {
    DBG_MSG("%s: received update #%lu", server_id_.c_str(), msg->seq_num);
    if (last_update_seq_num_ != (uint64_t)-1 && msg->seq_num != last_update_seq_num_ + 1) {
      std::ostringstream s;
      s << "Sequence number of update is out of order. Expected: " << last_update_seq_num_ + 1 <<
        " Received: " << msg->seq_num;
      errorReset(s.str());
      return;
    }
    last_update_seq_num_ = msg->seq_num;
  }

  switch (state_) {
    case INIT:
      if (update_queue_.size() > 100) {
        DBG_MSG("Update queue too large. Erasing update message with id %lu.",
          update_queue_.begin()->msg->seq_num);
        update_queue_.pop_back();
      }
      update_queue_.push_front(UpdateMessageContext(tf_, target_frame_, msg,
        enable_autocomplete_transparency));
      break;

    case RECEIVING:
      update_queue_.push_front(UpdateMessageContext(tf_, target_frame_, msg,
        enable_autocomplete_transparency));
      break;

    case TF_ERROR:
      break;
  }
}

void SingleClient::update()
{
  switch (state_) {
    case INIT:
      transformInitMsgs();
      transformUpdateMsgs();
      checkInitFinished();
      break;

    case RECEIVING:
      transformUpdateMsgs();
      pushUpdates();
      checkKeepAlive();
      if (update_queue_.size() > 100) {
        errorReset("Update queue overflow. Resetting connection.");
      }
      break;

    case TF_ERROR:
      if (state_.getDuration().seconds() > 1.0) {
        callbacks_.statusCb(InteractiveMarkerClient::ERROR, server_id_,
          "1 second has passed. Re-initializing.");
        state_ = INIT;
      }
      break;
  }
}

void SingleClient::checkKeepAlive()
{
  double time_since_upd = (rclcpp::Clock().now() - last_update_time_).seconds();
  if (time_since_upd > 2.0) {
    std::ostringstream s;
    s << "No update received for " << round(time_since_upd) << " seconds.";
    callbacks_.statusCb(InteractiveMarkerClient::WARN, server_id_, s.str());
    warn_keepalive_ = true;
  } else if (warn_keepalive_) {
    warn_keepalive_ = false;
    callbacks_.statusCb(InteractiveMarkerClient::OK, server_id_, "OK");
  }
}

void SingleClient::checkInitFinished()
{
  // check for all init messages received so far if tf info is ready
  // and the consecutive update exists.
  // If so, omit all updates with lower sequence number,
  // switch to RECEIVING mode and treat the init message like a regular update.

  if (last_update_seq_num_ == (uint64_t)-1) {
    callbacks_.statusCb(InteractiveMarkerClient::OK, server_id_,
      "Initialization: Waiting for first update/keep-alive message.");
    return;
  }

  M_InitMessageContext::iterator init_it;
  for (init_it = init_queue_.begin(); init_it != init_queue_.end(); ++init_it) {
    uint64_t init_seq_num = init_it->msg->seq_num;
    bool next_up_exists = init_seq_num >= first_update_seq_num_ &&
      init_seq_num <= last_update_seq_num_;

    if (!init_it->isReady()) {
      callbacks_.statusCb(InteractiveMarkerClient::OK, server_id_,
        "Initialization: Waiting for tf info.");
    } else if (next_up_exists) {
      DBG_MSG(
        "Init message with seq_id=%lu is ready & in line with updates. Switching to receive mode.",
        init_seq_num);
      while (!update_queue_.empty() && update_queue_.back().msg->seq_num <= init_seq_num) {
        DBG_MSG("Omitting update with seq_id=%lu", update_queue_.back().msg->seq_num);
        update_queue_.pop_back();
      }

      callbacks_.initCb(init_it->msg);
      callbacks_.statusCb(InteractiveMarkerClient::OK, server_id_, "Receiving updates.");

      init_queue_.clear();
      state_ = RECEIVING;

      pushUpdates();
      break;
    }
  }
}

void SingleClient::transformInitMsgs()
{
  M_InitMessageContext::iterator it;
  for (it = init_queue_.begin(); it != init_queue_.end(); ) {
    try {
      it->getTfTransforms();
    } catch (std::runtime_error & e) {
      // we want to notify the user, but also keep the init message
      // in case it is the only one we will receive.
      std::ostringstream s;
      s << "Cannot get tf info for init message with sequence number " << it->msg->seq_num <<
        ". Error: " << e.what();
      callbacks_.statusCb(InteractiveMarkerClient::WARN, server_id_, s.str());
    }
    ++it;
  }
}

void SingleClient::transformUpdateMsgs()
{
  M_UpdateMessageContext::iterator it;
  for (it = update_queue_.begin(); it != update_queue_.end(); ++it) {
    try {
      it->getTfTransforms();
    } catch (std::runtime_error & e) {
      std::ostringstream s;
      s << "Resetting due to tf error: " << e.what();
      errorReset(s.str());
      return;
    } catch (...) {
      std::ostringstream s;
      s << "Resetting due to unknown exception";
      errorReset(s.str());
    }
  }
}

void SingleClient::errorReset(std::string error_msg)
{
  // if we get an error here, we re-initialize everything
  state_ = TF_ERROR;
  update_queue_.clear();
  init_queue_.clear();
  first_update_seq_num_ = -1;
  last_update_seq_num_ = -1;
  warn_keepalive_ = false;

  callbacks_.statusCb(InteractiveMarkerClient::ERROR, server_id_, error_msg);
  callbacks_.resetCb(server_id_);
}

void SingleClient::pushUpdates()
{
  if (!update_queue_.empty() && update_queue_.back().isReady()) {
    callbacks_.statusCb(InteractiveMarkerClient::OK, server_id_, "OK");
  }
  while (!update_queue_.empty() && update_queue_.back().isReady()) {
    visualization_msgs::msg::InteractiveMarkerUpdate::SharedPtr msg = update_queue_.back().msg;
    DBG_MSG("Pushing out update #%lu.", msg->seq_num);
    callbacks_.updateCb(msg);
    update_queue_.pop_back();
  }
}

bool SingleClient::isInitialized()
{
  return state_ != INIT;
}

}  // namespace interactive_markers
