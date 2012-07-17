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

#ifndef INTERACTIVE_MARKER_CLIENT
#define INTERACTIVE_MARKER_CLIENT

#include <visualization_msgs/InteractiveMarkerInit.h>
#include <visualization_msgs/InteractiveMarkerUpdate.h>

#include <boost/scoped_ptr.hpp>
#include <boost/thread/thread.hpp>
#include <boost/thread/recursive_mutex.hpp>

#include <ros/ros.h>
#include <ros/callback_queue.h>

#include <boost/function.hpp>
#include <boost/unordered_map.hpp>

#include <tf/tf.h>

#include "detail/state_machine.h"

namespace interactive_markers
{

class SingleClient;

class InteractiveMarkerClient : boost::noncopyable
{
public:

  enum StatusT {
    OK = 0,
    WARN = 1,
    ERROR = 2
  };

  typedef visualization_msgs::InteractiveMarkerUpdateConstPtr UpdateConstPtr;
  typedef visualization_msgs::InteractiveMarkerInitConstPtr InitConstPtr;

  typedef boost::function< void ( const UpdateConstPtr& ) > UpdateCallback;
  typedef boost::function< void ( const InitConstPtr& ) > InitCallback;
  typedef boost::function< void ( const std::string& ) > ResetCallback;
  typedef boost::function< void ( StatusT, const std::string&, const std::string& ) > StatusCallback;

  static const uint8_t DEFAULT_FEEDBACK_CB = 255;

  InteractiveMarkerClient( tf::Transformer& tf,
      const std::string& target_frame = "",
      const std::string &topic_ns = "",
      bool spin_thread = false );

  ~InteractiveMarkerClient();

  /// Subscribe to given topic
  void subscribe( std::string topic_ns );

  /// Unsubscribe, clear queues & call reset callback
  void shutdown();

  /// Update tf info, call callbacks
  void spin();

  void setTargetFrame( std::string target_frame );

  void setInitCb( const InitCallback& cb );

  void setUpdateCb( const UpdateCallback& cb );

  void setResetCb( const ResetCallback& cb );

  void setStatusCb( const StatusCallback& cb );

private:

  // Process message from the init or update channel
  template<class MsgConstPtrT>
  void process( const MsgConstPtrT& msg );

  enum StateT
  {
    IDLE,
    INIT,
    RUNNING
  };

  StateMachine<StateT> state_;

  std::string topic_ns_;

  ros::Subscriber update_sub_;
  ros::Subscriber init_sub_;

  // subscribe to the init channel
  void subscribeInit();

  // subscribe to the init channel
  void subscribeUpdate();

  void statusCb( StatusT status, const std::string& server_id, const std::string& msg );

  typedef boost::shared_ptr<SingleClient> SingleClientPtr;
  typedef std::map<std::string, SingleClientPtr> M_SingleClient;
  M_SingleClient publisher_contexts_;

  ros::NodeHandle nh_;

  tf::Transformer& tf_;
  std::string target_frame_;

public:
  // for internal usage
  struct CbCollection
  {
    void initCb( const InitConstPtr& i ) const {
      if (init_cb_) init_cb_( i ); }
    void updateCb( const UpdateConstPtr& u ) const {
      if (update_cb_) update_cb_( u ); }
    void resetCb( const std::string& s ) const {
      if (reset_cb_) reset_cb_(s); }
    void statusCb( StatusT s, const std::string& id, const std::string& m ) const {
      if (status_cb_) status_cb_(s,id,m); }

    InitCallback init_cb_;
    UpdateCallback update_cb_;
    ResetCallback reset_cb_;
    StatusCallback status_cb_;
  };

private:
  CbCollection callbacks_;

  // this is the real (external) status signal
  StatusCallback status_cb_;
};



}

#endif
