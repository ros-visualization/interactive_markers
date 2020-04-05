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

#include <boost/shared_ptr.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/function.hpp>
#include <boost/unordered_map.hpp>

#include <string>

#include <ros/subscriber.h>
#include <ros/node_handle.h>

#include <tf2_ros/buffer.h>

#include <visualization_msgs/InteractiveMarkerInit.h>
#include <visualization_msgs/InteractiveMarkerUpdate.h>
#include <interactive_markers/visibility_control.hpp>

#include "detail/state_machine.h"

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

  /// @param tf           The tf transformer to use.
  /// @param target_frame tf frame to transform timestamped messages into.
  /// @param topic_ns     The topic namespace (will subscribe to topic_ns/update, topic_ns/init)
  INTERACTIVE_MARKERS_PUBLIC
  InteractiveMarkerClient(tf2_ros::Buffer &tf,
      const std::string& target_frame = "",
      const std::string &topic_ns = "" );

  /// Will cause a 'reset' call for all server ids
  INTERACTIVE_MARKERS_PUBLIC
  ~InteractiveMarkerClient();

  /// Subscribe to the topics topic_ns/update and topic_ns/init
  INTERACTIVE_MARKERS_PUBLIC
  void subscribe( std::string topic_ns );

  /// Unsubscribe, clear queues & call reset callbacks
  INTERACTIVE_MARKERS_PUBLIC
  void shutdown();

  /// Update tf info, call callbacks
  INTERACTIVE_MARKERS_PUBLIC
  void update();

  /// Change the target frame and reset the connection
  INTERACTIVE_MARKERS_PUBLIC
  void setTargetFrame( std::string target_frame );

  /// Set callback for init messages
  INTERACTIVE_MARKERS_PUBLIC
  void setInitCb( const InitCallback& cb );

  /// Set callback for update messages
  INTERACTIVE_MARKERS_PUBLIC
  void setUpdateCb( const UpdateCallback& cb );

  /// Set callback for resetting one server connection
  INTERACTIVE_MARKERS_PUBLIC
  void setResetCb( const ResetCallback& cb );

  /// Set callback for status updates
  INTERACTIVE_MARKERS_PUBLIC
  void setStatusCb( const StatusCallback& cb );

  INTERACTIVE_MARKERS_PUBLIC
  void setEnableAutocompleteTransparency( bool enable ) { enable_autocomplete_transparency_ = enable;}

private:

  // Process message from the init or update channel
  template<class MsgConstPtrT>
  void process( const MsgConstPtrT& msg );

  ros::NodeHandle nh_;

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
  typedef boost::unordered_map<std::string, SingleClientPtr> M_SingleClient;
  M_SingleClient publisher_contexts_;
  boost::mutex publisher_contexts_mutex_;

  tf2_ros::Buffer& tf_;
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

    void setInitCb( InitCallback init_cb ) {
      init_cb_ = init_cb;
    }
    void setUpdateCb( UpdateCallback update_cb ) {
      update_cb_ = update_cb;
    }
    void setResetCb( ResetCallback reset_cb ) {
      reset_cb_ = reset_cb;
    }
    void setStatusCb( StatusCallback status_cb ) {
      status_cb_ = status_cb;
    }

  private:
    InitCallback init_cb_;
    UpdateCallback update_cb_;
    ResetCallback reset_cb_;
    StatusCallback status_cb_;
  };

  // handle init message
  void processInit( const InitConstPtr& msg );

  // handle update message
  void processUpdate( const UpdateConstPtr& msg );

private:
  CbCollection callbacks_;

  // this is the real (external) status callback
  StatusCallback status_cb_;

  // this allows us to detect if a server died (in most cases)
  uint32_t last_num_publishers_;

  // if false, auto-completed markers will have alpha = 1.0
  bool enable_autocomplete_transparency_;
};



}

#endif
