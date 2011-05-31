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

#ifndef INTERACTIVE_MARKER_INTERFACE
#define INTERACTIVE_MARKER_INTERFACE

#include <visualization_msgs/InteractiveMarkerUpdate.h>
#include <visualization_msgs/InteractiveMarkerFeedback.h>

#include <boost/scoped_ptr.hpp>
#include <boost/thread/thread.hpp>
#include <boost/thread/recursive_mutex.hpp>

#include <ros/ros.h>
#include <ros/callback_queue.h>


#include <boost/function.hpp>
#include <boost/unordered_map.hpp>

namespace interactive_markers
{

// Acts as a server to one or many GUIs (e.g. rviz)
// displaying a set of interactive markers
class InteractiveMarkerServer : boost::noncopyable
{
public:

  // type for feedback functions, takes the interactive marker name
  // as first argument and the feedback as second
  typedef visualization_msgs::InteractiveMarkerFeedbackConstPtr FeedbackConstPtr;
  typedef boost::function< void ( const FeedbackConstPtr& ) > FeedbackCallback;

  // @param topic_ns:     The interface will use the topics topic_ns/update and
  //                      topic_ns/feedback for communication.
  // @param server_id:    If you run multiple servers on the same topic from
  //                      within the same node, you will need to assign different names to them.
  //                      Otherwise, leave this empty.
  // @param spin_thread:  If set to true, will spin up a thread for message handling.
  //                      All callbacks will be called from that thread.
  InteractiveMarkerServer( std::string topic_ns, std::string server_id="", bool spin_thread = false );

  // Destruction of the interface will lead to all markers being cleared.
  ~InteractiveMarkerServer();

  // Add or replace a marker.
  // @param int_marker The marker to be added or replaced
  // @param feedback_cb function to call on the arrival of a feedback message
  void insert( const visualization_msgs::InteractiveMarker &int_marker,
      FeedbackCallback feedback_cb=FeedbackCallback() );

  // Update the pose of a marker with the specified name
  // @return true if a marker with that name exists
  // @param name:   identifies the marker to be updated
  // @param pose:   the new pose
  // @param header: header replacement. Leave this empty to use the previous one.
  bool setPose( const std::string &name,
      const geometry_msgs::Pose &pose,
      const std_msgs::Header &header=std_msgs::Header() );

  // Erase the marker with the specified name
  // @return true if a marker with that name exists
  // @param name:   identifies the marker to be erased
  bool erase( const std::string &name );

  // apply pending updates, broadcast to all clients & reset update list
  void publishUpdate();

private:

  struct MarkerContext
  {
    ros::Time last_feedback;
    std::string last_client_id;
    FeedbackCallback feedback_cb;
    visualization_msgs::InteractiveMarker int_marker;
  };

  typedef boost::unordered_map< std::string, MarkerContext > M_MarkerContext;

  // represents an update to a single marker
  struct UpdateContext
  {
    enum {
      FULL_UPDATE,
      POSE_UPDATE,
      ERASE
    } update_type;
    FeedbackCallback feedback_cb;
    visualization_msgs::InteractiveMarker int_marker;
  };

  typedef boost::unordered_map< std::string, UpdateContext > M_UpdateContext;

  // main loop when spinning our own thread
  // - process callbacks in our callback queue
  // - process pending goals
  void spinThread();

  // if someone connects, we need to re-send all the markers
  void processConnect( const ros::SingleSubscriberPublisher& );

  // update marker pose & call user callback
  void processFeedback( const FeedbackConstPtr& feedback );

  // send an empty update to keep the client GUIs happy
  void keepAlive();

  // increase sequence number & publish an update
  void publish( visualization_msgs::InteractiveMarkerUpdate &update );

  // Update pose, schedule update without locking
  void doSetPose( M_MarkerContext::iterator marker_context_it,
      const geometry_msgs::Pose &pose,
      const std_msgs::Header &header );

  // all managed goals_
  M_MarkerContext marker_contexts_;

  // updates that have to be sent on the next publish
  M_UpdateContext pending_updates_;

  // topic namespace to use or the actions
  std::string topic_ns_;
  
  boost::recursive_mutex mutex_;

  // these are needed when spinning up a dedicated thread
  boost::scoped_ptr<boost::thread> spin_thread_;
  ros::NodeHandle node_handle_;
  ros::CallbackQueue callback_queue_;
  volatile bool need_to_terminate_;

  // this is needed when running in non-threaded mode
  ros::Timer keep_alive_timer_;

  ros::Publisher update_pub_;
  ros::Subscriber feedback_sub_;

  uint64_t seq_num_;

  std::string server_id_;
};

}

#endif
