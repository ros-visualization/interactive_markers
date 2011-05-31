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

#include "interactive_markers/interactive_marker_server.h"

#include <boost/bind.hpp>
#include <boost/make_shared.hpp>

namespace interactive_markers
{

InteractiveMarkerServer::InteractiveMarkerServer( std::string topic_ns, std::string server_id, bool spin_thread ) :
    topic_ns_(topic_ns),
    seq_num_(0)
{
  if ( spin_thread )
  {
    // if we're spinning our own thread, we'll also need our own callback queue
    node_handle_.setCallbackQueue( &callback_queue_ );
  }

  if (!server_id.empty())
  {
    server_id_ = ros::this_node::getName() + "/" + server_id;
  }
  else
  {
    server_id_ = ros::this_node::getName();
  }

  std::string marker_topic = topic_ns + "/update";
  std::string feedback_topic = topic_ns + "/feedback";

  update_pub_ = node_handle_.advertise<visualization_msgs::InteractiveMarkerUpdate>(
      marker_topic, 100, boost::bind( &InteractiveMarkerServer::processConnect, this, _1 ) );

  feedback_sub_ = node_handle_.subscribe( feedback_topic, 100, &InteractiveMarkerServer::processFeedback, this );

  keep_alive_timer_ =  node_handle_.createTimer(ros::Duration(0.5f), boost::bind( &InteractiveMarkerServer::keepAlive, this ) );

  if ( spin_thread )
  {
    need_to_terminate_ = false;
    spin_thread_.reset( new boost::thread(boost::bind(&InteractiveMarkerServer::spinThread, this)) );
  }
}


InteractiveMarkerServer::~InteractiveMarkerServer()
{
  if (spin_thread_.get())
  {
    need_to_terminate_ = true;
    spin_thread_->join();
  }
}


void InteractiveMarkerServer::spinThread()
{
  while (node_handle_.ok())
  {
    if (need_to_terminate_)
    {
      break;
    }
    callback_queue_.callAvailable(ros::WallDuration(0.033f));
  }
}


void InteractiveMarkerServer::publishUpdate()
{
  boost::recursive_mutex::scoped_lock lock( mutex_ );

  if ( pending_updates_.empty() )
  {
    return;
  }

  M_UpdateContext::iterator update_it;

  visualization_msgs::InteractiveMarkerUpdate update;
  update.type = visualization_msgs::InteractiveMarkerUpdate::UPDATE;

  update.markers.reserve( marker_contexts_.size() );
  update.poses.reserve( marker_contexts_.size() );
  update.erases.reserve( marker_contexts_.size() );

  for ( update_it = pending_updates_.begin(); update_it != pending_updates_.end(); update_it++ )
  {
    M_MarkerContext::iterator marker_context_it = marker_contexts_.find( update_it->first );

    switch ( update_it->second.update_type )
    {
      case UpdateContext::FULL_UPDATE:
      {
        if ( marker_context_it == marker_contexts_.end() )
        {
          ROS_DEBUG("Creating new context for %s", update_it->first.c_str());
          // create a new int_marker context
          marker_context_it = marker_contexts_.insert( std::make_pair( update_it->first, MarkerContext() ) ).first;
        }

        marker_context_it->second.int_marker = update_it->second.int_marker;
        marker_context_it->second.feedback_cb = update_it->second.feedback_cb;

        update.markers.push_back( marker_context_it->second.int_marker );
        break;
      }

      case UpdateContext::POSE_UPDATE:
      {
        if ( marker_context_it == marker_contexts_.end() )
        {
          ROS_ERROR( "Pending pose update for non-existing marker found. This is a bug in InteractiveMarkerInterface." );
        }
        else
        {
          marker_context_it->second.int_marker.pose = update_it->second.int_marker.pose;

          visualization_msgs::InteractiveMarkerPose pose_update;
          pose_update.header = marker_context_it->second.int_marker.header;
          pose_update.pose = marker_context_it->second.int_marker.pose;
          pose_update.name = marker_context_it->second.int_marker.name;
          update.poses.push_back( pose_update );
        }
        break;
      }

      case UpdateContext::ERASE:
      {
        if ( marker_context_it == marker_contexts_.end() )
        {
          ROS_ERROR( "Pending erase for non-existing marker found. This is a bug in InteractiveMarkerInterface." );
        }
        else
        {
          marker_contexts_.erase( update_it->first );
          update.erases.push_back( update_it->first );
        }
        break;
      }
    }
  }

  publish( update );
  pending_updates_.clear();
}


bool InteractiveMarkerServer::erase( const std::string &name )
{
  boost::recursive_mutex::scoped_lock lock( mutex_ );

  M_MarkerContext::iterator marker_context_it = marker_contexts_.find( name );
  if ( marker_context_it == marker_contexts_.end() )
  {
    return false;
  }

  pending_updates_[name].update_type = UpdateContext::ERASE;
  return true;
}


bool InteractiveMarkerServer::setPose( const std::string &name, const geometry_msgs::Pose &pose, const std_msgs::Header &header )
{
  boost::recursive_mutex::scoped_lock lock( mutex_ );

  M_MarkerContext::iterator marker_context_it = marker_contexts_.find( name );
  if ( marker_context_it == marker_contexts_.end() )
  {
    return false;
  }

  if ( header.frame_id.empty() )
  {
    // keep the old header
    doSetPose( marker_context_it, pose, marker_context_it->second.int_marker.header );
  }
  else
  {
    doSetPose( marker_context_it, pose, header );
  }
  return true;
}


void InteractiveMarkerServer::insert( const visualization_msgs::InteractiveMarker &int_marker, FeedbackCallback feedback_cb )
{
  boost::recursive_mutex::scoped_lock lock( mutex_ );

  const std::string &name = int_marker.name;
  // If there is an entry in pending_updates_, it will be POSE_UPDATE
  // or FULL_UPDATE. In both cases we'll want to keep it that way.
  M_UpdateContext::iterator update_it = pending_updates_.find( name );
  if ( update_it == pending_updates_.end() )
  {
    update_it = pending_updates_.insert( std::make_pair( name, UpdateContext() ) ).first;
  }

  update_it->second.update_type = UpdateContext::FULL_UPDATE;
  update_it->second.int_marker = int_marker;
  update_it->second.feedback_cb = feedback_cb;
}


void InteractiveMarkerServer::processConnect( const ros::SingleSubscriberPublisher& pub )
{
  boost::recursive_mutex::scoped_lock lock( mutex_ );

  ROS_INFO( "Re-sending all markers to new client '%s'.", pub.getSubscriberName().c_str() );

  // send full set of markers to the single new subscriber
  // don't increase sequence number
  visualization_msgs::InteractiveMarkerUpdate update;
  update.server_id = server_id_;
  update.type = visualization_msgs::InteractiveMarkerUpdate::INIT;
  update.seq_num = seq_num_;
  update.markers.reserve( marker_contexts_.size() );

  M_MarkerContext::iterator it;
  for ( it = marker_contexts_.begin(); it != marker_contexts_.end(); it++ )
  {
    ROS_DEBUG( "Publishing %s", it->second.int_marker.name.c_str() );
    update.markers.push_back( it->second.int_marker );
  }

  pub.publish( update );
}



void InteractiveMarkerServer::processFeedback( const FeedbackConstPtr& feedback )
{
  boost::recursive_mutex::scoped_lock lock( mutex_ );

  M_MarkerContext::iterator marker_context_it = marker_contexts_.find( feedback->marker_name );

  // ignore feedback for non-existing markers
  if ( marker_context_it == marker_contexts_.end() )
  {
    return;
  }

  MarkerContext &marker_context = marker_context_it->second;

  // if two callers try to modify the same marker, reject (timeout= 1 sec)
  if ( marker_context.last_client_id != feedback->client_id &&
      (ros::Time::now() - marker_context.last_feedback).toSec() < 1.0 )
  {
    ROS_DEBUG( "Rejecting feedback for %s: conflicting feedback from separate clients.", feedback->marker_name.c_str() );
    return;
  }

  marker_context.last_feedback = ros::Time::now();
  marker_context.last_client_id = feedback->client_id;

  if ( marker_context.int_marker.header.stamp == ros::Time(0) )
  {
    // keep the old header
    doSetPose( marker_context_it, feedback->pose, marker_context.int_marker.header );
  }
  else
  {
    doSetPose( marker_context_it, feedback->pose, feedback->header );
  }

  if ( feedback->event_type == visualization_msgs::InteractiveMarkerFeedback::KEEP_ALIVE )
  {
    ROS_DEBUG( "Received keep-alive from %s", feedback->marker_name.c_str() );
    // we don't want the user to receive keep-alive messages
    return;
  }

  // call user feedback handler
  if ( marker_context.feedback_cb )
  {
    marker_context.feedback_cb(  feedback );
  }
}


void InteractiveMarkerServer::keepAlive()
{
  visualization_msgs::InteractiveMarkerUpdate empty_update;
  empty_update.type = visualization_msgs::InteractiveMarkerUpdate::KEEP_ALIVE;
  publish( empty_update );
}


void InteractiveMarkerServer::publish( visualization_msgs::InteractiveMarkerUpdate &update )
{
  if ( update.type == visualization_msgs::InteractiveMarkerUpdate::UPDATE )
  {
    //only increase the sequence number for actual updates
    seq_num_++;
  }
  update.server_id = server_id_;
  update.seq_num = seq_num_;
  update_pub_.publish( update );
}


void InteractiveMarkerServer::doSetPose( M_MarkerContext::iterator marker_context_it, const geometry_msgs::Pose &pose, const std_msgs::Header &header )
{
  std::string &name = marker_context_it->second.int_marker.name;

  M_UpdateContext::iterator update_it = pending_updates_.find( name );
  if ( update_it == pending_updates_.end() )
  {
    update_it = pending_updates_.insert( std::make_pair( name, UpdateContext() ) ).first;
    pending_updates_[name].update_type = UpdateContext::POSE_UPDATE;
  }
  else if ( update_it->second.update_type != UpdateContext::FULL_UPDATE )
  {
    update_it->second.update_type = UpdateContext::POSE_UPDATE;
  }

  update_it->second.int_marker.pose = pose;
  update_it->second.int_marker.header = header;
  ROS_DEBUG( "Marker '%s' is now at %f, %f, %f", name.c_str(), pose.position.x, pose.position.y, pose.position.z );
}


}
