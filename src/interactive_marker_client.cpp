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

#include "interactive_markers/interactive_marker_client.h"
#include "interactive_markers/detail/single_client.h"

#include <boost/bind.hpp>
#include <boost/make_shared.hpp>

#define DBG_MSG( ... ) ROS_INFO_NAMED( "im_client", __VA_ARGS__ );

namespace interactive_markers
{

InteractiveMarkerClient::InteractiveMarkerClient(
    const tf::Transformer& tf,
    const std::string& target_frame,
    const std::string &topic_ns,
    bool spin_thread )
: state_(IDLE)
, tf_(tf)
, target_frame_(target_frame)
{
  subscribe( topic_ns );

  callbacks_.status_cb_ = boost::bind( &InteractiveMarkerClient::statusCb, this, _1, _2, _3 );
}

/// Subscribe to given topic
void InteractiveMarkerClient::subscribe( std::string topic_ns )
{
  topic_ns_ = topic_ns;
  subscribeUpdate();
  subscribeInit();
}

void InteractiveMarkerClient::setInitCb( const InitCallback& cb )
{
  callbacks_.init_cb_ = cb;
}

void InteractiveMarkerClient::setUpdateCb( const UpdateCallback& cb )
{
  callbacks_.update_cb_ = cb;
}

void InteractiveMarkerClient::setResetCb( const ResetCallback& cb )
{
  callbacks_.reset_cb_ = cb;
}

void InteractiveMarkerClient::setStatusCb( const StatusCallback& cb )
{
  status_cb_ = cb;
}

/// Clear all markers
void InteractiveMarkerClient::shutdown()
{
  publisher_contexts_.clear();
  init_sub_.shutdown();
  update_sub_.shutdown();
}

void InteractiveMarkerClient::subscribeUpdate()
{
  if ( state_ != INIT && !topic_ns_.empty() )
  {
    update_sub_ =nh_.subscribe( topic_ns_+"/update", 100, &InteractiveMarkerClient::process<UpdateConstPtr>, this );
    state_ = INIT;
  }
}

void InteractiveMarkerClient::subscribeInit()
{
  if ( state_ != INIT && !topic_ns_.empty() )
  {
    //boost::function< void(const InitConstPtr&) > cb = boost::bind( &InteractiveMarkerClient::process<InitConstPtr>, this, _1 );
    init_sub_ =nh_.subscribe( topic_ns_+"/init", 100, &InteractiveMarkerClient::process<InitConstPtr>, this );
    state_ = INIT;
  }
}

template<class MsgConstPtrT>
void InteractiveMarkerClient::process( const MsgConstPtrT& msg )
{
  // get caller ID of the sending entity
  if ( msg->server_id.empty() )
  {
    //setStatusError( "Topic", "server_id is empty!");
    return;
  }

  M_SingleClient::iterator context_it = publisher_contexts_.find(msg->server_id);

  // If we haven't seen this publisher before, we need to reset the
  // display and listen to the init topic, plus of course add this
  // publisher to our list.
  if ( context_it == publisher_contexts_.end() )
  {
    DBG_MSG( "New publisher detected: %s", msg->server_id.c_str() );

    SingleClientPtr pc(new SingleClient( msg->server_id, tf_, target_frame_, callbacks_ ));
    context_it = publisher_contexts_.insert( std::make_pair(msg->server_id,pc) ).first;

    // we need to subscribe to the init topic again
    subscribeInit();
  }

  // forward init/update to respective context
  context_it->second->process( msg );
}

void InteractiveMarkerClient::spin()
{
  switch ( (StateT)state_ )
  {
  case IDLE:
    break;

  case INIT:
  case RUNNING:
  {
    // check if all single clients are finished with the init channels
    bool initialized = true;
    M_SingleClient::iterator it;
    for ( it = publisher_contexts_.begin(); it!=publisher_contexts_.end(); ++it )
    {
      it->second->spin();
      if ( !it->second->isInitialized() )
      {
        initialized = false;
      }
    }
    if ( initialized )
    {
      init_sub_.shutdown();
      state_ = RUNNING;
    }
    break;
  }
  }
}

void InteractiveMarkerClient::statusCb( StatusT status, const std::string& server_id, const std::string& msg )
{
  switch ( status )
  {
  case OK:
    DBG_MSG( "%s: %s (Status: OK)", server_id.c_str(), msg.c_str() );
    break;
  case WARN:
    DBG_MSG( "%s: %s (Status: WARNING)", server_id.c_str(), msg.c_str() );
    break;
  case ERROR:
    DBG_MSG( "%s: %s (Status: ERROR)", server_id.c_str(), msg.c_str() );
    break;
  }

  if ( status_cb_ )
  {
    status_cb_( status, server_id, msg );
  }
}

}
