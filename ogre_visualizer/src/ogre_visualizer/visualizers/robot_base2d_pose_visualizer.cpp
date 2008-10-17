/*
 * Copyright (c) 2008, Willow Garage, Inc.
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
 */

#include "robot_base2d_pose_visualizer.h"
#include "properties/property.h"
#include "properties/property_manager.h"
#include "common.h"

#include "ogre_tools/arrow.h"

#include <Ogre.h>
#include <wx/wx.h>
#include <ros/node.h>
#include <rosTF/rosTF.h>

#include <boost/bind.hpp>

namespace ogre_vis
{

RobotBase2DPoseVisualizer::RobotBase2DPoseVisualizer( const std::string& name, VisualizationManager* manager )
: VisualizerBase( name, manager )
, color_( 1.0f, 1.0f, 1.0f )
, position_tolerance_( 0.01 )
, angle_tolerance_( 0.1 )
, color_property_( NULL )
, topic_property_( NULL )
, position_tolerance_property_( NULL )
, angle_tolerance_property_( NULL )
{
  scene_node_ = scene_manager_->getRootSceneNode()->createChildSceneNode();
}

RobotBase2DPoseVisualizer::~RobotBase2DPoseVisualizer()
{
  unsubscribe();
  clear();
}

void RobotBase2DPoseVisualizer::clear()
{
  V_Arrow::iterator it = arrows_.begin();
  V_Arrow::iterator end = arrows_.end();
  for ( ; it != end; ++it )
  {
    delete *it;
  }
  arrows_.clear();
  messages_.clear();
}

void RobotBase2DPoseVisualizer::setTopic( const std::string& topic )
{
  unsubscribe();

  topic_ = topic;

  subscribe();

  if ( topic_property_ )
  {
    topic_property_->changed();
  }

  causeRender();
}

void RobotBase2DPoseVisualizer::setColor( const Color& color )
{
  color_ = color;

  V_Arrow::iterator it = arrows_.begin();
  V_Arrow::iterator end = arrows_.end();
  for ( ; it != end; ++it )
  {
    ogre_tools::Arrow* arrow = *it;
    arrow->setColor( color.r_, color.g_, color.b_, 1.0f );
  }

  if ( color_property_ )
  {
    color_property_->changed();
  }

  causeRender();
}

void RobotBase2DPoseVisualizer::setPositionTolerance( float tol )
{
  position_tolerance_ = tol;

  if ( position_tolerance_property_ )
  {
    position_tolerance_property_->changed();
  }
}

void RobotBase2DPoseVisualizer::setAngleTolerance( float tol )
{
  angle_tolerance_ = tol;

  if ( angle_tolerance_property_ )
  {
    angle_tolerance_property_->changed();
  }
}

void RobotBase2DPoseVisualizer::subscribe()
{
  if ( !isEnabled() )
  {
    return;
  }

  if ( !topic_.empty() )
  {
    ros_node_->subscribe( topic_, message_, &RobotBase2DPoseVisualizer::incomingMessage, this, 1 );
  }
}

void RobotBase2DPoseVisualizer::unsubscribe()
{
  if ( !topic_.empty() )
  {
    ros_node_->unsubscribe( topic_, &RobotBase2DPoseVisualizer::incomingMessage, this );
  }
}

void RobotBase2DPoseVisualizer::onEnable()
{
  scene_node_->setVisible( true );
  subscribe();
}

void RobotBase2DPoseVisualizer::onDisable()
{
  unsubscribe();
  clear();
  scene_node_->setVisible( false );
}

void RobotBase2DPoseVisualizer::createProperties()
{
  color_property_ = property_manager_->createProperty<ColorProperty>( "Color", property_prefix_, boost::bind( &RobotBase2DPoseVisualizer::getColor, this ),
                                                                          boost::bind( &RobotBase2DPoseVisualizer::setColor, this, _1 ), parent_category_, this );
  topic_property_ = property_manager_->createProperty<ROSTopicStringProperty>( "Topic", property_prefix_, boost::bind( &RobotBase2DPoseVisualizer::getTopic, this ),
                                                                                boost::bind( &RobotBase2DPoseVisualizer::setTopic, this, _1 ), parent_category_, this );

  position_tolerance_property_ = property_manager_->createProperty<FloatProperty>( "Position Tolerance", property_prefix_, boost::bind( &RobotBase2DPoseVisualizer::getPositionTolerance, this ),
                                                                               boost::bind( &RobotBase2DPoseVisualizer::setPositionTolerance, this, _1 ), parent_category_, this );
  angle_tolerance_property_ = property_manager_->createProperty<FloatProperty>( "Angle Tolerance", property_prefix_, boost::bind( &RobotBase2DPoseVisualizer::getAngleTolerance, this ),
                                                                                 boost::bind( &RobotBase2DPoseVisualizer::setAngleTolerance, this, _1 ), parent_category_, this );
}

void RobotBase2DPoseVisualizer::processMessage( const std_msgs::RobotBase2DOdom& message )
{
  if ( !messages_.empty() )
  {
    const std_msgs::RobotBase2DOdom& last_message = messages_.back();
    if ( abs(last_message.pos.x - message.pos.x) < position_tolerance_
      && abs(last_message.pos.y - message.pos.y) < position_tolerance_
      && abs(last_message.pos.th - message.pos.th) < angle_tolerance_ )
    {
      return;
    }
  }

  ogre_tools::Arrow* arrow = new ogre_tools::Arrow( scene_manager_, scene_node_, 0.8f, 0.05f, 0.2f, 0.1f );

  transformArrow( message, arrow );

  arrow->setColor( color_.r_, color_.g_, color_.b_, 1.0f );
  arrow->setUserData( Ogre::Any((void*)this) );

  arrows_.push_back( arrow );
  messages_.push_back( message );
}

void RobotBase2DPoseVisualizer::transformArrow( const std_msgs::RobotBase2DOdom& message, ogre_tools::Arrow* arrow )
{
  std::string frame_id = message.header.frame_id;
  if ( frame_id.empty() )
  {
    frame_id = target_frame_;
  }

  libTF::TFPose pose = { message.pos.x, message.pos.y, 0.0f, message.pos.th, 0.0f, 0.0f, 0, message.header.frame_id };

  try
  {
    if ( frame_id != target_frame_ )
    {
      pose.yaw = -pose.yaw;
      pose = tf_client_->transformPose( target_frame_, pose );
    }
  }
  catch(libTF::Exception& e)
  {
    ROS_ERROR( "Error 2d base pose '%s' from frame '%s' to frame '%s'\n", name_.c_str(), message.header.frame_id.c_str(), target_frame_.c_str() );
  }

  Ogre::Matrix3 orient;
  orient.FromEulerAnglesZXY( Ogre::Radian( pose.roll ), Ogre::Radian( pose.pitch ), Ogre::Radian( pose.yaw ) );
  arrow->setOrientation( orient );

  Ogre::Vector3 pos( pose.x, pose.y, pose.z );
  robotToOgre( pos );
  arrow->setPosition( pos );
}

void RobotBase2DPoseVisualizer::targetFrameChanged()
{
  ROS_ASSERT( messages_.size() == arrows_.size() );
  V_RobotBase2DOdom::iterator msg_it = messages_.begin();
  V_Arrow::iterator arrow_it = arrows_.begin();
  V_RobotBase2DOdom::iterator msg_end = messages_.end();
  for ( ; msg_it != msg_end; ++msg_it, ++arrow_it )
  {
    transformArrow( *msg_it, *arrow_it );
  }
}

void RobotBase2DPoseVisualizer::update( float dt )
{
  V_RobotBase2DOdom local_queue;
  queue_mutex_.lock();

  local_queue.swap( message_queue_ );

  queue_mutex_.unlock();

  if ( !local_queue.empty() )
  {
    V_RobotBase2DOdom::iterator it = local_queue.begin();
    V_RobotBase2DOdom::iterator end = local_queue.end();
    for ( ; it != end; ++it )
    {
      processMessage( *it );
    }

    causeRender();
  }
}

void RobotBase2DPoseVisualizer::incomingMessage()
{
  queue_mutex_.lock();

  message_queue_.push_back( message_ );

  queue_mutex_.unlock();
}

} // namespace ogre_vis
