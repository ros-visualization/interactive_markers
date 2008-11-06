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

#include "planning_visualizer.h"
#include "common.h"
#include "helpers/robot.h"
#include "properties/property.h"
#include "properties/property_manager.h"

#include <ogre_tools/axes.h>

#include <urdf/URDF.h>
#include <tf/transform_listener.h>
#include <planning_models/kinematic.h>

#include <OgreSceneNode.h>
#include <OgreSceneManager.h>

namespace ogre_vis
{

PlanningVisualizer::PlanningVisualizer( const std::string& name, VisualizationManager* manager )
: VisualizerBase( name, manager )
, kinematic_model_( NULL )
, new_kinematic_path_( false )
, animating_path_( false )
, state_display_time_( 0.05f )
, visual_enabled_property_( NULL )
, collision_enabled_property_( NULL )
, state_display_time_property_( NULL )
, robot_description_property_( NULL )
, topic_property_( NULL )
{
  robot_ = new Robot( scene_manager_ );

  setVisualVisible( false );
  setCollisionVisible( true );
  robot_->setUserData( Ogre::Any( (void*)this ) );
}

PlanningVisualizer::~PlanningVisualizer()
{
  unsubscribe();

  delete robot_;
}

void PlanningVisualizer::initialize( const std::string& description_param, const std::string& kinematic_path_topic )
{
  setRobotDescription( description_param );
  setTopic( kinematic_path_topic );
}

void PlanningVisualizer::setRobotDescription( const std::string& description_param )
{
  description_param_ = description_param;

  if ( robot_description_property_ )
  {
    robot_description_property_->changed();
  }

  if ( isEnabled() )
  {
    load();
    causeRender();
  }
}

void PlanningVisualizer::setTopic( const std::string& topic )
{
  unsubscribe();
  kinematic_path_topic_ = topic;
  subscribe();

  if ( topic_property_ )
  {
    topic_property_->changed();
  }
}

void PlanningVisualizer::setStateDisplayTime( float time )
{
  state_display_time_ = time;

  if ( state_display_time_property_ )
  {
    state_display_time_property_->changed();
  }

  causeRender();
}

void PlanningVisualizer::setVisualVisible( bool visible )
{
  robot_->setVisualVisible( visible );

  if ( visual_enabled_property_ )
  {
    visual_enabled_property_->changed();
  }

  causeRender();
}

void PlanningVisualizer::setCollisionVisible( bool visible )
{
  robot_->setCollisionVisible( visible );

  if ( collision_enabled_property_ )
  {
    collision_enabled_property_->changed();
  }

  causeRender();
}

bool PlanningVisualizer::isVisualVisible()
{
  return robot_->isVisualVisible();
}

bool PlanningVisualizer::isCollisionVisible()
{
  return robot_->isCollisionVisible();
}

void PlanningVisualizer::load()
{
  std::string content;
  ros_node_->get_param(description_param_, content);
  robot_desc::URDF file;
  file.loadString(content.c_str());

  robot_->load( &file );

  delete kinematic_model_;
  kinematic_model_ = new planning_models::KinematicModel();
  kinematic_model_->setVerbose( false );
  kinematic_model_->build( file );
  kinematic_model_->defaultState();

  robot_->update( kinematic_model_, target_frame_ );
}

void PlanningVisualizer::onEnable()
{
  subscribe();

  load();
  robot_->setVisible( true );
}

void PlanningVisualizer::onDisable()
{
  unsubscribe();
  robot_->setVisible( false );
}

void PlanningVisualizer::subscribe()
{
  if ( !isEnabled() )
  {
    return;
  }

  if ( !kinematic_path_topic_.empty() )
  {
    ros_node_->subscribe( kinematic_path_topic_, incoming_kinematic_path_message_, &PlanningVisualizer::incomingKinematicPath, this, 0 );
  }

}

void PlanningVisualizer::unsubscribe()
{
  if ( !kinematic_path_topic_.empty() )
  {
    ros_node_->unsubscribe( kinematic_path_topic_, &PlanningVisualizer::incomingKinematicPath, this );
  }
}

void PlanningVisualizer::update( float dt )
{
  incoming_kinematic_path_message_.lock();

  if ( !animating_path_ && new_kinematic_path_ )
  {
    displaying_kinematic_path_message_ = incoming_kinematic_path_message_;

    animating_path_ = true;
    new_kinematic_path_ = false;
    current_state_ = -1;
    current_state_time_ = state_display_time_ + 1.0f;

    kinematic_model_->computeTransforms(&displaying_kinematic_path_message_.start_state.vals[0]);
    robot_->update( kinematic_model_, target_frame_ );
  }

  incoming_kinematic_path_message_.unlock();

  if ( animating_path_ )
  {
    if ( current_state_time_ > state_display_time_ )
    {
      ++current_state_;

      calculateRobotPosition();

      if ( (size_t)current_state_ < displaying_kinematic_path_message_.path.get_states_size() )
      {
        int group_id = kinematic_model_->getGroupID( displaying_kinematic_path_message_.model_name );
        kinematic_model_->computeTransforms(&displaying_kinematic_path_message_.path.states[ current_state_ ].vals[0], group_id);
        robot_->update( kinematic_model_, target_frame_ );

        causeRender();
      }
      else
      {
        animating_path_ = false;
      }

      current_state_time_ = 0.0f;
    }

    current_state_time_ += dt;
  }
}

void PlanningVisualizer::calculateRobotPosition()
{
  tf::Stamped<tf::Pose> pose( btTransform( btQuaternion( 0, 0, 0 ), btVector3( 0, 0, 0 ) ), ros::Time(0ULL), displaying_kinematic_path_message_.frame_id );

  try
  {
    tf_->transformPose( target_frame_, pose, pose );
  }
  catch(tf::TransformException& e)
  {
    ROS_ERROR( "Error transforming from frame '%s' to frame '%s'\n", pose.frame_id_.c_str(), target_frame_.c_str() );
  }

  Ogre::Vector3 position( pose.getOrigin().x(), pose.getOrigin().y(), pose.getOrigin().z() );
  robotToOgre( position );

  btScalar yaw, pitch, roll;
  pose.getBasis().getEulerZYX( yaw, pitch, roll );
  Ogre::Matrix3 orientation;
  orientation.FromEulerAnglesYXZ( Ogre::Radian( yaw ), Ogre::Radian( pitch ), Ogre::Radian( roll ) );

  robot_->setPosition( position );
  robot_->setOrientation( orientation );
}

void PlanningVisualizer::incomingKinematicPath()
{
  new_kinematic_path_ = true;
}

void PlanningVisualizer::targetFrameChanged()
{
  calculateRobotPosition();
}

void PlanningVisualizer::createProperties()
{
  visual_enabled_property_ = property_manager_->createProperty<BoolProperty>( "Visual Enabled", property_prefix_, boost::bind( &PlanningVisualizer::isVisualVisible, this ),
                                                                               boost::bind( &PlanningVisualizer::setVisualVisible, this, _1 ), parent_category_, this );
  collision_enabled_property_ = property_manager_->createProperty<BoolProperty>( "Collision Enabled", property_prefix_, boost::bind( &PlanningVisualizer::isCollisionVisible, this ),
                                                                                 boost::bind( &PlanningVisualizer::setCollisionVisible, this, _1 ), parent_category_, this );
  state_display_time_property_ = property_manager_->createProperty<FloatProperty>( "State Display Time", property_prefix_, boost::bind( &PlanningVisualizer::getStateDisplayTime, this ),
                                                                                  boost::bind( &PlanningVisualizer::setStateDisplayTime, this, _1 ), parent_category_, this );
  state_display_time_property_->setMin( 0.0001 );
  robot_description_property_ = property_manager_->createProperty<StringProperty>( "Robot Description", property_prefix_, boost::bind( &PlanningVisualizer::getRobotDescription, this ),
                                                                                   boost::bind( &PlanningVisualizer::setRobotDescription, this, _1 ), parent_category_, this );
  topic_property_ = property_manager_->createProperty<ROSTopicStringProperty>( "Topic", property_prefix_, boost::bind( &PlanningVisualizer::getTopic, this ),
                                                                               boost::bind( &PlanningVisualizer::setTopic, this, _1 ), parent_category_, this );
}

const char* PlanningVisualizer::getDescription()
{
  return "Displays a planned path given my a robot_msgs::DisplayKinematicPath message.";
}

} // namespace ogre_vis


