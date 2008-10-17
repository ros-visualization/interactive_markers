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

#include "robot_model_visualizer.h"
#include "common.h"
#include "helpers/robot.h"
#include "properties/property.h"
#include "properties/property_manager.h"

#include "ogre_tools/axes.h"

#include "urdf/URDF.h"
#include "rosTF/rosTF.h"

#include <Ogre.h>
#include <wx/wx.h>
#include <wx/propgrid/propgrid.h>
#include <wx/propgrid/advprops.h>
#include <wx/confbase.h>


namespace ogre_vis
{

RobotModelVisualizer::RobotModelVisualizer( const std::string& name, VisualizationManager* manager )
: VisualizerBase( name, manager )
, has_new_transforms_( false )
, time_since_last_transform_( 0.0f )
, update_rate_( 0.1f )
, visual_enabled_property_( NULL )
, collision_enabled_property_( NULL )
, update_rate_property_( NULL )
, robot_description_property_( NULL )
{
  robot_ = new Robot( scene_manager_, "Robot: " + name_ );

  setVisualVisible( true );
  setCollisionVisible( false );
  robot_->setUserData( Ogre::Any( (void*)this ) );
}

RobotModelVisualizer::~RobotModelVisualizer()
{
  delete robot_;
}

void RobotModelVisualizer::setRobotDescription( const std::string& description_param )
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

void RobotModelVisualizer::setVisualVisible( bool visible )
{
  robot_->setVisualVisible( visible );

  if ( visual_enabled_property_ )
  {
    visual_enabled_property_->changed();
  }

  causeRender();
}

void RobotModelVisualizer::setCollisionVisible( bool visible )
{
  robot_->setCollisionVisible( visible );

  if ( collision_enabled_property_ )
  {
    collision_enabled_property_->changed();
  }

  causeRender();
}

void RobotModelVisualizer::setUpdateRate( float rate )
{
  update_rate_ = rate;

  if ( update_rate_property_ )
  {
    update_rate_property_->changed();
  }

  causeRender();
}

bool RobotModelVisualizer::isVisualVisible()
{
  return robot_->isVisualVisible();
}

bool RobotModelVisualizer::isCollisionVisible()
{
  return robot_->isCollisionVisible();
}

void RobotModelVisualizer::load()
{
  std::string content;
  ros_node_->get_param(description_param_, content);
  robot_desc::URDF file;
  file.loadString(content.c_str());

  robot_->load( &file );
  robot_->update( tf_, target_frame_ );
}

void RobotModelVisualizer::onEnable()
{
  load();
  robot_->setVisible( true );
}

void RobotModelVisualizer::onDisable()
{
  robot_->setVisible( false );
}

void RobotModelVisualizer::update( float dt )
{
  time_since_last_transform_ += dt;

  if ( has_new_transforms_ || (update_rate_ > 0.0001f && time_since_last_transform_ >= update_rate_) )
  {
    robot_->update( tf_, target_frame_ );
    causeRender();

    has_new_transforms_ = false;
    time_since_last_transform_ = 0.0f;
  }
}

void RobotModelVisualizer::targetFrameChanged()
{
  has_new_transforms_ = true;
}

void RobotModelVisualizer::createProperties()
{
  visual_enabled_property_ = property_manager_->createProperty<BoolProperty>( "Visual Enabled", property_prefix_, boost::bind( &RobotModelVisualizer::isVisualVisible, this ),
                                                                               boost::bind( &RobotModelVisualizer::setVisualVisible, this, _1 ), parent_category_, this );
  collision_enabled_property_ = property_manager_->createProperty<BoolProperty>( "Collision Enabled", property_prefix_, boost::bind( &RobotModelVisualizer::isCollisionVisible, this ),
                                                                                 boost::bind( &RobotModelVisualizer::setCollisionVisible, this, _1 ), parent_category_, this );
  update_rate_property_ = property_manager_->createProperty<FloatProperty>( "Update Rate", property_prefix_, boost::bind( &RobotModelVisualizer::getUpdateRate, this ),
                                                                                  boost::bind( &RobotModelVisualizer::setUpdateRate, this, _1 ), parent_category_, this );
  update_rate_property_->setMin( 0.0 );

  robot_description_property_ = property_manager_->createProperty<StringProperty>( "Robot Description", property_prefix_, boost::bind( &RobotModelVisualizer::getRobotDescription, this ),
                                                                                   boost::bind( &RobotModelVisualizer::setRobotDescription, this, _1 ), parent_category_, this );

  robot_->setPropertyManager( property_manager_, parent_category_ );
}

} // namespace ogre_vis

