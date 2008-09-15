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

#include "ogre_tools/axes.h"

#include "urdf/URDF.h"
#include "rosTF/rosTF.h"

#include <Ogre.h>
#include <wx/wx.h>
#include <wx/propgrid/propgrid.h>
#include <wx/propgrid/advprops.h>

#define VISUAL_ENABLED_PROPERTY wxT("Show Visual")
#define COLLISION_ENABLED_PROPERTY wxT("Show Collision")
#define UPDATE_RATE_PROPERTY wxT("Update Rate")

namespace ogre_vis
{

RobotModelVisualizer::RobotModelVisualizer( Ogre::SceneManager* scene_manager, ros::node* node, rosTFClient* tf_client, const std::string& name )
: VisualizerBase( scene_manager, node, tf_client, name )
, has_new_transforms_( false )
, initialized_( false )
, time_since_last_transform_( 0.0f )
, update_rate_( 0.1f )
{
  robot_ = new Robot( scene_manager );

  robot_->setVisualVisible( true );
  robot_->setCollisionVisible( false );
  robot_->setUserData( (void*)this );
}

RobotModelVisualizer::~RobotModelVisualizer()
{
  unsubscribe();

  delete robot_;
}

void RobotModelVisualizer::initialize( const std::string& description_param, const std::string& transform_topic )
{
  transform_topic_ = transform_topic;
  description_param_ = description_param;

  initialized_ = true;

  if ( isEnabled() )
  {
    onEnable();
    causeRender();
  }
  else
  {
    onDisable();
  }
}

void RobotModelVisualizer::load()
{
  std::string content;
  ros_node_->get_param(description_param_, content);
  robot_desc::URDF file;
  file.loadString(content.c_str());

  robot_->load( &file );
  robot_->update( tf_client_, target_frame_ );
}

void RobotModelVisualizer::onEnable()
{
  if ( !initialized_ )
  {
    return;
  }

  subscribe();

  load();
  robot_->setVisible( true );
}

void RobotModelVisualizer::onDisable()
{
  if ( !initialized_ )
  {
    return;
  }

  unsubscribe();
  robot_->setVisible( false );
}

void RobotModelVisualizer::subscribe()
{
  if ( !isEnabled() )
  {
    return;
  }

  if ( initialized_ && !transform_topic_.empty() )
  {
    ros_node_->subscribe( transform_topic_, message_, &RobotModelVisualizer::incomingTransform, this, 5 );
  }
}

void RobotModelVisualizer::unsubscribe()
{
  if ( initialized_ && !transform_topic_.empty() )
  {
    ros_node_->unsubscribe( transform_topic_ );
  }
}

void RobotModelVisualizer::incomingTransform()
{
  has_new_transforms_ = true;
}

void RobotModelVisualizer::update( float dt )
{
  time_since_last_transform_ += dt;

  message_.lock();

  if ( has_new_transforms_ || (update_rate_ > 0.0f && time_since_last_transform_ >= update_rate_) )
  {
    robot_->update( tf_client_, target_frame_ );
    causeRender();

    has_new_transforms_ = false;
    time_since_last_transform_ = 0.0f;
  }

  message_.unlock();
}

void RobotModelVisualizer::fillPropertyGrid( wxPropertyGrid* property_grid )
{
  property_grid->Append( new wxBoolProperty( VISUAL_ENABLED_PROPERTY, wxPG_LABEL, robot_->isVisualVisible() ) );
  property_grid->Append( new wxBoolProperty( COLLISION_ENABLED_PROPERTY, wxPG_LABEL, robot_->isCollisionVisible() ) );
  property_grid->Append( new wxFloatProperty( UPDATE_RATE_PROPERTY, wxPG_LABEL, update_rate_ ) );

  property_grid->SetPropertyAttribute( VISUAL_ENABLED_PROPERTY, wxPG_BOOL_USE_CHECKBOX, true );
  property_grid->SetPropertyAttribute( COLLISION_ENABLED_PROPERTY, wxPG_BOOL_USE_CHECKBOX, true );
}

void RobotModelVisualizer::propertyChanged( wxPropertyGridEvent& event )
{
  wxPGProperty* property = event.GetProperty();

  const wxString& name = property->GetName();
  wxVariant value = property->GetValue();

  if ( name == VISUAL_ENABLED_PROPERTY )
  {
    bool visible = value.GetBool();
    robot_->setVisualVisible( visible );
  }
  else if ( name == COLLISION_ENABLED_PROPERTY )
  {
    bool visible = value.GetBool();
    robot_->setCollisionVisible( visible );
  }
  else if ( name == UPDATE_RATE_PROPERTY )
  {
    update_rate_ = value.GetDouble();
  }

  causeRender();
}

} // namespace ogre_vis

