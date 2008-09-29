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
#include <wx/confbase.h>

#define VISUAL_ENABLED_PROPERTY wxT("Show Visual")
#define COLLISION_ENABLED_PROPERTY wxT("Show Collision")
#define UPDATE_RATE_PROPERTY wxT("Update Rate")
#define ROBOT_DESCRIPTION_PROPERTY wxT("Robot Description")

namespace ogre_vis
{

RobotModelVisualizer::RobotModelVisualizer( Ogre::SceneManager* scene_manager, ros::node* node, rosTFClient* tf_client, const std::string& name )
: VisualizerBase( scene_manager, node, tf_client, name )
, has_new_transforms_( false )
, time_since_last_transform_( 0.0f )
, update_rate_( 0.1f )
{
  robot_ = new Robot( scene_manager );

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

  if ( property_grid_ )
  {
    property_grid_->SetPropertyValue( property_grid_->GetProperty( property_prefix_ + ROBOT_DESCRIPTION_PROPERTY ), wxString::FromAscii( description_param_.c_str() ) );
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

  if ( property_grid_ )
  {
    property_grid_->SetPropertyValue( property_grid_->GetProperty( property_prefix_ + VISUAL_ENABLED_PROPERTY ), visible );
  }
}

void RobotModelVisualizer::setCollisionVisible( bool visible )
{
  robot_->setCollisionVisible( visible );

  if ( property_grid_ )
  {
    property_grid_->SetPropertyValue( property_grid_->GetProperty( property_prefix_ + COLLISION_ENABLED_PROPERTY ), visible );
  }
}

void RobotModelVisualizer::setUpdateRate( float rate )
{
  update_rate_ = rate;

  if ( property_grid_ )
  {
    property_grid_->SetPropertyValue( property_grid_->GetProperty( property_prefix_ + UPDATE_RATE_PROPERTY ), update_rate_ );
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
    robot_->update( tf_client_, target_frame_ );
    causeRender();

    has_new_transforms_ = false;
    time_since_last_transform_ = 0.0f;
  }
}

void RobotModelVisualizer::targetFrameChanged()
{
  has_new_transforms_ = true;
}

void RobotModelVisualizer::fillPropertyGrid()
{
  property_grid_->Append( new wxStringProperty( ROBOT_DESCRIPTION_PROPERTY, property_prefix_ + ROBOT_DESCRIPTION_PROPERTY, wxString::FromAscii( description_param_.c_str() ) ) );
  wxPGProperty* prop = property_grid_->Append( new wxBoolProperty( VISUAL_ENABLED_PROPERTY, property_prefix_ + VISUAL_ENABLED_PROPERTY, robot_->isVisualVisible() ) );
  prop->SetAttribute( wxPG_BOOL_USE_CHECKBOX, true );
  prop = property_grid_->Append( new wxBoolProperty( COLLISION_ENABLED_PROPERTY, property_prefix_ + COLLISION_ENABLED_PROPERTY, robot_->isCollisionVisible() ) );
  prop->SetAttribute( wxPG_BOOL_USE_CHECKBOX, true );

  property_grid_->Append( new wxFloatProperty( UPDATE_RATE_PROPERTY, property_prefix_ + UPDATE_RATE_PROPERTY, update_rate_ ) );

  property_grid_->SetPropertyAttribute( VISUAL_ENABLED_PROPERTY, wxPG_BOOL_USE_CHECKBOX, true );
  property_grid_->SetPropertyAttribute( COLLISION_ENABLED_PROPERTY, wxPG_BOOL_USE_CHECKBOX, true );
}

void RobotModelVisualizer::propertyChanged( wxPropertyGridEvent& event )
{
  wxPGProperty* property = event.GetProperty();

  const wxString& name = property->GetName();
  wxVariant value = property->GetValue();

  if ( name == property_prefix_ + VISUAL_ENABLED_PROPERTY )
  {
    bool visible = value.GetBool();
    setVisualVisible( visible );
  }
  else if ( name == property_prefix_ + COLLISION_ENABLED_PROPERTY )
  {
    bool visible = value.GetBool();
    setCollisionVisible( visible );
  }
  else if ( name == property_prefix_ + UPDATE_RATE_PROPERTY )
  {
    setUpdateRate( value.GetDouble() );
  }
  else if ( name == property_prefix_ + ROBOT_DESCRIPTION_PROPERTY )
  {
    wxString prop = value.GetString();
    setRobotDescription( (const char*)prop.fn_str() );
  }

  causeRender();
}

void RobotModelVisualizer::loadProperties( wxConfigBase* config )
{
  bool visual_enabled, collision_enabled;
  double update_rate;
  wxString robot_description;

  {
    config->Read( VISUAL_ENABLED_PROPERTY, &visual_enabled, robot_->isVisualVisible() );
    config->Read( COLLISION_ENABLED_PROPERTY, &collision_enabled, robot_->isCollisionVisible() );
  }

  {
    config->Read( UPDATE_RATE_PROPERTY, &update_rate, update_rate_ );
  }

  {
    config->Read( ROBOT_DESCRIPTION_PROPERTY, &robot_description, wxString::FromAscii( description_param_.c_str() ) );
  }

  setVisualVisible( visual_enabled );
  setCollisionVisible( collision_enabled );
  setUpdateRate( update_rate );
  setRobotDescription( (const char*)robot_description.fn_str() );
}

void RobotModelVisualizer::saveProperties( wxConfigBase* config )
{
  config->Write( VISUAL_ENABLED_PROPERTY, robot_->isVisualVisible() );
  config->Write( COLLISION_ENABLED_PROPERTY, robot_->isCollisionVisible() );
  config->Write( UPDATE_RATE_PROPERTY, update_rate_ );
  config->Write( ROBOT_DESCRIPTION_PROPERTY, wxString::FromAscii( description_param_.c_str() ) );
}

} // namespace ogre_vis

