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

#include "marker_visualizer.h"
#include "../common.h"

#include <ogre_tools/arrow.h>
#include <ogre_tools/super_ellipsoid.h>

#include <ros/node.h>
#include <rosTF/rosTF.h>

#include <Ogre.h>


namespace ogre_vis
{

MarkerVisualizer::MarkerVisualizer( Ogre::SceneManager* scene_manager, ros::node* node, rosTFClient* tf_client, const std::string& name, bool enabled )
: VisualizerBase( scene_manager, node, tf_client, name, enabled )
{
  scene_node_ = scene_manager_->getRootSceneNode()->createChildSceneNode();

  if ( isEnabled() )
  {
    onEnable();
  }
}

MarkerVisualizer::~MarkerVisualizer()
{
  unsubscribe();
}

void MarkerVisualizer::clearMarkers()
{
  M_IDToObject::iterator marker_it = markers_.begin();
  M_IDToObject::iterator marker_end = markers_.end();
  for ( ; marker_it != marker_end; ++marker_it )
  {
    delete marker_it->second;
  }
  markers_.clear();
}

void MarkerVisualizer::onEnable()
{
  subscribe();

  scene_node_->setVisible( true );
}

void MarkerVisualizer::onDisable()
{
  unsubscribe();

  clearMarkers();

  scene_node_->setVisible( false );
}

void MarkerVisualizer::subscribe()
{
  if ( !isEnabled() )
  {
    return;
  }

  ros_node_->subscribe("visualizationMarker", current_message_, &MarkerVisualizer::incomingMarker,this, 0);
}

void MarkerVisualizer::unsubscribe()
{
  ros_node_->unsubscribe( "visualizationMarker" );
}

void MarkerVisualizer::incomingMarker()
{
  message_queue_.push_back( current_message_ );
}

void MarkerVisualizer::processMessage( const std_msgs::VisualizationMarker& message )
{
  switch ( message.action )
  {
  case MarkerActions::Add:
    processAdd( message );
    break;

  case MarkerActions::Modify:
    processModify( message );
    break;

  case MarkerActions::Delete:
    processDelete( message );
    break;

  default:
    printf( "Unknown marker action: %d\n", message.action );
  }
}

void MarkerVisualizer::processAdd( const std_msgs::VisualizationMarker& message )
{
  {
    M_IDToObject::iterator it = markers_.find( message.id );
    if ( it != markers_.end() )
    {
      printf( "Marker with id %d already exists, replacing...\n", message.id );

      delete it->second;
      markers_.erase( it );
    }
  }

  ogre_tools::Object* object = NULL;

  switch ( message.type )
  {
  case MarkerTypes::Cube:
    {
      ogre_tools::SuperEllipsoid* cube = new ogre_tools::SuperEllipsoid( scene_manager_, scene_node_ );
      cube->create( ogre_tools::SuperEllipsoid::Cube, 10, Ogre::Vector3( 1.0f, 1.0f, 1.0f ) );

      object = cube;
    }
    break;

  case MarkerTypes::Sphere:
    {
      ogre_tools::SuperEllipsoid* sphere = new ogre_tools::SuperEllipsoid( scene_manager_, scene_node_ );
      sphere->create( ogre_tools::SuperEllipsoid::Sphere, 20, Ogre::Vector3( 1.0f, 1.0f, 1.0f ) );

      object = sphere;
    }
    break;

  case MarkerTypes::Arrow:
    {
      object = new ogre_tools::Arrow( scene_manager_, scene_node_, 0.8, 0.5, 0.2, 1.0 );
    }
    break;

  default:
    printf( "Unknown marker type: %d\n", message.type );
  }

  if ( object )
  {
    markers_.insert( std::make_pair( message.id, object ) );

    setCommonValues( message, object );

    causeRender();
  }
}

void MarkerVisualizer::processModify( const std_msgs::VisualizationMarker& message )
{
  M_IDToObject::iterator it = markers_.find( message.id );
  if ( it == markers_.end() )
  {
    printf( "Tried to modify marker with id %d that does not exist\n", message.id );
    return;
  }

  setCommonValues( message, it->second );

  causeRender();
}

void MarkerVisualizer::processDelete( const std_msgs::VisualizationMarker& message )
{
  M_IDToObject::iterator it = markers_.find( message.id );
  if ( it != markers_.end() )
  {
    delete it->second;
    markers_.erase( it );
  }

  causeRender();
}

void MarkerVisualizer::setCommonValues( const std_msgs::VisualizationMarker& message, ogre_tools::Object* object )
{
  uint32_t frameId = 1;
  /*if ( !message.frame.empty() )
  {
    frameId = tf_client_->lookup( message.frame );
  }*/

  libTF::TFPoint tf_point;
  tf_point.x = message.x;
  tf_point.y = message.y;
  tf_point.z = message.z;
  tf_point.time = 0;
  tf_point.frame = frameId;
  try
  {
    tf_point = tf_client_->transformPoint( target_frame_, tf_point );
  }
  catch(libTF::TransformReference::LookupException& e)
  {
    printf( "Error transforming marker '%d': %s\n", message.id, e.what() );
  }
  catch(libTF::TransformReference::ConnectivityException& e)
  {
    printf( "Error transforming marker '%d': %s\n", message.id, e.what() );
  }
  catch(libTF::TransformReference::ExtrapolateException& e)
  {
    printf( "Error transforming marker '%d': %s\n", message.id, e.what() );
  }

  libTF::TFEulerYPR tf_eulers;
  tf_eulers.yaw = message.yaw;
  tf_eulers.pitch = message.pitch;
  tf_eulers.roll = message.roll;
  tf_eulers.time = 0;
  tf_eulers.frame = frameId;
  try
  {
    tf_eulers = tf_client_->transformEulerYPR( target_frame_, tf_eulers );
  }
  catch(libTF::TransformReference::LookupException& e)
  {
    printf( "Error transforming marker '%d': %s\n", message.id, e.what() );
  }
  catch(libTF::TransformReference::ConnectivityException& e)
  {
    printf( "Error transforming marker '%d': %s\n", message.id, e.what() );
  }
  catch(libTF::TransformReference::ExtrapolateException& e)
  {
    printf( "Error transforming marker '%d': %s\n", message.id, e.what() );
  }

  Ogre::Vector3 position( tf_point.x, tf_point.y, tf_point.z );
  robotToOgre( position );

  Ogre::Matrix3 orientation;
  orientation.FromEulerAnglesYXZ( Ogre::Radian( tf_eulers.yaw ), Ogre::Radian( tf_eulers.pitch ), Ogre::Radian( tf_eulers.roll ) );
  //Ogre::Matrix3 orientation( OgreMatrixFromRobotEulers( tf_eulers.yaw, tf_eulers.pitch, tf_eulers.roll ) );
  Ogre::Vector3 scale( message.xScale, message.yScale, message.zScale );
  robotToOgre( scale );

  scale.x = fabsf( scale.x );
  scale.y = fabsf( scale.y );
  scale.z = fabsf( scale.z );

  object->setPosition( position );
  object->setOrientation( orientation );
  object->setScale( scale );
  object->setColor( message.r / 255.0f, message.g / 255.0f, message.b / 255.0f );
}

void MarkerVisualizer::update( float dt )
{
  current_message_.lock();

  if ( !message_queue_.empty() )
  {
    V_MarkerMessage::iterator message_it = message_queue_.begin();
    V_MarkerMessage::iterator message_end = message_queue_.end();
    for ( ; message_it != message_end; ++message_it )
    {
      std_msgs::VisualizationMarker& marker = *message_it;

      processMessage( marker );
    }

    message_queue_.clear();
  }

  current_message_.unlock();
}

} // namespace ogre_vis
