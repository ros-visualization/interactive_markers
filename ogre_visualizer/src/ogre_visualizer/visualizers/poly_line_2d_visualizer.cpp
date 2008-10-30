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

#include "poly_line_2d_visualizer.h"
#include "properties/property.h"
#include "properties/property_manager.h"
#include "common.h"

#include "ogre_tools/arrow.h"

#include <ros/node.h>
#include <tf/transform_listener.h>

#include <boost/bind.hpp>

#include <OgreSceneNode.h>
#include <OgreSceneManager.h>
#include <OgreManualObject.h>

#include <ogre_tools/point_cloud.h>

namespace ogre_vis
{

PolyLine2DVisualizer::PolyLine2DVisualizer( const std::string& name, VisualizationManager* manager )
: VisualizerBase( name, manager )
, color_( 0.1f, 1.0f, 0.0f )
, render_operation_( Ogre::RenderOperation::OT_LINE_LIST )
, loop_( false )
, override_color_( false )
, new_message_( false )
, color_property_( NULL )
, topic_property_( NULL )
, override_color_property_( NULL )
, loop_property_( NULL )
, render_operation_property_( NULL )
, point_size_property_( NULL )
{
  scene_node_ = scene_manager_->getRootSceneNode()->createChildSceneNode();

  static int count = 0;
  std::stringstream ss;
  ss << "PolyLine2D" << count++;
  manual_object_ = scene_manager_->createManualObject( ss.str() );
  manual_object_->setDynamic( true );
  scene_node_->attachObject( manual_object_ );

  cloud_ = new ogre_tools::PointCloud( scene_manager_, scene_node_ );
  setPointSize( 0.1f );
}

PolyLine2DVisualizer::~PolyLine2DVisualizer()
{
  unsubscribe();
  clear();

  scene_manager_->destroyManualObject( manual_object_ );

  delete cloud_;
}

void PolyLine2DVisualizer::clear()
{
  manual_object_->clear();
  cloud_->clear();
}

void PolyLine2DVisualizer::setTopic( const std::string& topic )
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

void PolyLine2DVisualizer::setColor( const Color& color )
{
  color_ = color;

  if ( color_property_ )
  {
    color_property_->changed();
  }

  processMessage();
  causeRender();
}

void PolyLine2DVisualizer::setOverrideColor( bool override )
{
  override_color_ = override;

  if ( override_color_property_ )
  {
    override_color_property_->changed();
  }

  processMessage();
  causeRender();
}

void PolyLine2DVisualizer::setRenderOperation( int op )
{
  render_operation_ = op;

  if ( render_operation_property_ )
  {
    render_operation_property_->changed();
  }

  processMessage();
  causeRender();
}

void PolyLine2DVisualizer::setLoop( bool loop )
{
  loop_ = loop;

  if ( loop_property_ )
  {
    loop_property_->changed();
  }

  processMessage();
  causeRender();
}

void PolyLine2DVisualizer::setPointSize( float size )
{
  point_size_ = size;

  if ( point_size_property_ )
  {
    point_size_property_->changed();
  }

  cloud_->setBillboardDimensions( size, size );
  causeRender();
}

void PolyLine2DVisualizer::subscribe()
{
  if ( !isEnabled() )
  {
    return;
  }

  if ( !topic_.empty() )
  {
    ros_node_->subscribe( topic_, message_, &PolyLine2DVisualizer::incomingMessage, this, 1 );
  }
}

void PolyLine2DVisualizer::unsubscribe()
{
  if ( !topic_.empty() )
  {
    ros_node_->unsubscribe( topic_, &PolyLine2DVisualizer::incomingMessage, this );
  }
}

void PolyLine2DVisualizer::onEnable()
{
  scene_node_->setVisible( true );
  subscribe();
}

void PolyLine2DVisualizer::onDisable()
{
  unsubscribe();
  clear();
  scene_node_->setVisible( false );
}

void PolyLine2DVisualizer::createProperties()
{
  override_color_property_ = property_manager_->createProperty<BoolProperty>( "Override Color", property_prefix_, boost::bind( &PolyLine2DVisualizer::getOverrideColor, this ),
                                                                              boost::bind( &PolyLine2DVisualizer::setOverrideColor, this, _1 ), parent_category_, this );
  color_property_ = property_manager_->createProperty<ColorProperty>( "Color", property_prefix_, boost::bind( &PolyLine2DVisualizer::getColor, this ),
                                                                      boost::bind( &PolyLine2DVisualizer::setColor, this, _1 ), parent_category_, this );

  loop_property_ = property_manager_->createProperty<BoolProperty>( "Loop", property_prefix_, boost::bind( &PolyLine2DVisualizer::getLoop, this ),
                                                                    boost::bind( &PolyLine2DVisualizer::setLoop, this, _1 ), parent_category_, this );

  render_operation_property_ = property_manager_->createProperty<EnumProperty>( "Render Operation", property_prefix_, boost::bind( &PolyLine2DVisualizer::getRenderOperation, this ),
                                                                                boost::bind( &PolyLine2DVisualizer::setRenderOperation, this, _1 ), parent_category_, this );
  render_operation_property_->addOption( "Lines", Ogre::RenderOperation::OT_LINE_LIST );
  render_operation_property_->addOption( "Points", Ogre::RenderOperation::OT_POINT_LIST );

  point_size_property_ = property_manager_->createProperty<FloatProperty>( "Point Size", property_prefix_, boost::bind( &PolyLine2DVisualizer::getPointSize, this ),
                                                                      boost::bind( &PolyLine2DVisualizer::setPointSize, this, _1 ), parent_category_, this );

  topic_property_ = property_manager_->createProperty<ROSTopicStringProperty>( "Topic", property_prefix_, boost::bind( &PolyLine2DVisualizer::getTopic, this ),
                                                                                boost::bind( &PolyLine2DVisualizer::setTopic, this, _1 ), parent_category_, this );
}

void PolyLine2DVisualizer::fixedFrameChanged()
{
  clear();
}

void PolyLine2DVisualizer::update( float dt )
{
  if ( new_message_ )
  {
    processMessage();

    new_message_ = false;

    causeRender();
  }
}

void PolyLine2DVisualizer::processMessage()
{
  message_.lock();

  clear();

  tf::Stamped<tf::Pose> pose( btTransform( btQuaternion( 0.0f, 0.0f, 0.0f ), btVector3( 0.0f, 0.0f, 0.0f ) ),
                                ros::Time(0), "map" );

  try
  {
    tf_->transformPose( fixed_frame_, pose, pose );
  }
  catch(tf::TransformException& e)
  {
    ROS_ERROR( "Error transforming from frame 'map' to frame '%s'\n", fixed_frame_.c_str() );
  }

  Ogre::Vector3 position = Ogre::Vector3( pose.getOrigin().x(), pose.getOrigin().y(), pose.getOrigin().z() );
  robotToOgre( position );

  btQuaternion quat;
  pose.getBasis().getRotation( quat );
  Ogre::Quaternion orientation( Ogre::Quaternion::IDENTITY );
  ogreToRobot( orientation );
  orientation = Ogre::Quaternion( quat.w(), quat.x(), quat.y(), quat.z() ) * orientation;
  robotToOgre( orientation );

  scene_node_->setPosition( position );
  scene_node_->setOrientation( orientation );

  manual_object_->clear();

  Ogre::ColourValue color;
  if ( override_color_ )
  {
    color = Ogre::ColourValue( color_.r_, color_.g_, color_.b_ );
  }
  else
  {
    color = Ogre::ColourValue( message_.color.r, message_.color.g, message_.color.b );
  }

  uint32_t num_points = message_.get_points_size();
  if ( render_operation_ == Ogre::RenderOperation::OT_POINT_LIST )
  {
    typedef std::vector< ogre_tools::PointCloud::Point > V_Point;
    V_Point points;
    points.resize( num_points );
    for(uint32_t i = 0; i < num_points; i++)
    {
      ogre_tools::PointCloud::Point& current_point = points[ i ];

      current_point.x_ = -message_.points[i].y;
      current_point.y_ = 0.0f;
      current_point.z_ = -message_.points[i].x;
      current_point.r_ = color.r;
      current_point.g_ = color.g;
      current_point.b_ = color.b;
    }

    cloud_->clear();

    if ( !points.empty() )
    {
      cloud_->addPoints( &points.front(), points.size() );
    }
  }
  else
  {
    manual_object_->estimateVertexCount( num_points );
    manual_object_->begin( "BaseWhiteNoLighting", (Ogre::RenderOperation::OperationType)render_operation_ );
    for( uint32_t i=0; i < num_points; ++i)
    {
      manual_object_->position(-message_.points[i].y, 0.0f, -message_.points[i].x);
      manual_object_->colour( color );
    }

    if ( loop_ && num_points > 0 )
    {
      manual_object_->position(-message_.points[num_points - 1].y, 0.0f, -message_.points[num_points - 1].x);
      manual_object_->colour( color );
      manual_object_->position(-message_.points[0].y, 0.0f, -message_.points[0].x);
      manual_object_->colour( color );
    }

    manual_object_->end();
  }

  message_.unlock();
}

void PolyLine2DVisualizer::incomingMessage()
{
  new_message_ = true;
}

void PolyLine2DVisualizer::reset()
{
  clear();
}

} // namespace ogre_vis

