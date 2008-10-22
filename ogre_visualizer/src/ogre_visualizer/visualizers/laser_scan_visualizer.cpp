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

#include "laser_scan_visualizer.h"
#include "properties/property.h"
#include "properties/property_manager.h"
#include "common.h"
#include "ros_topic_property.h"

#include "ros/node.h"
#include "ogre_tools/point_cloud.h"

#include <tf/transform_listener.h>
#include <std_msgs/PointCloud.h>

#include <wx/wx.h>
#include <wx/propgrid/propgrid.h>
#include <wx/propgrid/advprops.h>
#include <wx/confbase.h>

#include <OgreSceneNode.h>
#include <OgreSceneManager.h>

namespace ogre_vis
{

LaserScanVisualizer::LaserScanVisualizer( const std::string& name, VisualizationManager* manager )
: VisualizerBase( name, manager )
, color_( 1.0f, 0.0f, 0.0f )
, intensity_min_( 999999.0f )
, intensity_max_( -999999.0f )
, point_decay_time_( 20.0f )
, style_( Billboards )
, billboard_size_( 0.003 )
, scan_topic_property_( NULL )
, cloud_topic_property_( NULL )
, billboard_size_property_( NULL )
, decay_time_property_( NULL )
, color_property_( NULL )
, style_property_( NULL )
{
  cloud_ = new ogre_tools::PointCloud( scene_manager_ );

  setStyle( style_ );
  setBillboardSize( billboard_size_ );
}

LaserScanVisualizer::~LaserScanVisualizer()
{
  unsubscribe();

  delete cloud_;
}

void LaserScanVisualizer::setCloudTopic( const std::string& topic )
{
  unsubscribe();

  cloud_topic_ = topic;

  subscribe();

  if ( cloud_topic_property_ )
  {
    cloud_topic_property_->changed();
  }

  causeRender();
}

void LaserScanVisualizer::setScanTopic( const std::string& topic )
{
  unsubscribe();

  scan_topic_ = topic;

  subscribe();

  if ( scan_topic_property_ )
  {
    scan_topic_property_->changed();
  }

  causeRender();
}

void LaserScanVisualizer::setColor( const Color& color )
{
  color_ = color;

  if ( color_property_ )
  {
    color_property_->changed();
  }

  causeRender();
}

void LaserScanVisualizer::setStyle( int style )
{
  {
    RenderAutoLock render_lock( this );

    ROS_ASSERT(style < StyleCount);

    style_ = style;
    cloud_->setUsePoints( style == Points );
  }

  causeRender();

  if ( style_property_ )
  {
    style_property_->changed();
  }

  causeRender();
}

void LaserScanVisualizer::setBillboardSize( float size )
{
  {
    RenderAutoLock render_lock( this );

    billboard_size_ = size;
    cloud_->setBillboardDimensions( size, size );
  }

  causeRender();

  if ( billboard_size_property_ )
  {
    billboard_size_property_->changed();
  }

  causeRender();
}

void LaserScanVisualizer::setDecayTime( float time )
{
  point_decay_time_ = time;

  if ( decay_time_property_ )
  {
    decay_time_property_->changed();
  }

  causeRender();
}

void LaserScanVisualizer::onEnable()
{
  cloud_->setCloudVisible( true );
  subscribe();
}

void LaserScanVisualizer::onDisable()
{
  unsubscribe();

  cloud_->setCloudVisible( false );
  cloud_->clear();
  points_.clear();
  point_times_.clear();
  cloud_messages_.clear();
}

void LaserScanVisualizer::subscribe()
{
  if ( !isEnabled() )
  {
    return;
  }

  if ( !cloud_topic_.empty() )
  {
    ros_node_->subscribe( cloud_topic_, cloud_message_, &LaserScanVisualizer::incomingCloudCallback, this, 1 );
  }

  if ( !scan_topic_.empty() )
  {
    ros_node_->subscribe( scan_topic_, scan_message_, &LaserScanVisualizer::incomingScanCallback, this, 1 );
  }
}

void LaserScanVisualizer::unsubscribe()
{
  if ( !cloud_topic_.empty() )
  {
    ros_node_->unsubscribe( cloud_topic_, &LaserScanVisualizer::incomingCloudCallback, this );
  }

  if ( !scan_topic_.empty() )
  {
    ros_node_->unsubscribe( scan_topic_, &LaserScanVisualizer::incomingScanCallback, this );
  }
}

void LaserScanVisualizer::update( float dt )
{
  cloud_message_.lock();

  D_float::iterator it = point_times_.begin();
  D_float::iterator end = point_times_.end();
  for ( ; it != end; ++it )
  {
    *it += dt;
  }

  cullPoints();

  cloud_message_.unlock();
}

void LaserScanVisualizer::cullPoints()
{
  if ( point_decay_time_ == 0.0f )
  {
    return;
  }

  bool removed = false;
  while ( !point_times_.empty() && point_times_.front() > point_decay_time_ )
  {
    point_times_.pop_front();
    points_.pop_front();
    cloud_messages_.pop_front();

    removed = true;
  }

  if ( removed )
  {
    updateCloud();
  }
}

void LaserScanVisualizer::transformCloud( std_msgs::PointCloudFloat32& message )
{
  if ( point_decay_time_ == 0.0f )
  {
    points_.clear();
    point_times_.clear();
    cloud_messages_.clear();
  }

  // Push back before transforming.  This will perform a full copy.
  cloud_messages_.push_back( message );

  if ( message.header.frame_id.empty() )
  {
    message.header.frame_id = target_frame_;
  }

  try
  {
    std_msgs::PointCloud* casted_message = reinterpret_cast<std_msgs::PointCloud*>(&message);
    tf_->transformPointCloud(target_frame_, *casted_message, *casted_message);
  }
  catch(tf::TransformException& e)
  {
    ROS_ERROR( "Error transforming laser scan '%s', frame '%s' to frame '%s'\n", name_.c_str(), message.header.frame_id.c_str(), target_frame_.c_str() );
  }

  uint32_t point_count_ = message.get_pts_size();
  for(uint32_t i = 0; i < point_count_; i++)
  {
    float& intensity = message.chan[0].vals[i];
    // arbitrarily cap to 4096 for now
    intensity = std::min( intensity, 4096.0f );
    intensity_min_ = std::min( intensity_min_, intensity );
    intensity_max_ = std::max( intensity_max_, intensity );
  }

  float diff_intensity = intensity_max_ - intensity_min_;



  points_.push_back( V_Point() );
  V_Point& points = points_.back();
  points.resize( point_count_ );

  point_times_.push_back( 0.0f );
  for(uint32_t i = 0; i < point_count_; i++)
  {
    Ogre::Vector3 point( message.pts[i].x, message.pts[i].y, message.pts[i].z );
    robotToOgre( point );

    float intensity = message.chan[0].vals[i];

    float normalized_intensity = (diff_intensity > 0.0f) ? ( intensity - intensity_min_ ) / diff_intensity : 1.0f;

    Ogre::Vector3 color( color_.r_, color_.g_, color_.b_ );
    color *= normalized_intensity;

    ogre_tools::PointCloud::Point& current_point = points[ i ];
    current_point.x_ = point.x;
    current_point.y_ = point.y;
    current_point.z_ = point.z;
    current_point.r_ = color.x;
    current_point.g_ = color.y;
    current_point.b_ = color.z;
  }

  updateCloud();
}

void LaserScanVisualizer::updateCloud()
{
  {
    RenderAutoLock render_lock( this );

    cloud_->clear();

    if ( !points_.empty() )
    {
      DV_Point::iterator it = points_.begin();
      DV_Point::iterator end = points_.end();
      for ( ; it != end; ++it )
      {
        V_Point& points = *it;

        if ( !points.empty() )
        {
          cloud_->addPoints( &points.front(), points.size() );
        }
      }
    }
  }

  causeRender();
}

void LaserScanVisualizer::incomingCloudCallback()
{
  transformCloud( cloud_message_ );
}

void LaserScanVisualizer::incomingScanCallback()
{
  cloud_message_.lock();

  if ( scan_message_.header.frame_id.empty() )
  {
    scan_message_.header.frame_id = target_frame_;
  }

  laser_projection_.projectLaser( scan_message_, cloud_message_ );
  transformCloud( cloud_message_ );

  cloud_message_.unlock();
}

void LaserScanVisualizer::targetFrameChanged()
{
  cloud_message_.lock();

  D_CloudMessage messages;
  messages.swap( cloud_messages_ );
  points_.clear();
  point_times_.clear();
  cloud_messages_.clear();

  D_CloudMessage::iterator it = messages.begin();
  D_CloudMessage::iterator end = messages.end();
  for ( ; it != end; ++it )
  {
    transformCloud( *it );
  }

  cloud_message_.unlock();
}

void LaserScanVisualizer::createProperties()
{
  style_property_ = property_manager_->createProperty<EnumProperty>( "Style", property_prefix_, boost::bind( &LaserScanVisualizer::getStyle, this ),
                                                                     boost::bind( &LaserScanVisualizer::setStyle, this, _1 ), parent_category_, this );
  style_property_->addOption( "Billboards", Billboards );
  style_property_->addOption( "Points", Points );

  color_property_ = property_manager_->createProperty<ColorProperty>( "Color", property_prefix_, boost::bind( &LaserScanVisualizer::getColor, this ),
                                                                        boost::bind( &LaserScanVisualizer::setColor, this, _1 ), parent_category_, this );

  billboard_size_property_ = property_manager_->createProperty<FloatProperty>( "Billboard Size", property_prefix_, boost::bind( &LaserScanVisualizer::getBillboardSize, this ),
                                                                                boost::bind( &LaserScanVisualizer::setBillboardSize, this, _1 ), parent_category_, this );
  billboard_size_property_->setMin( 0.0001 );
  decay_time_property_ = property_manager_->createProperty<FloatProperty>( "Decay Time", property_prefix_, boost::bind( &LaserScanVisualizer::getDecayTime, this ),
                                                                                  boost::bind( &LaserScanVisualizer::setDecayTime, this, _1 ), parent_category_, this );
  billboard_size_property_->setMin( 0.0 );

  scan_topic_property_ = property_manager_->createProperty<ROSTopicStringProperty>( "Scan Topic", property_prefix_, boost::bind( &LaserScanVisualizer::getScanTopic, this ),
                                                                            boost::bind( &LaserScanVisualizer::setScanTopic, this, _1 ), parent_category_, this );
  cloud_topic_property_ = property_manager_->createProperty<ROSTopicStringProperty>( "Cloud Topic", property_prefix_, boost::bind( &LaserScanVisualizer::getCloudTopic, this ),
                                                                              boost::bind( &LaserScanVisualizer::setCloudTopic, this, _1 ), parent_category_, this );
}

} // namespace ogre_vis
