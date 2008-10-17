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

#include "point_cloud_visualizer.h"
#include "common.h"
#include "ros_topic_property.h"
#include "properties/property.h"
#include "properties/property_manager.h"

#include "ros/node.h"
#include <ros/time.h>
#include "ogre_tools/point_cloud.h"

#include <rosTF/rosTF.h>

#include <Ogre.h>
#include <wx/wx.h>
#include <wx/propgrid/propgrid.h>
#include <wx/propgrid/advprops.h>
#include <wx/confbase.h>

namespace ogre_vis
{

PointCloudVisualizer::PointCloudVisualizer( const std::string& name, VisualizationManager* manager )
: VisualizerBase( name, manager )
, color_( 1.0f, 1.0f, 1.0f )
, style_( Billboards )
, billboard_size_( 0.003 )
, topic_property_( NULL )
, billboard_size_property_( NULL )
, color_property_( NULL )
, style_property_( NULL )
{
  scene_node_ = scene_manager_->getRootSceneNode()->createChildSceneNode();
  cloud_ = new ogre_tools::PointCloud( scene_manager_, scene_node_ );

  setStyle( style_ );
  setBillboardSize( billboard_size_ );
}

PointCloudVisualizer::~PointCloudVisualizer()
{
  unsubscribe();

  delete cloud_;
}

void PointCloudVisualizer::setTopic( const std::string& topic )
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

void PointCloudVisualizer::setColor( const Color& color )
{
  color_ = color;

  if ( color_property_ )
  {
    color_property_->changed();
  }

  causeRender();
}

void PointCloudVisualizer::setStyle( int style )
{
  ROS_ASSERT( style < StyleCount );

  {
    RenderAutoLock renderLock( this );

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

void PointCloudVisualizer::setBillboardSize( float size )
{
  {
    RenderAutoLock renderLock( this );

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

void PointCloudVisualizer::onEnable()
{
  cloud_->setCloudVisible( true );
  subscribe();
}

void PointCloudVisualizer::onDisable()
{
  unsubscribe();

  cloud_->clear();
  cloud_->setCloudVisible( false );
}

void PointCloudVisualizer::subscribe()
{
  if ( !isEnabled() )
  {
    return;
  }

  if ( !topic_.empty() )
  {
    ros_node_->subscribe( topic_, message_, &PointCloudVisualizer::incomingCloudCallback, this, 1 );
  }
}

void PointCloudVisualizer::unsubscribe()
{
  if ( !topic_.empty() )
  {
    ros_node_->unsubscribe( topic_, &PointCloudVisualizer::incomingCloudCallback, this );
  }
}

void PointCloudVisualizer::transformCloud()
{
  if ( message_.header.frame_id.empty() )
  {
    message_.header.frame_id = target_frame_;
  }

  libTF::TFPose pose = { 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0, message_.header.frame_id };

  try
  {
    pose = tf_client_->transformPose( target_frame_, pose );
  }
  catch(libTF::Exception& e)
  {
    ROS_ERROR( "Error transforming point cloud '%s' from frame '%s' to frame '%s'\n", name_.c_str(), message_.header.frame_id.c_str(), target_frame_.c_str() );
  }

  Ogre::Vector3 position( pose.x, pose.y, pose.z );
  robotToOgre( position );

  Ogre::Matrix3 orientation( ogreMatrixFromRobotEulers( pose.yaw, pose.pitch, pose.roll ) );
  scene_node_->setPosition( position );
  scene_node_->setOrientation( orientation );

  bool has_channel_0 = message_.get_chan_size() > 0;
  bool channel_is_rgb = has_channel_0 ? message_.chan[0].name == "rgb" : false;

  // First find the min/max intensity values
  float min_intensity = 999999.0f;
  float max_intensity = -999999.0f;

  uint32_t pointCount = message_.get_pts_size();
  if ( has_channel_0 && !channel_is_rgb )
  {
    for(uint32_t i = 0; i < pointCount; i++)
    {
      float& intensity = message_.chan[0].vals[i];
      // arbitrarily cap to 4096 for now
      intensity = std::min( intensity, 4096.0f );
      min_intensity = std::min( min_intensity, intensity );
      max_intensity = std::max( max_intensity, intensity );
    }
  }

  float diff_intensity = max_intensity - min_intensity;

  ros::Time start_vec = ros::Time::now();
  typedef std::vector< ogre_tools::PointCloud::Point > V_Point;
  V_Point points;
  points.resize( pointCount );
  for(uint32_t i = 0; i < pointCount; i++)
  {
    float channel = has_channel_0 ? message_.chan[0].vals[i] : 1.0f;

    Ogre::Vector3 color( color_.r_, color_.g_, color_.b_ );

    if ( channel_is_rgb )
    {
      int rgb = *(int*)&channel;
      float r = ((rgb >> 16) & 0xff) / 255.0f;
      float g = ((rgb >> 8) & 0xff) / 255.0f;
      float b = (rgb & 0xff) / 255.0f;
      color = Ogre::Vector3( r, g, b );
    }
    else
    {
      float normalized_intensity = diff_intensity > 0.0f ? ( channel - min_intensity ) / diff_intensity : 1.0f;
      color *= normalized_intensity;
    }

    ogre_tools::PointCloud::Point& current_point = points[ i ];
    current_point.x_ = message_.pts[i].x;
    current_point.y_ = message_.pts[i].y;
    current_point.z_ = message_.pts[i].z;
    current_point.r_ = color.x;
    current_point.g_ = color.y;
    current_point.b_ = color.z;
  }

  ros::Time start_add = ros::Time::now();
  {
    RenderAutoLock renderLock( this );

    cloud_->clear();

    if ( !points.empty() )
    {
      cloud_->addPoints( &points.front(), points.size() );
    }
  }

  causeRender();

}

void PointCloudVisualizer::incomingCloudCallback()
{
  transformCloud();
}

void PointCloudVisualizer::targetFrameChanged()
{
  message_.lock();

  transformCloud();

  message_.unlock();
}

void PointCloudVisualizer::createProperties()
{
  style_property_ = property_manager_->createProperty<EnumProperty>( "Style", property_prefix_, boost::bind( &PointCloudVisualizer::getStyle, this ),
                                                                     boost::bind( &PointCloudVisualizer::setStyle, this, _1 ), parent_category_, this );
  style_property_->addOption( "Billboards", Billboards );
  style_property_->addOption( "Points", Points );

  color_property_ = property_manager_->createProperty<ColorProperty>( "Color", property_prefix_, boost::bind( &PointCloudVisualizer::getColor, this ),
                                                                        boost::bind( &PointCloudVisualizer::setColor, this, _1 ), parent_category_, this );

  billboard_size_property_ = property_manager_->createProperty<FloatProperty>( "Billboard Size", property_prefix_, boost::bind( &PointCloudVisualizer::getBillboardSize, this ),
                                                                                boost::bind( &PointCloudVisualizer::setBillboardSize, this, _1 ), parent_category_, this );
  billboard_size_property_->setMin( 0.0001 );

  topic_property_ = property_manager_->createProperty<ROSTopicStringProperty>( "Topic", property_prefix_, boost::bind( &PointCloudVisualizer::getTopic, this ),
                                                                              boost::bind( &PointCloudVisualizer::setTopic, this, _1 ), parent_category_, this );
}

} // namespace ogre_vis
