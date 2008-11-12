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

#include "point_cloud_display.h"
#include "common.h"
#include "ros_topic_property.h"
#include "properties/property.h"
#include "properties/property_manager.h"

#include "ros/node.h"
#include <ros/time.h>
#include "ogre_tools/point_cloud.h"

#include <tf/transform_listener.h>
#include <tf/message_notifier.h>

#include <OgreSceneNode.h>
#include <OgreSceneManager.h>

namespace ogre_vis
{

PointCloudDisplay::PointCloudDisplay( const std::string& name, VisualizationManager* manager )
: Display( name, manager )
, color_( 1.0f, 1.0f, 1.0f )
, style_( Billboards )
, billboard_size_( 0.01 )
, topic_property_( NULL )
, billboard_size_property_( NULL )
, color_property_( NULL )
, style_property_( NULL )
{
  scene_node_ = scene_manager_->getRootSceneNode()->createChildSceneNode();
  cloud_ = new ogre_tools::PointCloud( scene_manager_, scene_node_ );

  setStyle( style_ );
  setBillboardSize( billboard_size_ );

  notifier_ = new tf::MessageNotifier<std_msgs::PointCloud>(tf_, ros_node_, boost::bind(&PointCloudDisplay::incomingCloudCallback, this, _1), "", "", 1);
}

PointCloudDisplay::~PointCloudDisplay()
{
  unsubscribe();

  delete notifier_;
  delete cloud_;
}

void PointCloudDisplay::setTopic( const std::string& topic )
{
  topic_ = topic;

  if ( isEnabled() )
  {
    notifier_->setTopic( topic );
  }

  if ( topic_property_ )
  {
    topic_property_->changed();
  }

  causeRender();
}

void PointCloudDisplay::setColor( const Color& color )
{
  color_ = color;

  if ( color_property_ )
  {
    color_property_->changed();
  }

  causeRender();
}

void PointCloudDisplay::setStyle( int style )
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

void PointCloudDisplay::setBillboardSize( float size )
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

void PointCloudDisplay::onEnable()
{
  cloud_->setCloudVisible( true );
  subscribe();
}

void PointCloudDisplay::onDisable()
{
  unsubscribe();
  notifier_->clear();

  cloud_->clear();
  cloud_->setCloudVisible( false );
}

void PointCloudDisplay::subscribe()
{
  if ( !isEnabled() )
  {
    return;
  }

  notifier_->setTopic( topic_ );
}

void PointCloudDisplay::unsubscribe()
{
  notifier_->setTopic( "" );
}

void transformIntensity( float val, ogre_tools::PointCloud::Point& point, float min_intensity, float max_intensity, float diff_intensity )
{
  float normalized_intensity = diff_intensity > 0.0f ? ( val - min_intensity ) / diff_intensity : 1.0f;
  point.r_ *= normalized_intensity;
  point.g_ *= normalized_intensity;
  point.b_ *= normalized_intensity;
}

void transformRGB( float val, ogre_tools::PointCloud::Point& point, float, float, float )
{
  int rgb = *reinterpret_cast<int*>(&val);
  point.r_ = ((rgb >> 16) & 0xff) / 255.0f;
  point.g_ = ((rgb >> 8) & 0xff) / 255.0f;
  point.b_ = (rgb & 0xff) / 255.0f;
}

void transformR( float val, ogre_tools::PointCloud::Point& point, float, float, float )
{
  point.r_ = val;
}

void transformG( float val, ogre_tools::PointCloud::Point& point, float, float, float )
{
  point.g_ = val;
}

void transformB( float val, ogre_tools::PointCloud::Point& point, float, float, float )
{
  point.b_ = val;
}

void PointCloudDisplay::transformCloud(const boost::shared_ptr<std_msgs::PointCloud>& cloud)
{
  std::string frame_id = cloud->header.frame_id;
  if ( frame_id.empty() )
  {
    frame_id = fixed_frame_;
  }

  tf::Stamped<tf::Pose> pose( btTransform( btQuaternion( 0, 0, 0 ), btVector3( 0, 0, 0 ) ), cloud->header.stamp, frame_id );

  try
  {
    tf_->transformPose( fixed_frame_, pose, pose );
  }
  catch(tf::TransformException& e)
  {
    ROS_ERROR( "Error transforming point cloud '%s' from frame '%s' to frame '%s'\n", name_.c_str(), frame_id.c_str(), fixed_frame_.c_str() );
  }

  Ogre::Vector3 position( pose.getOrigin().x(), pose.getOrigin().y(), pose.getOrigin().z() );
  robotToOgre( position );

  btScalar yaw, pitch, roll;
  pose.getBasis().getEulerZYX( yaw, pitch, roll );

  Ogre::Matrix3 orientation( ogreMatrixFromRobotEulers( yaw, pitch, roll ) );

  // First find the min/max intensity values
  float min_intensity = 999999.0f;
  float max_intensity = -999999.0f;

  typedef std::vector<std_msgs::ChannelFloat32> V_Chan;
  typedef std::vector<bool> V_bool;

  V_bool valid_channels(cloud->chan.size());
  uint32_t point_count = cloud->get_pts_size();
  V_Chan::iterator chan_it = cloud->chan.begin();
  V_Chan::iterator chan_end = cloud->chan.end();
  uint32_t index = 0;
  for ( ; chan_it != chan_end; ++chan_it, ++index )
  {
    std_msgs::ChannelFloat32& chan = *chan_it;
    uint32_t val_count = chan.vals.size();
    bool channel_size_correct = val_count == point_count;
    ROS_ERROR_COND(!channel_size_correct, "Point cloud '%s' on topic '%s' has channel 0 with fewer values than points (%d values, %d points)", name_.c_str(), topic_.c_str(), val_count, point_count);

    valid_channels[index] = channel_size_correct;

    if ( channel_size_correct && ( chan.name.empty() || chan.name == "intensity" || chan.name == "intensities" ) )
    {
      for(uint32_t i = 0; i < point_count; i++)
      {
        float& intensity = chan.vals[i];
        // arbitrarily cap to 4096 for now
        intensity = std::min( intensity, 4096.0f );
        min_intensity = std::min( min_intensity, intensity );
        max_intensity = std::max( max_intensity, intensity );
      }
    }
  }

  float diff_intensity = max_intensity - min_intensity;

  typedef std::vector< ogre_tools::PointCloud::Point > V_Point;
  V_Point points;
  points.resize( point_count );
  for(uint32_t i = 0; i < point_count; i++)
  {
    Ogre::Vector3 color( color_.r_, color_.g_, color_.b_ );
    ogre_tools::PointCloud::Point& current_point = points[ i ];

    current_point.x_ = cloud->pts[i].x;
    current_point.y_ = cloud->pts[i].y;
    current_point.z_ = cloud->pts[i].z;
    current_point.r_ = color.x;
    current_point.g_ = color.y;
    current_point.b_ = color.z;
  }

  chan_it = cloud->chan.begin();
  index = 0;
  for ( ; chan_it != chan_end; ++chan_it, ++index )
  {
    if ( !valid_channels[index] )
    {
      continue;
    }

    std_msgs::ChannelFloat32& chan = *chan_it;
    enum ChannelType
    {
      CT_INTENSITY,
      CT_RGB,
      CT_R,
      CT_G,
      CT_B,

      CT_COUNT
    };

    ChannelType type = CT_INTENSITY;
    if ( chan.name == "rgb" )
    {
      type = CT_RGB;
    }
    else if ( chan.name == "r" )
    {
      type = CT_R;
    }
    else if ( chan.name == "g" )
    {
      type = CT_G;
    }
    else if ( chan.name == "b" )
    {
      type = CT_B;
    }

    typedef void (*TransformFunc)(float, ogre_tools::PointCloud::Point&, float, float, float);
    TransformFunc funcs[CT_COUNT] =
    {
      transformIntensity,
      transformRGB,
      transformR,
      transformG,
      transformB
    };

    for(uint32_t i = 0; i < point_count; i++)
    {
      ogre_tools::PointCloud::Point& current_point = points[ i ];
      funcs[type]( chan.vals[i], current_point, min_intensity, max_intensity, diff_intensity );
    }
  }

  {
    RenderAutoLock renderLock( this );

    scene_node_->setPosition( position );
    scene_node_->setOrientation( orientation );

    cloud_->clear();

    if ( !points.empty() )
    {
      cloud_->addPoints( &points.front(), points.size() );
    }
  }

  causeRender();

}

void PointCloudDisplay::incomingCloudCallback(const boost::shared_ptr<std_msgs::PointCloud>& cloud)
{
  transformCloud( cloud );
}

void PointCloudDisplay::targetFrameChanged()
{
  notifier_->setTargetFrame( target_frame_ );
}

void PointCloudDisplay::fixedFrameChanged()
{
  RenderAutoLock renderLock( this );

  cloud_->clear();
}

void PointCloudDisplay::createProperties()
{
  style_property_ = property_manager_->createProperty<EnumProperty>( "Style", property_prefix_, boost::bind( &PointCloudDisplay::getStyle, this ),
                                                                     boost::bind( &PointCloudDisplay::setStyle, this, _1 ), parent_category_, this );
  style_property_->addOption( "Billboards", Billboards );
  style_property_->addOption( "Points", Points );

  color_property_ = property_manager_->createProperty<ColorProperty>( "Color", property_prefix_, boost::bind( &PointCloudDisplay::getColor, this ),
                                                                        boost::bind( &PointCloudDisplay::setColor, this, _1 ), parent_category_, this );

  billboard_size_property_ = property_manager_->createProperty<FloatProperty>( "Billboard Size", property_prefix_, boost::bind( &PointCloudDisplay::getBillboardSize, this ),
                                                                                boost::bind( &PointCloudDisplay::setBillboardSize, this, _1 ), parent_category_, this );
  billboard_size_property_->setMin( 0.0001 );

  topic_property_ = property_manager_->createProperty<ROSTopicStringProperty>( "Topic", property_prefix_, boost::bind( &PointCloudDisplay::getTopic, this ),
                                                                              boost::bind( &PointCloudDisplay::setTopic, this, _1 ), parent_category_, this );
}

void PointCloudDisplay::reset()
{
  RenderAutoLock renderLock( this );

  cloud_->clear();
}

const char* PointCloudDisplay::getDescription()
{
  return "Displays a point cloud from a std_msgs::PointCloud message.  Each message received clears the previous points.  More efficient than LaserScanDisplay.";
}

} // namespace ogre_vis
