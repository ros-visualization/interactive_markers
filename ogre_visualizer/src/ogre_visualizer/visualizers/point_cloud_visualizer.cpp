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
#include "../common.h"
#include "../ros_topic_property.h"

#include "ros/node.h"
#include "ogre_tools/point_cloud.h"

#include <rosTF/rosTF.h>

#include <Ogre.h>
#include <wx/wx.h>
#include <wx/propgrid/propgrid.h>
#include <wx/propgrid/advprops.h>

#define TOPIC_PROPERTY wxT("Topic")
#define COLOR_PROPERTY wxT("Color")
#define STYLE_PROPERTY wxT("Style")
#define BILLBOARD_SIZE_PROPERTY wxT("Billboard Size")

namespace ogre_vis
{

PointCloudVisualizer::PointCloudVisualizer( Ogre::SceneManager* scene_manager, ros::node* node, rosTFClient* tf_client, const std::string& name, bool enabled )
: VisualizerBase( scene_manager, node, tf_client, name, enabled )
, r_( 1.0 )
, g_( 1.0 )
, b_( 1.0 )
, style_( Billboards )
, billboard_size_( 0.003 )
{
  cloud_ = new ogre_tools::PointCloud( scene_manager_ );

  setStyle( style_ );
  setBillboardSize( billboard_size_ );

  if ( isEnabled() )
  {
    onEnable();
  }
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
}

void PointCloudVisualizer::setColor( float r, float g, float b )
{
  r_ = r;
  g_ = g;
  b_ = b;
}

void PointCloudVisualizer::setStyle( Style style )
{
  {
    RenderAutoLock renderLock( this );

    style_ = style;
    cloud_->setUsePoints( style == Points );
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
}

void PointCloudVisualizer::onEnable()
{
  cloud_->setVisible( true );
  subscribe();
}

void PointCloudVisualizer::onDisable()
{
  unsubscribe();

  cloud_->clear();
  cloud_->setVisible( false );
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
    ros_node_->unsubscribe( topic_ );

    // block if our callback is still running
    message_.lock();
    message_.unlock();

    // ugh -- race condition, so sleep for a bit
    usleep( 100000 );
  }
}

void PointCloudVisualizer::incomingCloudCallback()
{
  if ( message_.header.frame_id.empty() )
  {
    message_.header.frame_id = target_frame_;
  }

  try
  {
    tf_client_->transformPointCloud(target_frame_, message_, message_);
  }
  catch(libTF::TransformReference::LookupException& e)
  {
    printf( "Error transforming point cloud '%s': %s\n", name_.c_str(), e.what() );
  }
  catch(libTF::TransformReference::ConnectivityException& e)
  {
    printf( "Error transforming point cloud '%s': %s\n", name_.c_str(), e.what() );
  }
  catch(libTF::TransformReference::ExtrapolateException& e)
  {
    printf( "Error transforming point cloud '%s': %s\n", name_.c_str(), e.what() );
  }

  // First find the min/max intensity values
  float min_intensity = 999999.0f;
  float max_intensity = -999999.0f;

  uint32_t pointCount = message_.get_pts_size();
  for(uint32_t i = 0; i < pointCount; i++)
  {
    float& intensity = message_.chan[0].vals[i];
    // arbitrarily cap to 4096 for now
    intensity = std::min( intensity, 4096.0f );
    min_intensity = std::min( min_intensity, intensity );
    max_intensity = std::max( max_intensity, intensity );
  }

  float diff_intensity = max_intensity - min_intensity;

  typedef std::vector< ogre_tools::PointCloud::Point > V_Point;
  V_Point points;
  points.resize( pointCount );
  for(uint32_t i = 0; i < pointCount; i++)
  {
    Ogre::Vector3 point( message_.pts[i].x, message_.pts[i].y, message_.pts[i].z );
    robotToOgre( point );

    float intensity = message_.chan[0].vals[i];

    float normalized_intensity = diff_intensity > 0.0f ? ( intensity - min_intensity ) / diff_intensity : 1.0f;

    Ogre::Vector3 color( r_, g_, b_ );
    color *= normalized_intensity;

    ogre_tools::PointCloud::Point& current_point = points[ i ];
    current_point.x_ = point.x;
    current_point.y_ = point.y;
    current_point.z_ = point.z;
    current_point.r_ = color.x;
    current_point.g_ = color.y;
    current_point.b_ = color.z;
  }

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

void PointCloudVisualizer::fillPropertyGrid( wxPropertyGrid* property_grid )
{
  wxArrayString style_names;
  style_names.Add( wxT("Billboards") );
  style_names.Add( wxT("Points") );
  wxArrayInt style_ids;
  style_ids.Add( Billboards );
  style_ids.Add( Points );

  property_grid->Append( new wxEnumProperty( STYLE_PROPERTY, wxPG_LABEL, style_names, style_ids, style_ ) );

  property_grid->Append( new ROSTopicProperty( ros_node_, TOPIC_PROPERTY, wxPG_LABEL, wxString::FromAscii( topic_.c_str() ) ) );
  property_grid->Append( new wxColourProperty( COLOR_PROPERTY, wxPG_LABEL, wxColour( r_ * 255, g_ * 255, b_ * 255 ) ) );
  wxPGId prop = property_grid->Append( new wxFloatProperty( BILLBOARD_SIZE_PROPERTY, wxPG_LABEL, billboard_size_ ) );
  property_grid->SetPropertyAttribute( prop, wxT("Min"), 0.0 );
}

void PointCloudVisualizer::propertyChanged( wxPropertyGridEvent& event )
{
  wxPGProperty* property = event.GetProperty();

  const wxString& name = property->GetName();
  wxVariant value = property->GetValue();

  if ( name == TOPIC_PROPERTY )
  {
    wxString topic = value.GetString();
    setTopic( std::string(topic.fn_str()) );
  }
  else if ( name == COLOR_PROPERTY )
  {
    wxColour color;
    color << value;

    setColor( color.Red() / 255.0f, color.Green() / 255.0f, color.Blue() / 255.0f );
  }
  else if ( name == STYLE_PROPERTY )
  {
    int val = value.GetLong();
    setStyle( (Style)val );
  }
  else if ( name == BILLBOARD_SIZE_PROPERTY )
  {
    float val = value.GetDouble();
    setBillboardSize( val );
  }
}

} // namespace ogre_vis
