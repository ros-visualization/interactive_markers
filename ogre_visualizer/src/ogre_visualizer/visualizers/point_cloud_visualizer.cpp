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

PointCloudVisualizer::PointCloudVisualizer( Ogre::SceneManager* sceneManager, ros::node* node, rosTFClient* tfClient, const std::string& name, bool enabled )
: VisualizerBase( sceneManager, node, tfClient, name, enabled )
, r_( 1.0 )
, g_( 1.0 )
, b_( 1.0 )
, style_( Billboards )
, billboard_size_( 0.003 )
{
  cloud_ = new ogre_tools::PointCloud( scene_manager_ );

  SetStyle( style_ );
  SetBillboardSize( billboard_size_ );

  if ( IsEnabled() )
  {
    OnEnable();
  }
}

PointCloudVisualizer::~PointCloudVisualizer()
{
  Unsubscribe();

  delete cloud_;
}

void PointCloudVisualizer::SetTopic( const std::string& topic )
{
  Unsubscribe();

  topic_ = topic;

  Subscribe();
}

void PointCloudVisualizer::SetColor( float r, float g, float b )
{
  r_ = r;
  g_ = g;
  b_ = b;
}

void PointCloudVisualizer::SetStyle( Style style )
{
  {
    RenderAutoLock renderLock( this );

    style_ = style;
    cloud_->SetUsePoints( style == Points );
  }

  CauseRender();
}

void PointCloudVisualizer::SetBillboardSize( float size )
{
  {
    RenderAutoLock renderLock( this );

    billboard_size_ = size;
    cloud_->SetBillboardDimensions( size, size );
  }

  CauseRender();
}

void PointCloudVisualizer::OnEnable()
{
  cloud_->SetVisible( true );
  Subscribe();
}

void PointCloudVisualizer::OnDisable()
{
  Unsubscribe();

  cloud_->Clear();
  cloud_->SetVisible( false );
  points_.clear();
}

void PointCloudVisualizer::Subscribe()
{
  if ( !IsEnabled() )
  {
    return;
  }

  if ( !topic_.empty() )
  {
    ros_node_->subscribe( topic_, message_, &PointCloudVisualizer::IncomingCloudCallback, this, 1 );
  }
}

void PointCloudVisualizer::Unsubscribe()
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

void PointCloudVisualizer::IncomingCloudCallback()
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

  points_.clear();

  // First find the min/max intensity values
  float minIntensity = 999999.0f;
  float maxIntensity = -999999.0f;

  uint32_t pointCount = message_.get_pts_size();
  for(uint32_t i = 0; i < pointCount; i++)
  {
    float& intensity = message_.chan[0].vals[i];
    // arbitrarily cap to 4096 for now
    intensity = std::min( intensity, 4096.0f );
    minIntensity = std::min( minIntensity, intensity );
    maxIntensity = std::max( maxIntensity, intensity );
  }

  float diffIntensity = maxIntensity - minIntensity;

  points_.resize( pointCount );
  for(uint32_t i = 0; i < pointCount; i++)
  {
    Ogre::Vector3 point( message_.pts[i].x, message_.pts[i].y, message_.pts[i].z );
    RobotToOgre( point );

    float intensity = message_.chan[0].vals[i];

    float normalizedIntensity = diffIntensity > 0.0f ? ( intensity - minIntensity ) / diffIntensity : 1.0f;

    Ogre::Vector3 color( r_, g_, b_ );
    color *= normalizedIntensity;

    ogre_tools::PointCloud::Point& currentPoint = points_[ i ];
    currentPoint.m_X = point.x;
    currentPoint.m_Y = point.y;
    currentPoint.m_Z = point.z;
    currentPoint.r_ = color.x;
    currentPoint.g_ = color.y;
    currentPoint.b_ = color.z;
  }

  {
    RenderAutoLock renderLock( this );

    cloud_->Clear();

    if ( !points_.empty() )
    {
      cloud_->AddPoints( &points_.front(), points_.size() );
    }
  }

  CauseRender();
}

void PointCloudVisualizer::FillPropertyGrid( wxPropertyGrid* propertyGrid )
{
  wxArrayString styleNames;
  styleNames.Add( wxT("Billboards") );
  styleNames.Add( wxT("Points") );
  wxArrayInt styleIds;
  styleIds.Add( Billboards );
  styleIds.Add( Points );

  propertyGrid->Append( new wxEnumProperty( STYLE_PROPERTY, wxPG_LABEL, styleNames, styleIds, style_ ) );

  propertyGrid->Append( new ROSTopicProperty( ros_node_, TOPIC_PROPERTY, wxPG_LABEL, wxString::FromAscii( topic_.c_str() ) ) );
  propertyGrid->Append( new wxColourProperty( COLOR_PROPERTY, wxPG_LABEL, wxColour( r_ * 255, g_ * 255, b_ * 255 ) ) );
  wxPGId prop = propertyGrid->Append( new wxFloatProperty( BILLBOARD_SIZE_PROPERTY, wxPG_LABEL, billboard_size_ ) );
  propertyGrid->SetPropertyAttribute( prop, wxT("Min"), 0.0 );
}

void PointCloudVisualizer::PropertyChanged( wxPropertyGridEvent& event )
{
  wxPGProperty* property = event.GetProperty();

  const wxString& name = property->GetName();
  wxVariant value = property->GetValue();

  if ( name == TOPIC_PROPERTY )
  {
    wxString topic = value.GetString();
    SetTopic( std::string(topic.char_str()) );
  }
  else if ( name == COLOR_PROPERTY )
  {
    wxColour color;
    color << value;

    SetColor( color.Red() / 255.0f, color.Green() / 255.0f, color.Blue() / 255.0f );
  }
  else if ( name == STYLE_PROPERTY )
  {
    int val = value.GetLong();
    SetStyle( (Style)val );
  }
  else if ( name == BILLBOARD_SIZE_PROPERTY )
  {
    float val = value.GetDouble();
    SetBillboardSize( val );
  }
}

} // namespace ogre_vis
