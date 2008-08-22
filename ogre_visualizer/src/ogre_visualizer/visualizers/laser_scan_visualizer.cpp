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
#include "../common.h"
#include "../ros_topic_property.h"

#include "ros/node.h"
#include "ogre_tools/point_cloud.h"

#include <rosTF/rosTF.h>

#include <Ogre.h>
#include <wx/wx.h>
#include <wx/propgrid/propgrid.h>
#include <wx/propgrid/advprops.h>

#define SCAN_TOPIC_PROPERTY wxT("Scan Topic")
#define CLOUD_TOPIC_PROPERTY wxT("Cloud Topic")
#define COLOR_PROPERTY wxT("Color")
#define DECAY_TIME_PROPERTY wxT("Decay Time")
#define STYLE_PROPERTY wxT("Style")
#define BILLBOARD_SIZE_PROPERTY wxT("Billboard Size")

namespace ogre_vis
{

LaserScanVisualizer::LaserScanVisualizer( Ogre::SceneManager* sceneManager, ros::node* node, rosTFClient* tfClient, const std::string& name, bool enabled )
: VisualizerBase( sceneManager, node, tfClient, name, enabled )
, m_R( 1.0 )
, m_G( 0.0 )
, m_B( 0.0 )
, m_IntensityMin( 999999.0f )
, m_IntensityMax( -999999.0f )
, m_PointDecayTime( 20.0f )
, m_Style( Billboards )
, m_BillboardSize( 0.003 )
{
  m_Cloud = new ogre_tools::PointCloud( m_SceneManager );

  SetStyle( m_Style );
  SetBillboardSize( m_BillboardSize );

  if ( IsEnabled() )
  {
    OnEnable();
  }
}

LaserScanVisualizer::~LaserScanVisualizer()
{
  Unsubscribe();

  delete m_Cloud;
}

void LaserScanVisualizer::SetCloudTopic( const std::string& topic )
{
  Unsubscribe();

  m_CloudTopic = topic;

  Subscribe();
}

void LaserScanVisualizer::SetScanTopic( const std::string& topic )
{
  Unsubscribe();

  m_ScanTopic = topic;

  Subscribe();
}

void LaserScanVisualizer::SetColor( float r, float g, float b )
{
  m_R = r;
  m_G = g;
  m_B = b;
}

void LaserScanVisualizer::SetStyle( Style style )
{
  {
    RenderAutoLock renderLock( this );

    m_Style = style;
    m_Cloud->SetUsePoints( style == Points );
  }

  CauseRender();
}

void LaserScanVisualizer::SetBillboardSize( float size )
{
  {
    RenderAutoLock renderLock( this );

    m_BillboardSize = size;
    m_Cloud->SetBillboardDimensions( size, size );
  }

  CauseRender();
}

void LaserScanVisualizer::OnEnable()
{
  m_Cloud->SetVisible( true );
  Subscribe();
}

void LaserScanVisualizer::OnDisable()
{
  Unsubscribe();

  m_Cloud->SetVisible( false );
  m_Cloud->Clear();
  m_Points.clear();
  m_PointTimes.clear();
}

void LaserScanVisualizer::Subscribe()
{
  if ( !IsEnabled() )
  {
    return;
  }

  if ( !m_CloudTopic.empty() )
  {
    m_ROSNode->subscribe( m_CloudTopic, m_CloudMessage, &LaserScanVisualizer::IncomingCloudCallback, this, 1 );
  }

  if ( !m_ScanTopic.empty() )
  {
    m_ROSNode->subscribe( m_ScanTopic, m_ScanMessage, &LaserScanVisualizer::IncomingScanCallback, this, 1 );
  }
}

void LaserScanVisualizer::Unsubscribe()
{
  if ( !m_CloudTopic.empty() )
  {
    m_ROSNode->unsubscribe( m_CloudTopic );
  }

  if ( !m_ScanTopic.empty() )
  {
    m_ROSNode->unsubscribe( m_ScanTopic );
  }
}

void LaserScanVisualizer::Update( float dt )
{
  m_CloudMessage.lock();

  D_float::iterator it = m_PointTimes.begin();
  D_float::iterator end = m_PointTimes.end();
  for ( ; it != end; ++it )
  {
    *it += dt;
  }

  CullPoints();

  m_CloudMessage.unlock();
}

void LaserScanVisualizer::CullPoints()
{
  if ( m_PointDecayTime == 0.0f )
  {
    return;
  }

  while ( !m_PointTimes.empty() && m_PointTimes.front() > m_PointDecayTime )
  {
    m_PointTimes.pop_front();
    m_Points.pop_front();
  }
}

void LaserScanVisualizer::TransformCloud()
{
  if ( m_CloudMessage.header.frame_id.empty() )
  {
    m_CloudMessage.header.frame_id = m_TargetFrame;
  }

  try
  {
    m_TFClient->transformPointCloud(m_TargetFrame, m_CloudMessage, m_CloudMessage);
  }
  catch(libTF::TransformReference::LookupException& e)
  {
    printf( "Error transforming laser scan '%s': %s\n", m_Name.c_str(), e.what() );
  }
  catch(libTF::TransformReference::ConnectivityException& e)
  {
    printf( "Error transforming laser scan '%s': %s\n", m_Name.c_str(), e.what() );
  }
  catch(libTF::TransformReference::ExtrapolateException& e)
  {
    printf( "Error transforming laser scan '%s': %s\n", m_Name.c_str(), e.what() );
  }

  uint32_t pointCount = m_CloudMessage.get_pts_size();
  for(uint32_t i = 0; i < pointCount; i++)
  {
    float& intensity = m_CloudMessage.chan[0].vals[i];
    // arbitrarily cap to 4096 for now
    intensity = std::min( intensity, 4096.0f );
    m_IntensityMin = std::min( m_IntensityMin, intensity );
    m_IntensityMax = std::max( m_IntensityMax, intensity );
  }

  float diffIntensity = m_IntensityMax - m_IntensityMin;

  if ( m_PointDecayTime == 0.0f )
  {
    m_Points.clear();
    m_PointTimes.clear();
  }

  m_Points.push_back( V_Point() );
  V_Point& points = m_Points.back();
  points.resize( pointCount );

  m_PointTimes.push_back( 0.0f );
  for(uint32_t i = 0; i < pointCount; i++)
  {
    Ogre::Vector3 point( m_CloudMessage.pts[i].x, m_CloudMessage.pts[i].y, m_CloudMessage.pts[i].z );
    RobotToOgre( point );

    float intensity = m_CloudMessage.chan[0].vals[i];

    float normalizedIntensity = (diffIntensity > 0.0f) ? ( intensity - m_IntensityMin ) / diffIntensity : 1.0f;

    Ogre::Vector3 color( m_R, m_G, m_B );
    color *= normalizedIntensity;

    ogre_tools::PointCloud::Point& currentPoint = points[ i ];
    currentPoint.m_X = point.x;
    currentPoint.m_Y = point.y;
    currentPoint.m_Z = point.z;
    currentPoint.m_R = color.x;
    currentPoint.m_G = color.y;
    currentPoint.m_B = color.z;
  }

  {
    RenderAutoLock renderLock( this );

    m_Cloud->Clear();

    if ( !m_Points.empty() )
    {
      DV_Point::iterator it = m_Points.begin();
      DV_Point::iterator end = m_Points.end();
      for ( ; it != end; ++it )
      {
        V_Point& points = *it;

        if ( !points.empty() )
        {
          m_Cloud->AddPoints( &points.front(), points.size() );
        }
      }
    }
  }

  CauseRender();
}

void LaserScanVisualizer::IncomingCloudCallback()
{
  m_CloudMessage.lock();

  TransformCloud();

  m_CloudMessage.unlock();
}

void LaserScanVisualizer::IncomingScanCallback()
{
  m_CloudMessage.lock();

  if ( m_ScanMessage.header.frame_id.empty() )
  {
    m_ScanMessage.header.frame_id = m_TargetFrame;
  }

  m_LaserProjection.projectLaser( m_ScanMessage, m_CloudMessage );
  TransformCloud();

  m_CloudMessage.unlock();
}

void LaserScanVisualizer::FillPropertyGrid( wxPropertyGrid* propertyGrid )
{
  wxArrayString styleNames;
  styleNames.Add( wxT("Billboards") );
  styleNames.Add( wxT("Points") );
  wxArrayInt styleIds;
  styleIds.Add( Billboards );
  styleIds.Add( Points );

  propertyGrid->Append( new wxEnumProperty( STYLE_PROPERTY, wxPG_LABEL, styleNames, styleIds, m_Style ) );

  propertyGrid->Append( new ROSTopicProperty( m_ROSNode, SCAN_TOPIC_PROPERTY, wxPG_LABEL, wxString::FromAscii( m_ScanTopic.c_str() ) ) );
  propertyGrid->Append( new ROSTopicProperty( m_ROSNode, CLOUD_TOPIC_PROPERTY, wxPG_LABEL, wxString::FromAscii( m_CloudTopic.c_str() ) ) );
  propertyGrid->Append( new wxColourProperty( COLOR_PROPERTY, wxPG_LABEL, wxColour( m_R * 255, m_G * 255, m_B * 255 ) ) );
  wxPGId prop = propertyGrid->Append( new wxFloatProperty( DECAY_TIME_PROPERTY, wxPG_LABEL, m_PointDecayTime ) );

  propertyGrid->SetPropertyAttribute( prop, wxT("Min"), 0.0 );

  prop = propertyGrid->Append( new wxFloatProperty( BILLBOARD_SIZE_PROPERTY, wxPG_LABEL, m_BillboardSize ) );
  propertyGrid->SetPropertyAttribute( prop, wxT("Min"), 0.0 );
}

void LaserScanVisualizer::PropertyChanged( wxPropertyGridEvent& event )
{
  wxPGProperty* property = event.GetProperty();

  const wxString& name = property->GetName();
  wxVariant value = property->GetValue();

  if ( name == SCAN_TOPIC_PROPERTY )
  {
    wxString topic = value.GetString();
    SetScanTopic( std::string(topic.char_str()) );
  }
  else if ( name == CLOUD_TOPIC_PROPERTY )
  {
    wxString topic = value.GetString();
    SetCloudTopic( std::string(topic.char_str()) );
  }
  else if ( name == COLOR_PROPERTY )
  {
    wxColour color;
    color << value;

    SetColor( color.Red() / 255.0f, color.Green() / 255.0f, color.Blue() / 255.0f );
  }
  else if ( name == DECAY_TIME_PROPERTY )
  {
    float val = value.GetDouble();
    SetDecayTime( val );
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
