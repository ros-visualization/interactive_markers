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

#include "ros/node.h"
#include "ogre_tools/point_cloud.h"

#include <rosTF/rosTF.h>

#include <Ogre.h>

namespace ogre_vis
{

LaserScanVisualizer::LaserScanVisualizer( Ogre::SceneManager* sceneManager, ros::node* node, rosTFClient* tfClient, const std::string& name, bool enabled )
    : VisualizerBase( sceneManager, node, tfClient, name, enabled )
    , m_RegenerateCloud( false )
    , m_ClearNextFrame( false )
    , m_R( 1.0 )
    , m_G( 0.0 )
    , m_B( 0.0 )
    , m_IntensityMin( 999999.0f )
    , m_IntensityMax( -999999.0f )
{
  m_Cloud = new ogre_tools::PointCloud( m_SceneManager );

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

void LaserScanVisualizer::SetShutterTopic( const std::string& topic )
{
  Unsubscribe();

  m_ShutterTopic = topic;

  Subscribe();
}

void LaserScanVisualizer::SetColor( float r, float g, float b )
{
  m_R = r;
  m_G = g;
  m_B = b;

  m_RegenerateCloud = true;
}

void LaserScanVisualizer::OnEnable()
{
  m_Cloud->SetVisible( true );
  Subscribe();
}

void LaserScanVisualizer::OnDisable()
{
  m_Cloud->SetVisible( false );
  Unsubscribe();
}

void LaserScanVisualizer::Subscribe()
{
  if ( !IsEnabled() )
  {
    return;
  }

  if ( !m_CloudTopic.empty() )
  {
    m_ROSNode->subscribe( m_CloudTopic, m_CloudMessage, &LaserScanVisualizer::IncomingCloudCallback, this );
  }

  if ( !m_ScanTopic.empty() )
  {
    m_ROSNode->subscribe( m_ScanTopic, m_ScanMessage, &LaserScanVisualizer::IncomingScanCallback, this );
  }

  if ( !m_ShutterTopic.empty() )
  {
    m_ROSNode->subscribe( m_ShutterTopic, m_ShutterMessage, &LaserScanVisualizer::IncomingShutterCallback, this );
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

  if ( !m_ShutterTopic.empty() )
  {
    m_ROSNode->unsubscribe( m_ShutterTopic );
  }
}

void LaserScanVisualizer::Update( float dt )
{
  m_CloudMessage.lock();

  if ( m_RegenerateCloud )
  {
    if ( m_ClearNextFrame || m_ShutterTopic.empty() )
    {
      m_Cloud->Clear();

      m_ClearNextFrame = false;
    }

    if ( !m_NewPoints.empty() )
    {
      m_Cloud->AddPoints( &m_NewPoints.front(), m_NewPoints.size() );
    }

    m_RegenerateCloud = false;

    CauseRender();
  }

  m_CloudMessage.unlock();
}

void LaserScanVisualizer::TransformCloud()
{
  if ( m_CloudMessage.header.frame_id == 0 )
  {
    m_CloudMessage.header.frame_id = m_TFClient->lookup( "FRAMEID_BASE" );
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
  catch(libTF::Pose3DCache::ExtrapolateException& e)
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

  m_NewPoints.resize( pointCount );
  for(uint32_t i = 0; i < pointCount; i++)
  {
    Ogre::Vector3 point( m_CloudMessage.pts[i].x, m_CloudMessage.pts[i].y, m_CloudMessage.pts[i].z );
    RobotToOgre( point );

    float intensity = m_CloudMessage.chan[0].vals[i];

    float normalizedIntensity = (diffIntensity > 0.0f) ? ( intensity - m_IntensityMin ) / diffIntensity : 1.0f;

    Ogre::Vector3 color( m_R, m_G, m_B );
    color *= normalizedIntensity;

    ogre_tools::PointCloud::Point& currentPoint = m_NewPoints[ i ];
    currentPoint.m_X = point.x;
    currentPoint.m_Y = point.y;
    currentPoint.m_Z = point.z;
    currentPoint.m_R = color.x;
    currentPoint.m_G = color.y;
    currentPoint.m_B = color.z;
  }
}

void LaserScanVisualizer::IncomingCloudCallback()
{
  m_CloudMessage.lock();

  TransformCloud();

  m_RegenerateCloud = true;

  m_CloudMessage.unlock();
}

void LaserScanVisualizer::IncomingScanCallback()
{
  m_CloudMessage.lock();

  if ( m_ScanMessage.header.frame_id == 0 )
  {
    m_ScanMessage.header.frame_id = m_TFClient->lookup( "FRAMEID_BASE" );
  }

  m_LaserProjection.projectLaser( m_ScanMessage, m_CloudMessage );
  TransformCloud();

  m_CloudMessage.unlock();

  m_RegenerateCloud = true;
}

void LaserScanVisualizer::IncomingShutterCallback()
{
  m_ClearNextFrame = true;
}

} // namespace ogre_vis
