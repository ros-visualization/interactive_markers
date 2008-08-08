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

#include "ros/node.h"
#include "ogre_tools/point_cloud.h"

#include <rosTF/rosTF.h>

#include <Ogre.h>

namespace ogre_vis
{

PointCloudVisualizer::PointCloudVisualizer( Ogre::SceneManager* sceneManager, ros::node* node, rosTFClient* tfClient, const std::string& name, bool enabled )
    : VisualizerBase( sceneManager, node, tfClient, name, enabled )
    , m_RegenerateCloud( false )
    , m_R( 1.0 )
    , m_G( 1.0 )
    , m_B( 1.0 )
{
  m_Cloud = new ogre_tools::PointCloud( m_SceneManager );

  if ( IsEnabled() )
  {
    OnEnable();
  }
}

PointCloudVisualizer::~PointCloudVisualizer()
{
  Unsubscribe();

  delete m_Cloud;
}

void PointCloudVisualizer::SetTopic( const std::string& topic )
{
  Unsubscribe();

  m_Topic = topic;

  Subscribe();
}

void PointCloudVisualizer::SetColor( float r, float g, float b )
{
  m_R = r;
  m_G = g;
  m_B = b;

  m_RegenerateCloud = true;
}

void PointCloudVisualizer::OnEnable()
{
  m_Cloud->SetVisible( true );
  Subscribe();
}

void PointCloudVisualizer::OnDisable()
{
  m_Cloud->SetVisible( false );
  Unsubscribe();
}

void PointCloudVisualizer::Subscribe()
{
  if ( !IsEnabled() )
  {
    return;
  }

  if ( !m_Topic.empty() )
  {
    m_ROSNode->subscribe( m_Topic, m_Message, &PointCloudVisualizer::IncomingCloudCallback, this );
  }
}

void PointCloudVisualizer::Unsubscribe()
{
  if ( !m_Topic.empty() )
  {
    m_ROSNode->unsubscribe( m_Topic );

    // block if our callback is still running
    m_Message.lock();
    m_Message.unlock();

    // ugh -- race condition, so sleep for a bit
    usleep( 100000 );
  }
}

void PointCloudVisualizer::Update( float dt )
{
  m_Message.lock();

  if ( m_RegenerateCloud )
  {
    m_Cloud->Clear();

    if ( !m_Points.empty() )
    {
      m_Cloud->AddPoints( &m_Points.front(), m_Points.size() );
    }

    m_RegenerateCloud = false;

    CauseRender();
  }

  m_Message.unlock();
}

void PointCloudVisualizer::IncomingCloudCallback()
{
  m_Message.lock();

  if ( m_Message.header.frame_id == 0 )
  {
    m_Message.header.frame_id = m_TFClient->lookup( m_TargetFrame );
  }

  try
  {
    m_TFClient->transformPointCloud(m_TargetFrame, m_Message, m_Message);
  }
  catch(libTF::TransformReference::LookupException& e)
  {
    printf( "Error transforming point cloud '%s': %s\n", m_Name.c_str(), e.what() );
  }
  catch(libTF::TransformReference::ConnectivityException& e)
  {
    printf( "Error transforming point cloud '%s': %s\n", m_Name.c_str(), e.what() );
  }
  catch(libTF::TransformReference::ExtrapolateException& e)
  {
    printf( "Error transforming point cloud '%s': %s\n", m_Name.c_str(), e.what() );
  }

  m_Points.clear();

  // First find the min/max intensity values
  float minIntensity = 999999.0f;
  float maxIntensity = -999999.0f;

  uint32_t pointCount = m_Message.get_pts_size();
  for(uint32_t i = 0; i < pointCount; i++)
  {
    float& intensity = m_Message.chan[0].vals[i];
    // arbitrarily cap to 4096 for now
    intensity = std::min( intensity, 4096.0f );
    minIntensity = std::min( minIntensity, intensity );
    maxIntensity = std::max( maxIntensity, intensity );
  }

  float diffIntensity = maxIntensity - minIntensity;

  m_Points.resize( pointCount );
  for(uint32_t i = 0; i < pointCount; i++)
  {
    Ogre::Vector3 point( m_Message.pts[i].x, m_Message.pts[i].y, m_Message.pts[i].z );
    RobotToOgre( point );

    float intensity = m_Message.chan[0].vals[i];

    float normalizedIntensity = diffIntensity > 0.0f ? ( intensity - minIntensity ) / diffIntensity : 1.0f;

    Ogre::Vector3 color( m_R, m_G, m_B );
    color *= normalizedIntensity;

    ogre_tools::PointCloud::Point& currentPoint = m_Points[ i ];
    currentPoint.m_X = point.x;
    currentPoint.m_Y = point.y;
    currentPoint.m_Z = point.z;
    currentPoint.m_R = color.x;
    currentPoint.m_G = color.y;
    currentPoint.m_B = color.z;
  }

  m_RegenerateCloud = true;

  m_Message.unlock();
}

} // namespace ogre_vis
