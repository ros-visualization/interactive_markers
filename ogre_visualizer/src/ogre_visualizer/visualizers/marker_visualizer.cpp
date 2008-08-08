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

MarkerVisualizer::MarkerVisualizer( Ogre::SceneManager* sceneManager, ros::node* node, rosTFClient* tfClient, const std::string& name, bool enabled )
: VisualizerBase( sceneManager, node, tfClient, name, enabled )
{
  m_SceneNode = m_SceneManager->getRootSceneNode()->createChildSceneNode();

  if ( IsEnabled() )
  {
    OnEnable();
  }
}

MarkerVisualizer::~MarkerVisualizer()
{
  Unsubscribe();
}

void MarkerVisualizer::ClearMarkers()
{
  M_IDToObject::iterator markerIt = m_Markers.begin();
  M_IDToObject::iterator markerEnd = m_Markers.end();
  for ( ; markerIt != markerEnd; ++markerIt )
  {
    delete markerIt->second;
  }
  m_Markers.clear();
}

void MarkerVisualizer::OnEnable()
{
  Subscribe();

  m_SceneNode->setVisible( true );
}

void MarkerVisualizer::OnDisable()
{
  Unsubscribe();

  ClearMarkers();

  m_SceneNode->setVisible( false );
}

void MarkerVisualizer::Subscribe()
{
  if ( !IsEnabled() )
  {
    return;
  }

  m_ROSNode->subscribe("visualizationMarker", m_CurrentMessage, &MarkerVisualizer::IncomingMarker,this);
}

void MarkerVisualizer::Unsubscribe()
{
  m_ROSNode->unsubscribe( "visualizationMarker" );
}

void MarkerVisualizer::IncomingMarker()
{
  m_MessageQueue.push_back( m_CurrentMessage );
}

void MarkerVisualizer::ProcessMessage( const std_msgs::VisualizationMarker& message )
{
  switch ( message.action )
  {
  case MarkerActions::Add:
    ProcessAdd( message );
    break;

  case MarkerActions::Modify:
    ProcessModify( message );
    break;

  case MarkerActions::Delete:
    ProcessDelete( message );
    break;

  default:
    printf( "Unknown marker action: %d\n", message.action );
  }
}

void MarkerVisualizer::ProcessAdd( const std_msgs::VisualizationMarker& message )
{
  {
    M_IDToObject::iterator it = m_Markers.find( message.id );
    if ( it != m_Markers.end() )
    {
      printf( "Marker with id %d already exists, replacing...\n", message.id );

      delete it->second;
      m_Markers.erase( it );
    }
  }

  ogre_tools::Object* object = NULL;

  switch ( message.type )
  {
  case MarkerTypes::Cube:
    {
      ogre_tools::SuperEllipsoid* cube = new ogre_tools::SuperEllipsoid( m_SceneManager, m_SceneNode );
      cube->Create( ogre_tools::SuperEllipsoid::Cube, 10, Ogre::Vector3( 1.0f, 1.0f, 1.0f ) );

      object = cube;
    }
    break;

  case MarkerTypes::Sphere:
    {
      ogre_tools::SuperEllipsoid* sphere = new ogre_tools::SuperEllipsoid( m_SceneManager, m_SceneNode );
      sphere->Create( ogre_tools::SuperEllipsoid::Sphere, 20, Ogre::Vector3( 1.0f, 1.0f, 1.0f ) );

      object = sphere;
    }
    break;

  case MarkerTypes::Arrow:
    {
      object = new ogre_tools::Arrow( m_SceneManager, m_SceneNode, 0.8, 0.5, 0.2, 1.0 );
    }
    break;

  default:
    printf( "Unknown marker type: %d\n", message.type );
  }

  if ( object )
  {
    m_Markers.insert( std::make_pair( message.id, object ) );

    SetCommonValues( message, object );

    CauseRender();
  }
}

void MarkerVisualizer::ProcessModify( const std_msgs::VisualizationMarker& message )
{
  M_IDToObject::iterator it = m_Markers.find( message.id );
  if ( it == m_Markers.end() )
  {
    printf( "Tried to modify marker with id %d that does not exist\n", message.id );
    return;
  }

  SetCommonValues( message, it->second );

  CauseRender();
}

void MarkerVisualizer::ProcessDelete( const std_msgs::VisualizationMarker& message )
{
  M_IDToObject::iterator it = m_Markers.find( message.id );
  if ( it != m_Markers.end() )
  {
    delete it->second;
    m_Markers.erase( it );
  }

  CauseRender();
}

void MarkerVisualizer::SetCommonValues( const std_msgs::VisualizationMarker& message, ogre_tools::Object* object )
{
  uint32_t frameId = 1;
  /*if ( !message.frame.empty() )
  {
    frameId = m_TFClient->lookup( message.frame );
  }*/

  libTF::TFPoint tfPoint;
  tfPoint.x = message.x;
  tfPoint.y = message.y;
  tfPoint.z = message.z;
  tfPoint.time = 0;
  tfPoint.frame = frameId;
  try
  {
    tfPoint = m_TFClient->transformPoint( m_TargetFrame, tfPoint );
  }
  catch(libTF::TransformReference::LookupException& e)
  {
    printf( "Error transforming marker '%d': %s\n", message.id, e.what() );
  }
  catch(libTF::TransformReference::ConnectivityException& e)
  {
    printf( "Error transforming marker '%d': %s\n", message.id, e.what() );
  }
  catch(libTF::Pose3DCache::ExtrapolateException& e)
  {
    printf( "Error transforming marker '%d': %s\n", message.id, e.what() );
  }

  libTF::TFEulerYPR tfEulers;
  tfEulers.yaw = message.yaw;
  tfEulers.pitch = message.pitch;
  tfEulers.roll = message.roll;
  tfEulers.time = 0;
  tfEulers.frame = frameId;
  try
  {
    tfEulers = m_TFClient->transformEulerYPR( m_TargetFrame, tfEulers );
  }
  catch(libTF::TransformReference::LookupException& e)
  {
    printf( "Error transforming marker '%d': %s\n", message.id, e.what() );
  }
  catch(libTF::TransformReference::ConnectivityException& e)
  {
    printf( "Error transforming marker '%d': %s\n", message.id, e.what() );
  }
  catch(libTF::Pose3DCache::ExtrapolateException& e)
  {
    printf( "Error transforming marker '%d': %s\n", message.id, e.what() );
  }

  Ogre::Vector3 position( tfPoint.x, tfPoint.y, tfPoint.z );
  RobotToOgre( position );

  Ogre::Matrix3 orientation;
  orientation.FromEulerAnglesYXZ( Ogre::Radian( tfEulers.yaw ), Ogre::Radian( tfEulers.pitch ), Ogre::Radian( tfEulers.roll ) );
  //Ogre::Matrix3 orientation( OgreMatrixFromRobotEulers( tfEulers.yaw, tfEulers.pitch, tfEulers.roll ) );
  Ogre::Vector3 scale( message.xScale, message.yScale, message.zScale );
  RobotToOgre( scale );

  scale.x = fabsf( scale.x );
  scale.y = fabsf( scale.y );
  scale.z = fabsf( scale.z );

  object->SetPosition( position );
  object->SetOrientation( orientation );
  object->SetScale( scale );
  object->SetColor( message.r / 255.0f, message.g / 255.0f, message.b / 255.0f );
}

void MarkerVisualizer::Update( float dt )
{
  m_CurrentMessage.lock();

  if ( !m_MessageQueue.empty() )
  {
    V_MarkerMessage::iterator messageIt = m_MessageQueue.begin();
    V_MarkerMessage::iterator messageEnd = m_MessageQueue.end();
    for ( ; messageIt != messageEnd; ++messageIt )
    {
      std_msgs::VisualizationMarker& marker = *messageIt;

      ProcessMessage( marker );
    }

    m_MessageQueue.clear();
  }

  m_CurrentMessage.unlock();
}

} // namespace ogre_vis
