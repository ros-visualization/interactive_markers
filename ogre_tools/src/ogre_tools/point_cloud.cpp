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

#include "point_cloud.h"

#include <Ogre.h>

#include <sstream>

#define MAX_POINTS_PER_SECTION 65535

namespace ogre_tools
{

PointCloud::PointCloud( Ogre::SceneManager* sceneManager )
    : m_SceneManager( sceneManager )
{
  static uint32_t pointCloudCount = 0;
  std::stringstream ss;
  ss << "PointCloud" << pointCloudCount++;

  m_ManualObject = m_SceneManager->createManualObject( ss.str() );
  m_ManualObject->setDynamic( true );

  m_SceneNode = m_SceneManager->getRootSceneNode()->createChildSceneNode();
  m_SceneNode->attachObject( m_ManualObject );
}

PointCloud::~PointCloud()
{
  m_SceneManager->destroySceneNode( m_SceneNode->getName() );
  m_SceneManager->destroyManualObject( m_ManualObject );
}

void PointCloud::Clear()
{
  m_ManualObject->clear();
}

void PointCloud::AddPoints( Point* points, uint32_t numPoints )
{
  m_ManualObject->estimateVertexCount( numPoints );
  m_ManualObject->begin( "BaseWhiteNoLighting", Ogre::RenderOperation::OT_POINT_LIST );

  uint32_t pointsInSection = 0;

  Point* currentPoint = points;
  for ( uint32_t i = 0; i < numPoints; ++i, ++currentPoint )
  {
    if ( pointsInSection >= MAX_POINTS_PER_SECTION )
    {
      m_ManualObject->end();
      m_ManualObject->begin( "BaseWhiteNoLighting", Ogre::RenderOperation::OT_POINT_LIST );

      pointsInSection = 0;
    }

    m_ManualObject->position( currentPoint->m_X, currentPoint->m_Y, currentPoint->m_Z );
    m_ManualObject->colour( currentPoint->m_R, currentPoint->m_G, currentPoint->m_B );
    ++pointsInSection;
  }

  m_ManualObject->end();
}

void PointCloud::SetVisible( bool visible )
{
  m_SceneNode->setVisible( visible );
}

} // namespace ogre_tools
