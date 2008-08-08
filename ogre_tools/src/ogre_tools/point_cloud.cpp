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

#define MAX_POINTS_PER_BBS (65535/4)

namespace ogre_tools
{

Ogre::String PointCloud::sm_Type = "PointCloud";

PointCloud::PointCloud( Ogre::SceneManager* sceneManager )
    : m_SceneManager( sceneManager )
    , m_BoundingRadius( 0.0f )
    , m_PointCount( 0 )
{
  static uint32_t pointCloudCount = 0;
  std::stringstream ss;
  ss << "PointCloud" << pointCloudCount++;

  m_SceneNode = m_SceneManager->getRootSceneNode()->createChildSceneNode();
  m_SceneNode->attachObject( this );



  Clear();
}

PointCloud::~PointCloud()
{
  V_BillboardSet::iterator bbsIt = m_BillboardSets.begin();
  V_BillboardSet::iterator bbsEnd = m_BillboardSets.end();
  for ( ; bbsIt != bbsEnd; ++bbsIt )
  {
    delete (*bbsIt);
  }
  m_BillboardSets.clear();

  m_SceneManager->destroySceneNode( m_SceneNode->getName() );
}

const Ogre::AxisAlignedBox& PointCloud::getBoundingBox() const
{
  return m_BoundingBox;
}

float PointCloud::getBoundingRadius() const
{
  return m_BoundingRadius;
}

void PointCloud::Clear()
{
  m_PointCount = 0;
  m_BoundingBox.setExtents( -10000.0f, -10000.0f, -10000.0f, 10000.0f, 10000.0f, 10000.0f );
  m_BoundingRadius = 30000.0f;
}

Ogre::BillboardSet* PointCloud::CreateBillboardSet()
{
  Ogre::BillboardSet* bbs = new Ogre::BillboardSet( "", 0, true );
  bbs->setPointRenderingEnabled( false );
  bbs->setDefaultDimensions( 0.003f, 0.003f );
  bbs->setBillboardsInWorldSpace(true);
  bbs->setBillboardOrigin( Ogre::BBO_CENTER );
  bbs->setBillboardRotationType( Ogre::BBR_VERTEX );
  bbs->setMaterialName( "BaseWhiteNoLighting" );
  bbs->setCullIndividually( false );
  bbs->setPoolSize( MAX_POINTS_PER_BBS );

  return bbs;
}

void PointCloud::AddPoints( Point* points, uint32_t numPoints )
{
  if ( m_Points.size() < m_PointCount + numPoints )
  {
    m_Points.resize( m_PointCount + numPoints );
  }

  Point* begin = &m_Points.front() + m_PointCount;
  memcpy( begin, points, sizeof( Point ) * numPoints );

  // update bounding box and radius
  /*uint32_t totalPoints = m_PointCount + numPoints;
  for ( uint32_t i = m_PointCount; i < totalPoints; ++i )
  {
    Point& p = m_Points[i];

    Ogre::Vector3 pos( p.m_X, p.m_Y, p.m_Z );
    m_BoundingBox.merge( pos );

    m_BoundingRadius = std::max( m_BoundingRadius, pos.length() );
  }*/

  m_PointCount += numPoints;
}

void PointCloud::_notifyCurrentCamera( Ogre::Camera* camera )
{
  MovableObject::_notifyCurrentCamera( camera );


  V_BillboardSet::iterator bbsIt = m_BillboardSets.begin();
  V_BillboardSet::iterator bbsEnd = m_BillboardSets.end();
  for ( ; bbsIt != bbsEnd; ++bbsIt )
  {
    (*bbsIt)->_notifyCurrentCamera( camera );
  }
}

void PointCloud::_updateRenderQueue( Ogre::RenderQueue* queue )
{
  if ( m_PointCount == 0 )
  {
    return;
  }

    // Update billboard set geometry
  Ogre::Billboard bb;
  uint32_t pointsInCurrent = 0;
  uint32_t currentBBS = 0;
  Ogre::BillboardSet* bbs = NULL;
  for ( uint32_t i = 0; i < m_PointCount; ++i, ++pointsInCurrent )
  {
    bool newBBS = false;
    if ( pointsInCurrent > MAX_POINTS_PER_BBS )
    {
      bbs->endBillboards();

      pointsInCurrent = 0;
      ++currentBBS;

      newBBS = true;
    }

    if ( currentBBS >= m_BillboardSets.size() )
    {
      bbs = CreateBillboardSet();
      m_BillboardSets.push_back( bbs );

      newBBS = true;
    }

    if ( newBBS || !bbs )
    {
      bbs = m_BillboardSets[ currentBBS ];
      bbs->beginBillboards( std::min<uint32_t>( m_PointCount - i, MAX_POINTS_PER_BBS ) );
    }

    Point& p = m_Points[i];

    bb.mPosition.x = p.m_X;
    bb.mPosition.y = p.m_Y;
    bb.mPosition.z = p.m_Z;
    bb.mColour.r = p.m_R;
    bb.mColour.g = p.m_G;
    bb.mColour.b = p.m_B;

    bbs->injectBillboard(bb);
  }

  bbs->endBillboards();

  // Update the queue
  V_BillboardSet::iterator bbsIt = m_BillboardSets.begin();
  V_BillboardSet::iterator bbsEnd = m_BillboardSets.end();
  for ( ; bbsIt != bbsEnd; ++bbsIt )
  {
    (*bbsIt)->_updateRenderQueue( queue );
  }
}

void PointCloud::SetVisible( bool visible )
{
  m_SceneNode->setVisible( visible );
}

} // namespace ogre_tools
