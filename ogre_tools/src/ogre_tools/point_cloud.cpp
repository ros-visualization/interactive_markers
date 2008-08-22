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

#define MAX_POINTS_PER_BBS (65535)
#define MAX_BILLBOARDS_PER_BBS (65535/4)

namespace ogre_tools
{

Ogre::String PointCloud::sm_Type = "PointCloud";

PointCloud::PointCloud( Ogre::SceneManager* sceneManager )
: scene_manager_( sceneManager )
, bounding_radius_( 0.0f )
, point_count_( 0 )
, points_per_bbs_( MAX_BILLBOARDS_PER_BBS )
, use_points_( false )
, billboard_width_( 0.003f )
, billboard_height_( 0.003f )
{
  static uint32_t pointCloudCount = 0;
  std::stringstream ss;
  ss << "PointCloud" << pointCloudCount++;

  scene_node_ = scene_manager_->getRootSceneNode()->createChildSceneNode();
  scene_node_->attachObject( this );



  Clear();
}

PointCloud::~PointCloud()
{
  V_BillboardSet::iterator bbsIt = billboard_sets_.begin();
  V_BillboardSet::iterator bbsEnd = billboard_sets_.end();
  for ( ; bbsIt != bbsEnd; ++bbsIt )
  {
    delete (*bbsIt);
  }
  billboard_sets_.clear();

  scene_manager_->destroySceneNode( scene_node_->getName() );
}

const Ogre::AxisAlignedBox& PointCloud::getBoundingBox() const
{
  return bounding_box_;
}

float PointCloud::getBoundingRadius() const
{
  return bounding_radius_;
}

void PointCloud::Clear()
{
  point_count_ = 0;
  bounding_box_.setExtents( -10000.0f, -10000.0f, -10000.0f, 10000.0f, 10000.0f, 10000.0f );
  bounding_radius_ = 30000.0f;
}

void PointCloud::SetUsePoints( bool usePoints )
{
  use_points_ = usePoints;

  if ( usePoints )
  {
    points_per_bbs_ = MAX_POINTS_PER_BBS;
  }
  else
  {
    points_per_bbs_ = MAX_BILLBOARDS_PER_BBS;
  }

  V_BillboardSet::iterator bbsIt = billboard_sets_.begin();
  V_BillboardSet::iterator bbsEnd = billboard_sets_.end();
  for ( ; bbsIt != bbsEnd; ++bbsIt )
  {
    Ogre::BillboardSet* bbs = *bbsIt;

    bbs->setPointRenderingEnabled( usePoints );
    bbs->setPoolSize( points_per_bbs_ );
  }
}

void PointCloud::SetBillboardDimensions( float width, float height )
{
  billboard_width_ = width;
  billboard_height_ = height;
  V_BillboardSet::iterator bbsIt = billboard_sets_.begin();
  V_BillboardSet::iterator bbsEnd = billboard_sets_.end();
  for ( ; bbsIt != bbsEnd; ++bbsIt )
  {
    Ogre::BillboardSet* bbs = *bbsIt;

    bbs->setDefaultDimensions( width, height );
  }
}

Ogre::BillboardSet* PointCloud::CreateBillboardSet()
{
  Ogre::BillboardSet* bbs = new Ogre::BillboardSet( "", 0, true );
  bbs->setPointRenderingEnabled( use_points_ );
  bbs->setDefaultDimensions( billboard_width_, billboard_height_ );
  bbs->setBillboardsInWorldSpace(true);
  bbs->setBillboardOrigin( Ogre::BBO_CENTER );
  bbs->setBillboardRotationType( Ogre::BBR_VERTEX );
  bbs->setMaterialName( "BaseWhiteNoLighting" );
  bbs->setCullIndividually( false );
  bbs->setPoolSize( points_per_bbs_ );

  return bbs;
}

void PointCloud::AddPoints( Point* points, uint32_t numPoints )
{
  if ( points_.size() < point_count_ + numPoints )
  {
    points_.resize( point_count_ + numPoints );
  }

  Point* begin = &points_.front() + point_count_;
  memcpy( begin, points, sizeof( Point ) * numPoints );

  // update bounding box and radius
  /*uint32_t totalPoints = point_count_ + numPoints;
  for ( uint32_t i = point_count_; i < totalPoints; ++i )
  {
    Point& p = points_[i];

    Ogre::Vector3 pos( p.m_X, p.m_Y, p.m_Z );
    bounding_box_.merge( pos );

    bounding_radius_ = std::max( bounding_radius_, pos.length() );
  }*/

  point_count_ += numPoints;
}

void PointCloud::_notifyCurrentCamera( Ogre::Camera* camera )
{
  MovableObject::_notifyCurrentCamera( camera );


  V_BillboardSet::iterator bbsIt = billboard_sets_.begin();
  V_BillboardSet::iterator bbsEnd = billboard_sets_.end();
  for ( ; bbsIt != bbsEnd; ++bbsIt )
  {
    (*bbsIt)->_notifyCurrentCamera( camera );
  }
}

void PointCloud::_updateRenderQueue( Ogre::RenderQueue* queue )
{
  if ( point_count_ == 0 )
  {
    return;
  }

    // Update billboard set geometry
  Ogre::Billboard bb;
  uint32_t pointsInCurrent = 0;
  uint32_t currentBBS = 0;
  Ogre::BillboardSet* bbs = NULL;
  V_BillboardSet used;
  for ( uint32_t i = 0; i < point_count_; ++i, ++pointsInCurrent )
  {
    bool newBBS = false;
    if ( pointsInCurrent > points_per_bbs_ )
    {
      bbs->endBillboards();

      pointsInCurrent = 0;
      ++currentBBS;

      newBBS = true;
    }

    if ( currentBBS >= billboard_sets_.size() )
    {
      bbs = CreateBillboardSet();
      billboard_sets_.push_back( bbs );

      newBBS = true;
    }

    if ( newBBS || !bbs )
    {
      bbs = billboard_sets_[ currentBBS ];
      bbs->beginBillboards( std::min<uint32_t>( point_count_ - i, points_per_bbs_ ) );

      used.push_back( bbs );
    }

    Point& p = points_[i];

    bb.mPosition.x = p.m_X;
    bb.mPosition.y = p.m_Y;
    bb.mPosition.z = p.m_Z;
    bb.mColour.r = p.r_;
    bb.mColour.g = p.g_;
    bb.mColour.b = p.b_;

    bbs->injectBillboard(bb);
  }

  bbs->endBillboards();

  // Update the queue
  V_BillboardSet::iterator bbsIt = used.begin();
  V_BillboardSet::iterator bbsEnd = used.end();
  for ( ; bbsIt != bbsEnd; ++bbsIt )
  {
    (*bbsIt)->_updateRenderQueue( queue );
  }
}

void PointCloud::SetVisible( bool visible )
{
  scene_node_->setVisible( visible );
}

} // namespace ogre_tools
