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

#include <OgreSceneManager.h>
#include <OgreSceneNode.h>
#include <OgreVector3.h>
#include <OgreQuaternion.h>
#include <OgreManualObject.h>
#include <OgreMaterialManager.h>
#include <OgreBillboardSet.h>
#include <OgreBillboard.h>

#include <sstream>

#define MAX_POINTS_PER_BBS (65535)
#define MAX_BILLBOARDS_PER_BBS (65535/4)

namespace ogre_tools
{

Ogre::String PointCloud::sm_Type = "PointCloud";

PointCloud::PointCloud( Ogre::SceneManager* scene_manager, Ogre::SceneNode* parent )
: scene_manager_( scene_manager )
, bounding_radius_( 0.0f )
, point_count_( 0 )
, points_per_bbs_( MAX_BILLBOARDS_PER_BBS )
, use_points_( false )
, billboard_width_( 0.01f )
, billboard_height_( 0.01f )
, billboard_type_( Ogre::BBT_POINT )
, common_direction_( Ogre::Vector3::NEGATIVE_UNIT_Z )
, common_up_vector_( Ogre::Vector3::UNIT_Y )
{
  if ( parent )
  {
    scene_node_ = parent->createChildSceneNode();
  }
  else
  {
    scene_node_ = scene_manager_->getRootSceneNode()->createChildSceneNode();
  }

  scene_node_->attachObject( this );

  std::stringstream ss;
  static int count = 0;
  ss << "PointCloudMaterial" << count++;
  material_name_ = ss.str();
  Ogre::MaterialPtr material = Ogre::MaterialManager::getSingleton().create( material_name_, Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME );
  material->setReceiveShadows(false);
  material->getTechnique(0)->setLightingEnabled(false);
  material->setCullingMode( Ogre::CULL_NONE );

  setAlpha( 1.0f );

  clear();
}

PointCloud::~PointCloud()
{
  V_BillboardSet::iterator bbs_it = billboard_sets_.begin();
  V_BillboardSet::iterator bbs_end = billboard_sets_.end();
  for ( ; bbs_it != bbs_end; ++bbs_it )
  {
    delete (*bbs_it);
  }
  billboard_sets_.clear();

  scene_node_->detachObject(this);
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

void PointCloud::getWorldTransforms( Ogre::Matrix4* xform ) const
{
  *xform = _getParentNodeFullTransform();
}

void PointCloud::clear()
{
  point_count_ = 0;
  bounding_box_.setExtents( -10000.0f, -10000.0f, -10000.0f, 10000.0f, 10000.0f, 10000.0f );
  bounding_radius_ = 30000.0f;

  V_BillboardSet::iterator bbs_it = billboard_sets_.begin();
  V_BillboardSet::iterator bbs_end = billboard_sets_.end();
  for ( ; bbs_it != bbs_end; ++bbs_it )
  {
    (*bbs_it)->setVisible( false );
  }
}

void PointCloud::setCloudVisible( bool visible )
{
  scene_node_->setVisible( visible );
}

void PointCloud::setUsePoints( bool usePoints )
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

  V_BillboardSet::iterator bbs_it = billboard_sets_.begin();
  V_BillboardSet::iterator bbs_end = billboard_sets_.end();
  for ( ; bbs_it != bbs_end; ++bbs_it )
  {
    Ogre::BillboardSet* bbs = *bbs_it;

    bbs->setPointRenderingEnabled( usePoints );
    bbs->setPoolSize( points_per_bbs_ );
  }
}

void PointCloud::setBillboardDimensions( float width, float height )
{
  billboard_width_ = width;
  billboard_height_ = height;
  V_BillboardSet::iterator bbs_it = billboard_sets_.begin();
  V_BillboardSet::iterator bbs_end = billboard_sets_.end();
  for ( ; bbs_it != bbs_end; ++bbs_it )
  {
    Ogre::BillboardSet* bbs = *bbs_it;

    bbs->setDefaultDimensions( width, height );
  }
}

void PointCloud::setBillboardType( int type )
{
  billboard_type_ = type;

  V_BillboardSet::iterator bbs_it = billboard_sets_.begin();
  V_BillboardSet::iterator bbs_end = billboard_sets_.end();
  for ( ; bbs_it != bbs_end; ++bbs_it )
  {
    Ogre::BillboardSet* bbs = *bbs_it;

    bbs->setBillboardType( (Ogre::BillboardType)billboard_type_ );
  }
}

void PointCloud::setCommonDirection( const Ogre::Vector3& vec )
{
  common_direction_ = vec;

  V_BillboardSet::iterator bbs_it = billboard_sets_.begin();
  V_BillboardSet::iterator bbs_end = billboard_sets_.end();
  for ( ; bbs_it != bbs_end; ++bbs_it )
  {
    Ogre::BillboardSet* bbs = *bbs_it;

    bbs->setCommonDirection( vec );
  }
}

void PointCloud::setCommonUpVector( const Ogre::Vector3& vec )
{
  common_up_vector_ = vec;

  V_BillboardSet::iterator bbs_it = billboard_sets_.begin();
  V_BillboardSet::iterator bbs_end = billboard_sets_.end();
  for ( ; bbs_it != bbs_end; ++bbs_it )
  {
    Ogre::BillboardSet* bbs = *bbs_it;

    bbs->setCommonUpVector( vec );
  }
}

void PointCloud::setAlpha( float alpha )
{
  alpha_ = alpha;

  Ogre::MaterialPtr material = Ogre::MaterialManager::getSingleton().getByName( material_name_ );
  if ( alpha_ < 0.9998 )
  {
    material->setSceneBlending( Ogre::SBT_TRANSPARENT_ALPHA );
    material->setDepthWriteEnabled( false );
  }
  else
  {
    material->setSceneBlending( Ogre::SBT_REPLACE );
    material->setDepthWriteEnabled( true );
  }
}

Ogre::BillboardSet* PointCloud::createBillboardSet()
{
  static uint32_t count = 0;
  std::stringstream name;
  name << "ogre_tools::PointCloud BillboardSet" << count++;

  Ogre::BillboardSet* bbs = new Ogre::BillboardSet( name.str(), 0, true );
  bbs->setPointRenderingEnabled( use_points_ );
  bbs->setDefaultDimensions( billboard_width_, billboard_height_ );
  bbs->setBillboardsInWorldSpace(false);
  bbs->setBillboardOrigin( Ogre::BBO_CENTER );
  bbs->setBillboardRotationType( Ogre::BBR_VERTEX );
  bbs->setMaterialName( material_name_ );
  bbs->setCullIndividually( false );
  bbs->setPoolSize( points_per_bbs_ );
  bbs->setBillboardType( (Ogre::BillboardType)billboard_type_ );
  bbs->setCommonDirection( common_direction_ );
  bbs->setCommonUpVector( common_up_vector_ );

  scene_node_->attachObject( bbs );

  return bbs;
}

void PointCloud::addPoints( Point* points, uint32_t num_points )
{
  if ( points_.size() < point_count_ + num_points )
  {
    points_.resize( point_count_ + num_points );
  }

  Point* begin = &points_.front() + point_count_;
  memcpy( begin, points, sizeof( Point ) * num_points );

  // update bounding box and radius
  /*uint32_t totalPoints = point_count_ + num_points;
  for ( uint32_t i = point_count_; i < totalPoints; ++i )
  {
    Point& p = points_[i];

    Ogre::Vector3 pos( p.m_X, p.m_Y, p.m_Z );
    bounding_box_.merge( pos );

    bounding_radius_ = std::max( bounding_radius_, pos.length() );
  }*/

  point_count_ += num_points;
}

void PointCloud::_notifyCurrentCamera( Ogre::Camera* camera )
{
  MovableObject::_notifyCurrentCamera( camera );


  V_BillboardSet::iterator bbs_it = billboard_sets_.begin();
  V_BillboardSet::iterator bbs_end = billboard_sets_.end();
  for ( ; bbs_it != bbs_end; ++bbs_it )
  {
    (*bbs_it)->_notifyCurrentCamera( camera );
  }
}

void PointCloud::_updateRenderQueue( Ogre::RenderQueue* queue )
{
  if ( point_count_ == 0 )
  {
    V_BillboardSet::iterator bbs_it = billboard_sets_.begin();
    V_BillboardSet::iterator bbs_end = billboard_sets_.end();
    for ( ; bbs_it != bbs_end; ++bbs_it )
    {
      (*bbs_it)->setVisible( false );
    }
    return;
  }

    // Update billboard set geometry
  Ogre::Billboard bb;
  uint32_t points_in_current = 0;
  uint32_t current_bbs = 0;
  Ogre::BillboardSet* bbs = NULL;
  V_BillboardSet used;
  for ( uint32_t i = 0; i < point_count_; ++i, ++points_in_current )
  {
    bool new_bbs = false;
    if ( points_in_current > points_per_bbs_ )
    {
      bbs->endBillboards();

      points_in_current = 0;
      ++current_bbs;

      new_bbs = true;
    }

    if ( current_bbs >= billboard_sets_.size() )
    {
      bbs = createBillboardSet();
      billboard_sets_.push_back( bbs );

      new_bbs = true;
    }

    if ( new_bbs || !bbs )
    {
      bbs = billboard_sets_[ current_bbs ];
      bbs->beginBillboards( std::min<uint32_t>( point_count_ - i, points_per_bbs_ ) );

      bbs->setVisible( true );

      used.push_back( bbs );
    }

    Point& p = points_[i];

    bb.mPosition.x = p.x_;
    bb.mPosition.y = p.y_;
    bb.mPosition.z = p.z_;
    bb.mColour.r = p.r_;
    bb.mColour.g = p.g_;
    bb.mColour.b = p.b_;
    bb.mColour.a = alpha_;

    bbs->injectBillboard(bb);
  }

  bbs->endBillboards();

  for ( uint32_t i = current_bbs + 1; i < billboard_sets_.size(); ++i )
  {
    billboard_sets_[i]->setVisible( false );
  }
}

} // namespace ogre_tools
