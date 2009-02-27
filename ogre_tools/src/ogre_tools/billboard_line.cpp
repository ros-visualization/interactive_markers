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

#include "billboard_line.h"

#include <OGRE/OgreSceneManager.h>
#include <OGRE/OgreSceneNode.h>
#include <OGRE/OgreVector3.h>
#include <OGRE/OgreQuaternion.h>
#include <OGRE/OgreBillboardChain.h>
#include <OGRE/OgreMaterialManager.h>

#include <sstream>

#include <ros/assert.h>

#define MAX_ELEMENTS (65536/4)

namespace ogre_tools
{

BillboardLine::BillboardLine( Ogre::SceneManager* scene_manager, Ogre::SceneNode* parent_node )
: Object( scene_manager )
, width_( 0.1f )
, current_line_(0)
, total_elements_(0)
{
  if ( !parent_node )
  {
    parent_node = scene_manager_->getRootSceneNode();
  }

  scene_node_ = parent_node->createChildSceneNode();

  std::stringstream ss;
  static int count = 0;
  ss << "BillboardLine chain" << count++;
  chain_ = scene_manager_->createBillboardChain(ss.str());
  setNumLines(1);
  setMaxPointsPerLine(100);

  scene_node_->attachObject( chain_ );

  ss << "Material";
  material_ = Ogre::MaterialManager::getSingleton().create( ss.str(), Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME );
  material_->setReceiveShadows(false);
  material_->getTechnique(0)->setLightingEnabled(false);

  chain_->setMaterialName( material_->getName() );
}

BillboardLine::~BillboardLine()
{
  scene_manager_->destroyBillboardChain(chain_);
  scene_manager_->destroySceneNode( scene_node_->getName() );
}

void BillboardLine::clear()
{
  chain_->clearAllChains();
  current_line_ = 0;
  total_elements_ = 0;

  for (V_uint32::iterator it = num_elements_.begin(); it != num_elements_.end(); ++it)
  {
    *it = 0;
  }
}

void BillboardLine::setMaxPointsPerLine(uint32_t max)
{
  chain_->setMaxChainElements(max);
}

void BillboardLine::setNumLines(uint32_t num)
{
  chain_->setNumberOfChains(num);
  num_elements_.resize(num);
}

void BillboardLine::newLine()
{
  ++current_line_;

  ROS_ASSERT(current_line_ < chain_->getNumberOfChains());
}

void BillboardLine::addPoint( const Ogre::Vector3& point )
{
  if (total_elements_ >= MAX_ELEMENTS)
  {
    ROS_ERROR("BillboardLine is full.  Cannot add any more points. (max %d segments)", MAX_ELEMENTS);
    return;
  }

  ++num_elements_[current_line_];
  ++total_elements_;

  ROS_ASSERT(num_elements_[current_line_] <=  (chain_->getMaxChainElements() * (current_line_+1)));

  Ogre::BillboardChain::Element e;
  e.position = point;
  e.width = width_;
  e.colour = color_;
  chain_->addChainElement(current_line_, e);
}

void BillboardLine::setPoints( const V_Vector3& points )
{
  clear();

  V_Vector3::const_iterator it = points.begin();
  V_Vector3::const_iterator end = points.end();
  for (; it != end; ++it)
  {
    addPoint( *it );
  }
}

void BillboardLine::setLineWidth( float width )
{
  width_ = width;

  uint32_t num_chains = chain_->getNumberOfChains();
  for (uint32_t chain = 0; chain < num_chains; ++chain)
  {
    uint32_t element_count = num_elements_[chain];

    for ( uint32_t i = 0; i < element_count; ++i )
    {
      Ogre::BillboardChain::Element e = chain_->getChainElement(chain, i);

      e.width = width_;
      chain_->updateChainElement(chain, i, e);
    }
  }
}

void BillboardLine::setPosition( const Ogre::Vector3& position )
{
  scene_node_->setPosition( position );
}

void BillboardLine::setOrientation( const Ogre::Quaternion& orientation )
{
  scene_node_->setOrientation( orientation );
}

void BillboardLine::setScale( const Ogre::Vector3& scale )
{
  // Setting scale doesn't really make sense here
}

void BillboardLine::setColor( float r, float g, float b, float a )
{
  if ( a < 0.9998 )
  {
    material_->setSceneBlending( Ogre::SBT_TRANSPARENT_ALPHA );
    material_->setDepthWriteEnabled( false );
  }
  else
  {
    material_->setSceneBlending( Ogre::SBT_REPLACE );
    material_->setDepthWriteEnabled( true );
  }

  color_ = Ogre::ColourValue( r, g, b, a );

  uint32_t num_chains = chain_->getNumberOfChains();
  for (uint32_t chain = 0; chain < num_chains; ++chain)
  {
    uint32_t element_count = num_elements_[chain];

    for ( uint32_t i = 0; i < element_count; ++i )
    {
      Ogre::BillboardChain::Element e = chain_->getChainElement(chain, i);

      e.colour = color_;
      chain_->updateChainElement(chain, i, e);
    }
  }
}

const Ogre::Vector3& BillboardLine::getPosition()
{
  return scene_node_->getPosition();
}

const Ogre::Quaternion& BillboardLine::getOrientation()
{
  return scene_node_->getOrientation();
}

} // namespace ogre_tools


