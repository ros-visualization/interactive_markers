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

#include "arrow.h"
#include "super_ellipsoid.h"
#include "cone.h"

#include <OgreSceneManager.h>
#include <OgreSceneNode.h>
#include <OgreVector3.h>
#include <OgreQuaternion.h>

#include <sstream>

namespace ogre_tools
{

Arrow::Arrow( Ogre::SceneManager* scene_manager, Ogre::SceneNode* parent_node, float shaft_length, float shaft_radius,
              float head_length, float head_radius )
: Object( scene_manager )
{
  if ( !parent_node )
  {
    parent_node = scene_manager_->getRootSceneNode();
  }

  scene_node_ = parent_node->createChildSceneNode();

  shaft_ = new SuperEllipsoid( scene_manager_, scene_node_ );
  head_ = new Cone( scene_manager_, scene_node_, 5, 5 );

  set( shaft_length, shaft_radius, head_length, head_radius );

  setOrientation( Ogre::Quaternion::IDENTITY );
}

Arrow::~Arrow()
{
  delete shaft_;
  delete head_;

  scene_manager_->destroySceneNode( scene_node_->getName() );
}

void Arrow::set( float shaft_length, float shaft_radius, float head_length, float head_radius )
{
  shaft_->create( SuperEllipsoid::Cylinder, 5, Ogre::Vector3( shaft_radius, shaft_length, shaft_radius ) );
  shaft_->setColor( 0.5f, 0.5f, 0.5f, 1.0f );
  shaft_->setPosition( Ogre::Vector3( 0.0f, shaft_length/2.0f, 0.0f ) );

  head_->setScale( Ogre::Vector3( head_radius, head_length, head_radius ) );
  head_->setColor( 0.5f, 0.5f, 0.5f, 1.0f );
  head_->setPosition( Ogre::Vector3( 0.0f, shaft_length, 0.0f ) );
}

void Arrow::setColor( float r, float g, float b, float a )
{
  shaft_->setColor( r, g, b, a );
  head_->setColor( r, g, b, a );
}

void Arrow::setShaftColor( float r, float g, float b, float a )
{
  shaft_->setColor( r, g, b, a );
}

void Arrow::setHeadColor( float r, float g, float b, float a )
{
  head_->setColor( r, g, b, a );
}

void Arrow::setPosition( const Ogre::Vector3& position )
{
  scene_node_->setPosition( position );
}

void Arrow::setOrientation( const Ogre::Quaternion& orientation )
{
  // "forward" (negative z) should always be our identity orientation
  scene_node_->setOrientation( orientation * Ogre::Quaternion( Ogre::Degree( -90 ), Ogre::Vector3::UNIT_X ) );
}

void Arrow::setScale( const Ogre::Vector3& scale )
{
  scene_node_->setScale( Ogre::Vector3( scale.x, scale.z, scale.y ) );
}

const Ogre::Vector3& Arrow::getPosition()
{
  return scene_node_->getPosition();
}

const Ogre::Quaternion& Arrow::getOrientation()
{
  return scene_node_->getOrientation();
}

void Arrow::setUserData( const Ogre::Any& data )
{
  head_->setUserData( data );
  shaft_->setUserData( data );
}

} // namespace ogre_tools

