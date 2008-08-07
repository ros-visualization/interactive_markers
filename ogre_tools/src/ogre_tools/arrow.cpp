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

#include <Ogre.h>

#include <sstream>

namespace ogre_tools
{

Arrow::Arrow( Ogre::SceneManager* sceneManager, Ogre::SceneNode* parentNode, float shaftLength, float shaftRadius,
              float headLength, float headRadius )
: Object( sceneManager )
{
  if ( !parentNode )
  {
    parentNode = m_SceneManager->getRootSceneNode();
  }

  m_SceneNode = parentNode->createChildSceneNode();

  m_Shaft = new SuperEllipsoid( m_SceneManager, m_SceneNode );
  m_Head = new Cone( m_SceneManager, m_SceneNode );

  Set( shaftLength, shaftRadius, headLength, headRadius );

  SetOrientation( Ogre::Quaternion::IDENTITY );
}

Arrow::~Arrow()
{
  delete m_Shaft;
  delete m_Head;

  m_SceneManager->destroySceneNode( m_SceneNode->getName() );
}

void Arrow::Set( float shaftLength, float shaftRadius, float headLength, float headRadius )
{
  m_Shaft->Create( SuperEllipsoid::Cylinder, 20, Ogre::Vector3( shaftRadius, shaftLength, shaftRadius ) );
  m_Shaft->SetColor( 0.5f, 0.5f, 0.5f );
  m_Shaft->SetPosition( Ogre::Vector3( 0.0f, shaftLength/2.0f, 0.0f ) );

  m_Head->SetScale( Ogre::Vector3( headRadius, headLength, headRadius ) );
  m_Head->SetColor( 0.5f, 0.5f, 0.5f );
  m_Head->SetPosition( Ogre::Vector3( 0.0f, shaftLength, 0.0f ) );
}

void Arrow::SetColor( float r, float g, float b )
{
  m_Shaft->SetColor( r, g, b );
  m_Head->SetColor( r, g, b );
}

void Arrow::SetShaftColor( float r, float g, float b )
{
  m_Shaft->SetColor( r, g, b );
}

void Arrow::SetHeadColor( float r, float g, float b )
{
  m_Head->SetColor( r, g, b );
}

void Arrow::SetPosition( const Ogre::Vector3& position )
{
  m_SceneNode->setPosition( position );
}

void Arrow::SetOrientation( const Ogre::Quaternion& orientation )
{
  // "forward" (negative z) should always be our identity orientation
  m_SceneNode->setOrientation( orientation * Ogre::Quaternion( Ogre::Degree( -90 ), Ogre::Vector3::UNIT_X ) );
}

void Arrow::SetScale( const Ogre::Vector3& scale )
{
  m_SceneNode->setScale( Ogre::Vector3( scale.x, scale.z, scale.y ) );
}

} // namespace ogre_tools

