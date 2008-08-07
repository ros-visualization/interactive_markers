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

#include "axes.h"
#include "super_ellipsoid.h"

#include <Ogre.h>

#include <sstream>

namespace ogre_tools
{

Axes::Axes( Ogre::SceneManager* sceneManager, Ogre::SceneNode* parentNode, float length, float radius )
    : Object( sceneManager )
{
  if ( !parentNode )
  {
    parentNode = m_SceneManager->getRootSceneNode();
  }

  m_SceneNode = parentNode->createChildSceneNode();

  m_XAxis = new SuperEllipsoid( m_SceneManager, m_SceneNode );
  m_YAxis = new SuperEllipsoid( m_SceneManager, m_SceneNode );
  m_ZAxis = new SuperEllipsoid( m_SceneManager, m_SceneNode );

  Set( length, radius );
}

Axes::~Axes()
{
  delete m_XAxis;
  delete m_YAxis;
  delete m_ZAxis;

  m_SceneManager->destroySceneNode( m_SceneNode->getName() );
}

void Axes::Set( float length, float radius )
{
  m_XAxis->Create( SuperEllipsoid::Cylinder, 20, Ogre::Vector3( radius, length, radius ) );
  m_YAxis->Create( SuperEllipsoid::Cylinder, 20, Ogre::Vector3( radius, length, radius ) );
  m_ZAxis->Create( SuperEllipsoid::Cylinder, 20, Ogre::Vector3( radius, length, radius ) );

  m_XAxis->SetPosition( Ogre::Vector3( length/2.0f, 0.0f, 0.0f ) );
  m_XAxis->SetOrientation( Ogre::Quaternion( Ogre::Degree( -90 ), Ogre::Vector3::UNIT_Z ) );
  m_YAxis->SetPosition( Ogre::Vector3( 0.0f, length/2.0f, 0.0f ) );
  m_ZAxis->SetPosition( Ogre::Vector3( 0.0, 0.0f, length/2.0f ) );
  m_ZAxis->SetOrientation( Ogre::Quaternion( Ogre::Degree( 90 ), Ogre::Vector3::UNIT_X ) );

  m_XAxis->SetColor( 1.0f, 0.0f, 0.0f );
  m_YAxis->SetColor( 0.0f, 1.0f, 0.0f );
  m_ZAxis->SetColor( 0.0f, 0.0f, 1.0f );
}

void Axes::SetPosition( const Ogre::Vector3& position )
{
  m_SceneNode->setPosition( position );
}

void Axes::SetOrientation( const Ogre::Quaternion& orientation )
{
  m_SceneNode->setOrientation( orientation );
}

void Axes::SetScale( const Ogre::Vector3& scale )
{
  m_SceneNode->setScale( scale );
}

void Axes::SetColor( float r, float g, float b )
{
  // for now, do nothing
  // TODO: what should be done here?
}

} // namespace ogre_tools

