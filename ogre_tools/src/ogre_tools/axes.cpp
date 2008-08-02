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

#include "Ogre.h"
#include "OgreSceneManager.h"

#include <sstream>

namespace ogre_tools
{

Axes::Axes( Ogre::SceneManager* sceneManager, float length, float radius )
    : m_SceneManager( sceneManager )
{
  m_SceneNode = m_SceneManager->getRootSceneNode()->createChildSceneNode();

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

  m_SceneNode->detachAllObjects();
  m_SceneNode->getParentSceneNode()->removeAndDestroyChild( m_SceneNode->getName() );
}

void Axes::Set( float length, float radius )
{
  m_XAxis->Create( SuperEllipsoid::Cylinder, 60, Ogre::Vector3( radius, radius, length ) );
  m_YAxis->Create( SuperEllipsoid::Cylinder, 60, Ogre::Vector3( radius, radius, length) );
  m_ZAxis->Create( SuperEllipsoid::Cylinder, 60, Ogre::Vector3( radius, radius, length ) );

  m_XAxis->SetColor( 1.0f, 0.0f, 0.0f );
  m_YAxis->SetColor( 0.0f, 1.0f, 0.0f );
  m_ZAxis->SetColor( 0.0f, 0.0f, 1.0f );

  m_XAxis->SetOffset( Ogre::Vector3( length/2.0, 0.0, 0.0 ) );
  m_XAxis->SetOrientation( Ogre::Quaternion( Ogre::Degree( 90 ), Ogre::Vector3::UNIT_Y ) );

  m_YAxis->SetOffset( Ogre::Vector3( 0.0, length/2.0, 0.0 ) );
  m_YAxis->SetOrientation( Ogre::Quaternion( Ogre::Degree( -90 ), Ogre::Vector3::UNIT_X ) );

  m_ZAxis->SetOffset( Ogre::Vector3( 0.0, 0.0, length/2.0 ) );
  m_ZAxis->SetOrientation( Ogre::Quaternion( Ogre::Degree( 180 ), Ogre::Vector3::UNIT_X ) );
}

void Axes::SetPosition( Ogre::Vector3& position )
{
  m_SceneNode->setPosition( position );
}

void Axes::SetOrientation( Ogre::Quaternion& orientation )
{
  m_SceneNode->setOrientation( orientation );
}

} // namespace ogre_tools

