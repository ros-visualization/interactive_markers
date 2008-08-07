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

#include "grid.h"

#include <Ogre.h>


#include <sstream>

namespace ogre_tools
{

Grid::Grid( Ogre::SceneManager* sceneManager, uint32_t gridSize, float cellLength, float r, float g, float b )
    : m_SceneManager( sceneManager )
{
  static uint32_t gridCount = 0;
  std::stringstream ss;
  ss << "Grid" << gridCount++;

  m_ManualObject = m_SceneManager->createManualObject( ss.str() );

  m_SceneNode = m_SceneManager->getRootSceneNode()->createChildSceneNode();
  m_SceneNode->attachObject( m_ManualObject );

  Set( gridSize, cellLength, r, g, b );
}

Grid::~Grid()
{
  m_SceneManager->destroySceneNode( m_SceneNode->getName() );
  m_SceneManager->destroyManualObject( m_ManualObject );
}

void Grid::Set( uint32_t gridSize, float cellLength, float r, float g, float b )
{
  m_ManualObject->clear();

  m_ManualObject->estimateVertexCount( gridSize * 4 );
  m_ManualObject->begin( "BaseWhiteNoLighting", Ogre::RenderOperation::OT_LINE_LIST );

  float extent = (cellLength*((double)gridSize))/2;

  for( uint32_t i = 0; i <= gridSize; i++ )
  {
    float inc = extent - ( i * cellLength );

    m_ManualObject->position( inc, 0, -extent );
    m_ManualObject->colour( r, g, b );
    m_ManualObject->position( inc, 0, extent );
    m_ManualObject->colour( r, g, b );

    m_ManualObject->position( -extent, 0, inc );
    m_ManualObject->colour( r, g, b );
    m_ManualObject->position( extent, 0, inc );
    m_ManualObject->colour( r, g, b );
  }

  m_ManualObject->end();
}

} // namespace ogre_tools
