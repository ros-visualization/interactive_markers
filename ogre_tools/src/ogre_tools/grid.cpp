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

#include <OgreSceneManager.h>
#include <OgreSceneNode.h>
#include <OgreVector3.h>
#include <OgreQuaternion.h>
#include <OgreManualObject.h>
#include <OgreMaterialManager.h>

#include <sstream>

namespace ogre_tools
{

Grid::Grid( Ogre::SceneManager* scene_manager, uint32_t gridSize, float cell_length, float r, float g, float b )
    : scene_manager_( scene_manager )
{
  static uint32_t gridCount = 0;
  std::stringstream ss;
  ss << "Grid" << gridCount++;

  manual_object_ = scene_manager_->createManualObject( ss.str() );

  scene_node_ = scene_manager_->getRootSceneNode()->createChildSceneNode();
  scene_node_->attachObject( manual_object_ );

  set( gridSize, cell_length, r, g, b );
}

Grid::~Grid()
{
  scene_manager_->destroySceneNode( scene_node_->getName() );
  scene_manager_->destroyManualObject( manual_object_ );
}

void Grid::set( uint32_t gridSize, float cell_length, float r, float g, float b )
{
  manual_object_->clear();

  manual_object_->estimateVertexCount( gridSize * 4 );
  manual_object_->begin( "BaseWhiteNoLighting", Ogre::RenderOperation::OT_LINE_LIST );

  float extent = (cell_length*((double)gridSize))/2;

  for( uint32_t i = 0; i <= gridSize; i++ )
  {
    float inc = extent - ( i * cell_length );

    manual_object_->position( inc, 0, -extent );
    manual_object_->colour( r, g, b );
    manual_object_->position( inc, 0, extent );
    manual_object_->colour( r, g, b );

    manual_object_->position( -extent, 0, inc );
    manual_object_->colour( r, g, b );
    manual_object_->position( extent, 0, inc );
    manual_object_->colour( r, g, b );
  }

  manual_object_->end();
}

void Grid::setUserData( const Ogre::Any& data )
{
  manual_object_->setUserAny( data );
}

} // namespace ogre_tools
