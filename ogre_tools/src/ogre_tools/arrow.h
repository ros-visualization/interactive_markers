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

#include "object.h"

#ifndef OGRE_TOOLS_ARROW_H
#define OGRE_TOOLS_ARROW_H

namespace Ogre
{
class SceneManager;
class SceneNode;
class Vector3;
class Quaternion;
}

namespace ogre_tools
{
class SuperEllipsoid;
class Cone;

class Arrow : public Object
{
public:
  Arrow( Ogre::SceneManager* manager, Ogre::SceneNode* parent_node = 0, float shaft_length = 1.0f, float shaft_radius = 0.1f,
      float head_length = 0.3f, float head_radius =  0.2f );
  virtual ~Arrow();

  void set( float shaft_length, float shaft_radius, float head_length, float head_radius );

  virtual void setColor( float r, float g, float b );
  void setHeadColor( float r, float g, float b );
  void setShaftColor( float r, float g, float b );

  virtual void setOrientation( const Ogre::Quaternion& orientation );
  virtual void setPosition( const Ogre::Vector3& position );
  virtual void setScale( const Ogre::Vector3& scale );

  Ogre::SceneNode* getSceneNode() { return scene_node_; }

private:
  Ogre::SceneNode* scene_node_;

  SuperEllipsoid* shaft_;
  Cone* head_;
};

} // namespace ogre_tools

#endif /* OGRE_TOOLS_ARROW_H */
