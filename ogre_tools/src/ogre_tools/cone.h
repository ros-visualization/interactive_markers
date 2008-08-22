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

#ifndef OGRE_TOOLS_CONE_H
#define OGRE_TOOLS_CONE_H

#include "object.h"

#include <OgreMaterial.h>

namespace Ogre
{
class SceneManager;
class SceneNode;
class ManualObject;
class Vector3;
}

namespace ogre_tools
{

class Cone : public Object
{
public:
  Cone( Ogre::SceneManager* sceneManager, Ogre::SceneNode* parentNode = 0, int xTes = 20, int yTes = 20, float r = 1.0f, float g = 1.0f, float b = 1.0f );
  virtual ~Cone();

  void create( int xTes, int yTes, float r, float g, float b );

  void setOffset( const Ogre::Vector3& offset );
  virtual void setColor( float r, float g, float b );

  virtual void setPosition( const Ogre::Vector3& position );
  virtual void setOrientation( const Ogre::Quaternion& orientation );
  virtual void setScale( const Ogre::Vector3& scale );

protected:
  void getVertex( double theta, double h, Ogre::Vector3& vertex );
  void getNormal( double theta, double h, Ogre::Vector3& normal );

  Ogre::SceneNode* scene_node_;
  Ogre::SceneNode* offset_node_;
  Ogre::ManualObject* manual_object_;
  Ogre::MaterialPtr material_;
  std::string material_name_;

  int x_tes_;
  int y_tes_;
};

} // namespace ogre_tools

#endif /* OGRE_TOOLS_CONE_H */
