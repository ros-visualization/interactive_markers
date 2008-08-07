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

// adapted from http://www.ogre3d.org/wiki/index.php/SuperEllipsoid

#ifndef OGRE_TOOLS_SUPER_ELLIPSOID_H
#define OGRE_TOOLS_SUPER_ELLIPSOID_H

#include "object.h"

#include <OgreMaterial.h>
#include <OgreVector3.h>

namespace Ogre
{
class SceneManager;
class SceneNode;
}

namespace ogre_tools
{

class SuperEllipsoid : public Object
{
public:
  enum Shape
  {
    Cube,
    RoundedCube,
    Cylinder,
    Sphere,
  };

  SuperEllipsoid(Ogre::SceneManager* sceneManager, Ogre::SceneNode* parentNode = NULL);
  virtual ~SuperEllipsoid();

  void Create(int samples, float n1, float n2, const Ogre::Vector3& scale = Ogre::Vector3( 1.0f, 1.0f, 1.0f ));
  void Create(Shape shape, int samples = 60, const Ogre::Vector3& scale = Ogre::Vector3( 1.0f, 1.0f, 1.0f ));

  virtual void SetColor( float r, float g, float b );
  void SetOffset( const Ogre::Vector3&  offset );

  virtual void SetPosition( const Ogre::Vector3& position );
  virtual void SetOrientation( const Ogre::Quaternion& orientation );
  virtual void SetScale( const Ogre::Vector3& scale );

  Ogre::SceneNode* GetRootNode() { return m_SceneNode; }

private:
  Ogre::Vector3 Sample(float phi, float beta, float n1, float n2,
                                     float scaleX = 1.0, float scaleY = 1.0, float scaleZ = 1.0);
  Ogre::Vector3 CalculateNormal(float phi, float beta, float n1, float n2,
                                float scaleX, float scaleY, float scaleZ);

  Ogre::SceneNode* m_SceneNode;
  Ogre::SceneNode* m_OffsetNode;
  Ogre::ManualObject* m_ManualObject;
  Ogre::MaterialPtr m_Material;
  std::string m_MaterialName;
};

} // namespace ogre_tools

#endif
