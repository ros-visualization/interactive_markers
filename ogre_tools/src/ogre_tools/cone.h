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
  ~Cone();

  void Create( int xTes, int yTes, float r, float g, float b );

  void SetOffset( const Ogre::Vector3& offset );
  void SetColor( float r, float g, float b );

  virtual void SetPosition( const Ogre::Vector3& position );
  virtual void SetOrientation( const Ogre::Quaternion& orientation );
  virtual void SetScale( const Ogre::Vector3& scale );

protected:
  void GetVertex( double theta, double h, Ogre::Vector3& vertex );
  void GetNormal( double theta, double h, Ogre::Vector3& normal );

  Ogre::SceneNode* m_SceneNode;
  Ogre::SceneNode* m_OffsetNode;
  Ogre::ManualObject* m_ManualObject;
  Ogre::MaterialPtr m_Material;
  std::string m_MaterialName;

  int m_XTes;
  int m_YTes;
};

} // namespace ogre_tools

#endif /* OGRE_TOOLS_CONE_H */
