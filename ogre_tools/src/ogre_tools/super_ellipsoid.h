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

#ifndef OGRE_TOOLS_SUPER_ELLIPSOID_H
#define OGRE_TOOLS_SUPER_ELLIPSOID_H

#include "object.h"

#include <OgreMaterial.h>
#include <OgreVector3.h>

namespace Ogre
{
class SceneManager;
class SceneNode;
class Any;
}

namespace ogre_tools
{

/**
 * \brief Adapted from http://www.ogre3d.org/wiki/index.php/SuperEllipsoid
 *
 * For a good explanation, see http://en.wikipedia.org/wiki/Superellipse#Superellipsoid
 */
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

  /**
   * \brief Constructor
   *
   * @param scene_manager The scene manager this object is associated with
   * @param parent_node A scene node to use as the parent of this object.  If NULL, uses the root scene node.
   */
  SuperEllipsoid(Ogre::SceneManager* scene_manager, Ogre::SceneNode* parent_node = NULL);
  virtual ~SuperEllipsoid();

  /**
   * \brief Create a shape
   * @param samples Number of samples to use
   * @param n1
   * @param n2
   * @param scale The scale of the object
   */
  void create(int samples, float n1, float n2, const Ogre::Vector3& scale = Ogre::Vector3( 1.0f, 1.0f, 1.0f ));
  /**
   * \brief Create an object from one of the pre-defined shapes
   * @param shape The shape to use
   * @param samples The number of samples to use
   * @param scale The scale of the object
   */
  void create(Shape shape, int samples = 60, const Ogre::Vector3& scale = Ogre::Vector3( 1.0f, 1.0f, 1.0f ));

  /**
   * \brief Set the offset for this shape
   *
   * The default is no offset, which puts the pivot point directly in the center of the object.
   *
   * @param offset Amount to offset the center of the object from the pivot point
   */
  void setOffset( const Ogre::Vector3&  offset );

  virtual void setColor( float r, float g, float b );
  virtual void setPosition( const Ogre::Vector3& position );
  virtual void setOrientation( const Ogre::Quaternion& orientation );
  virtual void setScale( const Ogre::Vector3& scale );
  virtual const Ogre::Vector3& getPosition();
  virtual const Ogre::Quaternion& getOrientation();

  /**
   * \brief Get the root scene node (pivot node) for this object
   *
   * @return The root scene node of this object
   */
  Ogre::SceneNode* getRootNode() { return scene_node_; }

  /**
   * \brief Sets user data on all ogre objects we own
   */
  void setUserData( const Ogre::Any& data );

private:
  Ogre::Vector3 Sample(float phi, float beta, float n1, float n2,
                                     float scaleX = 1.0, float scaleY = 1.0, float scaleZ = 1.0);
  Ogre::Vector3 CalculateNormal(float phi, float beta, float n1, float n2,
                                float scaleX, float scaleY, float scaleZ);

  Ogre::SceneNode* scene_node_;
  Ogre::SceneNode* offset_node_;
  Ogre::ManualObject* manual_object_;
  Ogre::MaterialPtr material_;
  std::string material_name_;
};

} // namespace ogre_tools

#endif
