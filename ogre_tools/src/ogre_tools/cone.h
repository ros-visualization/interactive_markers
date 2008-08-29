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

/**
 * \class Cone
 * \brief A tesselated cone object
 *
 * The cone is created in unit space, with the only control over its with or height defined by its scale
 * Uses the parametric equation of a cone:
 @verbatim
 x = (1 - h)*0.5*cos(theta);
 y = h - 0.5;
 z = (1 - h)*0.5*sin(theta);
 @endverbatim
 * where h is the height ([0,1]), and theta is the angle ([0,2*pi)).
 */
class Cone : public Object
{
public:
  /**
   * \brief Constructor
   * @param scene_manager Scene manager this cone is part of
   * @param parent_node Parent node to attach this cone to.  If NULL, uses the root node
   * @param x_tes Number of sections to divide the cone into along the cone's width
   * @param y_tes Number of sections to divide the cone into along the cone's height
   */
  Cone( Ogre::SceneManager* scene_manager, Ogre::SceneNode* parent_node = 0, int x_tes = 20, int y_tes = 20 );
  virtual ~Cone();

  /**
   * \brief Set the tesselation parameters for this cone.
   *
   * @param x_tes Number of sections to divide the cone into along the cone's width
   * @param y_tes Number of sections to divide the cone into along the cone's height
   */
  void create( int x_tes, int y_tes );

  /**
   * \brief Set the offset for this cone.
   * The constructor sets this to (0.0, 0.5, 0.0) so the cone's pivot point is at the center of its base
   * @param offset Amount to offset by
   */
  void setOffset( const Ogre::Vector3& offset );

  virtual void setColor( float r, float g, float b );
  virtual void setPosition( const Ogre::Vector3& position );
  virtual void setOrientation( const Ogre::Quaternion& orientation );
  virtual void setScale( const Ogre::Vector3& scale );
  virtual const Ogre::Vector3& getPosition();
  virtual const Ogre::Quaternion& getOrientation();

protected:
  /**
   * Get a vertex of the cone
   * @param theta Angle around Y
   * @param h Height
   * @param vertex Return value -- the position of the cone
   */
  void getVertex( double theta, double h, Ogre::Vector3& vertex );
  /**
   * Get a normal of the cone
   * @param theta Angle around Y
   * @param h Height
   * @param vertex Return value -- the normal of the cone
   */
  void getNormal( double theta, double h, Ogre::Vector3& normal );

  Ogre::SceneNode* scene_node_;
  Ogre::SceneNode* offset_node_;

  Ogre::ManualObject* manual_object_;     ///< The ogre object used to display this cone
  Ogre::MaterialPtr material_;            ///< The ogre material used on this cone
  std::string material_name_;             ///< The name of the material created for (and used by) this cone
};

} // namespace ogre_tools

#endif /* OGRE_TOOLS_CONE_H */
