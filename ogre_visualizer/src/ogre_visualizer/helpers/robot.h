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

#ifndef OGRE_VISUALIZER_ROBOT_H_
#define OGRE_VISUALIZER_ROBOT_H_

#include <string>
#include <map>

#include <urdf/URDF.h>

namespace Ogre
{
class SceneManager;
class Entity;
class SceneNode;
}

namespace ogre_tools
{
class Object;
}

namespace robot_desc
{
class URDF;
}

class rosTFClient;

namespace ogre_vis
{

/**
 * \class Robot
 *
 * A helper class to draw a representation of a robot, as specified by a URDF.  Can display either the visual models of the robot,
 * or the collision models.
 */
class Robot
{
public:
  Robot( Ogre::SceneManager* scene_manager );
  ~Robot();

  /**
   * \brief Loads meshes/primitives from a robot description.  Calls clear() before loading.
   *
   * @param urdf The robot description to read from
   */
  void load( robot_desc::URDF* urdf );
  /**
   * \brief Clears all data loaded from a URDF
   */
  void clear();

  /**
   * \brief Updates positions/orientations from a rosTF client
   *
   * @param tf_client The rosTF client to load from
   */
  void update( rosTFClient* tf_client, const std::string& target_frame );

  /**
   * \brief Set the robot as a whole to be visible or not
   * @param visible Should we be visible?
   */
  void setVisible( bool visible );

  /**
   * \brief Set whether the visual meshes of the robot should be visible
   * @param visible Whether the visual meshes of the robot should be visible
   */
  void setVisualVisible( bool visible );

  /**
   * \brief Set whether the collision meshes/primitives of the robot should be visible
   * @param visible Whether the collision meshes/primitives should be visible
   */
  void setCollisionVisible( bool visible );

  bool isVisible();
  bool isVisualVisible();
  bool isCollisionVisible();

protected:
  /**
   * \struct LinkInfo
   * \brief Contains any data we need from a link in the robot.
   */
  struct LinkInfo
  {
    LinkInfo()
    : visual_mesh_( NULL )
    , collision_mesh_( NULL )
    , collision_object_( NULL )
    , visual_node_( NULL )
    , collision_node_( NULL )
    {}

    std::string name_;                          ///< Name of this link
    std::string material_name_;                 ///< Name of the ogre material used by the meshes in this link

    Ogre::Entity* visual_mesh_;                 ///< The entity representing the visual mesh of this link (if it exists)

    Ogre::Entity* collision_mesh_;              ///< The entity representing the collision mesh of this link (if it exists)
    ogre_tools::Object* collision_object_;      ///< The object representing the collision primitive of this link (if it exists)

    Ogre::SceneNode* visual_node_;              ///< The scene node the visual mesh is attached to
    Ogre::SceneNode* collision_node_;           ///< The scene node the collision mesh/primitive is attached to
  };


  void createCollisionForLink( LinkInfo& info, robot_desc::URDF::Link* link );


  typedef std::map< std::string, LinkInfo > M_NameToLinkInfo;
  M_NameToLinkInfo links_;                      ///< Map of name to link info, stores all loaded links.

  Ogre::SceneManager* scene_manager_;
  Ogre::SceneNode* root_visual_node_;           ///< Node all our visual nodes are children of
  Ogre::SceneNode* root_collision_node_;        ///< Node all our collision nodes are children of

  bool visual_visible_;
  bool collision_visible_;
};

} // namespace ogre_vis

#endif /* OGRE_VISUALIZER_ROBOT_H_ */
