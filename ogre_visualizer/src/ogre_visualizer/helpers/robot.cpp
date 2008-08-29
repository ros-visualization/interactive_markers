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

#include "robot.h"
#include "common.h"

#include "ogre_tools/object.h"
#include "ogre_tools/super_ellipsoid.h"

#include <rosTF/rosTF.h>

#include <Ogre.h>

namespace ogre_vis
{

Robot::Robot( Ogre::SceneManager* scene_manager )
: scene_manager_( scene_manager )
, visual_visible_( true )
, collision_visible_( false )
{
  root_visual_node_ = scene_manager_->getRootSceneNode()->createChildSceneNode();
  root_collision_node_ = scene_manager_->getRootSceneNode()->createChildSceneNode();

  setVisualVisible( visual_visible_ );
  setCollisionVisible( collision_visible_ );
}

Robot::~Robot()
{
  clear();

  scene_manager_->destroySceneNode( root_visual_node_->getName() );
  scene_manager_->destroySceneNode( root_collision_node_->getName() );
}

void Robot::setVisible( bool visible )
{
  if ( visible )
  {
    root_visual_node_->setVisible( visual_visible_ );
    root_collision_node_->setVisible( collision_visible_ );
  }
  else
  {
    root_visual_node_->setVisible( false );
    root_collision_node_->setVisible( false );
  }
}

void Robot::setVisualVisible( bool visible )
{
  visual_visible_ = visible;
  root_visual_node_->setVisible( visible );
}

void Robot::setCollisionVisible( bool visible )
{
  collision_visible_ = visible;
  root_collision_node_->setVisible( visible );
}

bool Robot::isVisible()
{
  return isVisualVisible() || isCollisionVisible();
}

bool Robot::isVisualVisible()
{
  return visual_visible_;
}

bool Robot::isCollisionVisible()
{
  return collision_visible_;
}

void Robot::clear()
{
  M_NameToLinkInfo::iterator it = links_.begin();
  M_NameToLinkInfo::iterator end = links_.end();
  for ( ; it != end; ++it )
  {
    LinkInfo& info = it->second;

    scene_manager_->destroyEntity( info.visual_mesh_ );
    delete info.collision_object_;
  }

  links_.clear();
  root_visual_node_->removeAndDestroyAllChildren();
  root_collision_node_->removeAndDestroyAllChildren();
}

void Robot::createCollisionForLink( LinkInfo& info, robot_desc::URDF::Link* link )
{
  if ( !link->isSet[ "collision" ] )
  {
    return;
  }

  robot_desc::URDF::Link::Collision* collision = link->collision;
  robot_desc::URDF::Link::Geometry* geometry = collision->geometry;

  info.collision_node_ = root_collision_node_->createChildSceneNode();

  switch ( geometry->type )
  {
  case robot_desc::URDF::Link::Geometry::BOX:
    {
      robot_desc::URDF::Link::Geometry::Box* box = static_cast<robot_desc::URDF::Link::Geometry::Box*>( geometry->shape );

      ogre_tools::SuperEllipsoid* obj = new ogre_tools::SuperEllipsoid( scene_manager_, info.collision_node_ );

      Ogre::Vector3 scale( box->size[0], box->size[1], box->size[2] );

      obj->create( ogre_tools::SuperEllipsoid::Cube, 10, scale );
      info.collision_object_ = obj;
    }
    break;

  case robot_desc::URDF::Link::Geometry::SPHERE:
    {
      robot_desc::URDF::Link::Geometry::Sphere* sphere = static_cast<robot_desc::URDF::Link::Geometry::Sphere*>( geometry->shape );

      ogre_tools::SuperEllipsoid* obj = new ogre_tools::SuperEllipsoid( scene_manager_, info.collision_node_ );

      Ogre::Vector3 scale( sphere->radius, sphere->radius, sphere->radius );
      // No need to convert robot->ogre because a sphere is uniform

      obj->create( ogre_tools::SuperEllipsoid::Sphere, 20, scale );
      info.collision_object_ = obj;
    }
    break;

  case robot_desc::URDF::Link::Geometry::CYLINDER:
    {
      robot_desc::URDF::Link::Geometry::Cylinder* cylinder = static_cast<robot_desc::URDF::Link::Geometry::Cylinder*>( geometry->shape );

      ogre_tools::SuperEllipsoid* obj = new ogre_tools::SuperEllipsoid( scene_manager_, info.collision_node_ );
      Ogre::Vector3 scale( cylinder->radius*2, cylinder->length, cylinder->radius*2 );

      obj->create( ogre_tools::SuperEllipsoid::Cylinder, 20, scale );
      info.collision_object_ = obj;
    }
    break;

  default:
    printf( "Unsupported collision shape type: %d\n", geometry->type );
  }

  if ( info.collision_object_ )
  {
    Ogre::Vector3 position( collision->xyz[0], collision->xyz[1], collision->xyz[2] );
    Ogre::Matrix3 orientation;
    orientation.FromEulerAnglesYXZ( Ogre::Radian( collision->rpy[0] ), Ogre::Radian( collision->rpy[1] ), Ogre::Radian( collision->rpy[2] ) );

    info.collision_object_->setPosition( position );
    info.collision_object_->setOrientation( orientation );

    info.collision_object_->setColor( 0.0f, 0.6f, 1.0f );
  }
}

void Robot::load( robot_desc::URDF* urdf )
{
  clear();

  typedef std::vector<robot_desc::URDF::Link*> V_Link;
  V_Link links;
  urdf->getLinks(links);

  V_Link::iterator link_it = links.begin();
  V_Link::iterator link_end = links.end();
  for ( ; link_it != link_end; ++link_it )
  {
    robot_desc::URDF::Link* link = *link_it;
    robot_desc::URDF::Link::Geometry::Mesh* mesh = static_cast<robot_desc::URDF::Link::Geometry::Mesh*>(link->visual->geometry->shape);
    if ( mesh->filename.empty() )
    {
      continue;
    }

    std::string model_name = mesh->filename + "_hi.mesh";

    static int count = 0;
    std::stringstream ss;
    ss << "RobotVis" << count++ << " Link " << link->name ;

    LinkInfo info;

    info.name_ = link->name;

    try
    {
      info.visual_mesh_ = scene_manager_->createEntity( ss.str(), model_name );
    }
    catch( Ogre::Exception& e )
    {
      printf( "Could not load model '%s' for link '%s': %s\n", model_name.c_str(), link->name.c_str(), e.what() );
    }

    if ( info.visual_mesh_ )
    {
      info.visual_node_ = root_visual_node_->createChildSceneNode();
      info.visual_node_->attachObject( info.visual_mesh_ );

      // assign the material from the link
      uint16_t num_sub_entities = info.visual_mesh_->getNumSubEntities();
      for ( uint16_t i = 0; i < num_sub_entities; ++i )
      {
        Ogre::SubEntity* subEntity = info.visual_mesh_->getSubEntity( i );

        typedef std::vector<std::string> V_string;
        V_string gazebo_names;
        link->visual->data.getMapTagNames("gazebo", gazebo_names);

        V_string::iterator name_it = gazebo_names.begin();
        V_string::iterator name_end = gazebo_names.end();
        for ( ; name_it != name_end; ++name_it )
        {
          typedef std::map<std::string, std::string> M_string;
          M_string m = link->visual->data.getMapTagValues("gazebo", *name_it);

          M_string::iterator it = m.find( "material" );
          if ( it != m.end() )
          {
            info.material_name_ = it->second;
            subEntity->setMaterialName( info.material_name_ );
            break;
          }
        }
      }
    }

    createCollisionForLink( info, link );

    links_.insert( std::make_pair( info.name_, info ) );
  }
}

void Robot::update( rosTFClient* tf_client, const std::string& target_frame )
{
  M_NameToLinkInfo::iterator link_it = links_.begin();
  M_NameToLinkInfo::iterator link_end = links_.end();
  for ( ; link_it != link_end; ++link_it )
  {
    const std::string& name = link_it->first;
    const LinkInfo& info = link_it->second;

    libTF::TFPose pose = { 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0, "" };

    pose.frame = name;

    try
    {
      pose = tf_client->transformPose( target_frame, pose );
    }
    catch(libTF::TransformReference::LookupException& e)
    {
      printf( "Error transforming from frame '%s' to frame '%s': %s\n", name.c_str(), target_frame.c_str(), e.what() );
    }
    catch(libTF::TransformReference::ConnectivityException& e)
    {
      printf( "Error transforming from frame '%s' to frame '%s': %s\n", name.c_str(), target_frame.c_str(), e.what() );
    }
    catch(libTF::TransformReference::ExtrapolateException& e)
    {
      printf( "Error transforming from frame '%s' to frame '%s': %s\n", name.c_str(), target_frame.c_str(), e.what() );
    }
    catch(libTF::TransformReference::MaxDepthException& e)
    {
      printf( "Error transforming from frame '%s' to frame '%s': %s\n", name.c_str(), target_frame.c_str(), e.what() );
    }

    Ogre::Vector3 position( pose.x, pose.y, pose.z );
    robotToOgre( position );

    Ogre::Matrix3 orientation( ogreMatrixFromRobotEulers( pose.yaw, pose.pitch, pose.roll ) );

    if ( info.visual_node_ )
    {
      info.visual_node_->setPosition( position );
      info.visual_node_->setOrientation( orientation );
    }

    if ( info.collision_node_ )
    {
      info.collision_node_->setPosition( position );
      info.collision_node_->setOrientation( orientation );
    }
  }
}

} // namespace ogre_vis
