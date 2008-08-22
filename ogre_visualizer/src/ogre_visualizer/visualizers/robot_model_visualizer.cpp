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

#include "robot_model_visualizer.h"
#include "../common.h"

#include "ogre_tools/axes.h"

#include "urdf/URDF.h"
#include "rosTF/rosTF.h"

#include <Ogre.h>

namespace ogre_vis
{

RobotModelVisualizer::RobotModelVisualizer( Ogre::SceneManager* scene_manager, ros::node* node, rosTFClient* tf_client, const std::string& name, bool enabled )
: VisualizerBase( scene_manager, node, tf_client, name, enabled )
, has_new_transforms_( false )
, initialized_( false )
{
  root_node_ = scene_manager->getRootSceneNode()->createChildSceneNode();
}

RobotModelVisualizer::~RobotModelVisualizer()
{
  unsubscribe();

  clear();

  scene_manager_->destroySceneNode( root_node_->getName() );
}

void RobotModelVisualizer::clear()
{
  M_StringToEntity::iterator modelIt = models_.begin();
  M_StringToEntity::iterator modelEnd = models_.end();
  for ( ; modelIt != modelEnd; ++modelIt )
  {
    Ogre::Entity* entity = modelIt->second;
    Ogre::SceneNode* node = entity->getParentSceneNode();

    node->detachObject( entity );
    scene_manager_->destroyEntity( entity );
  }

  models_.clear();

  root_node_->removeAndDestroyAllChildren();
}

void RobotModelVisualizer::initialize( const std::string& description_param, const std::string& transform_topic )
{
  transform_topic_ = transform_topic;
  description_param_ = description_param;

  initialized_ = true;

  if ( isEnabled() )
  {
    onEnable();
    causeRender();
  }
  else
  {
    root_node_->setVisible( false );
  }
}

void RobotModelVisualizer::load()
{
  clear();

  std::string content;
  ros_node_->get_param(description_param_, content);
  robot_desc::URDF* file = new robot_desc::URDF();
  std::auto_ptr<robot_desc::URDF> file_ptr( file );

  file->loadString(content.c_str());

  typedef std::vector<robot_desc::URDF::Link*> V_Link;
  V_Link links;
  file->getLinks(links);

  V_Link::iterator linkIt = links.begin();
  V_Link::iterator linkEnd = links.end();
  for ( ; linkIt != linkEnd; ++linkIt )
  {
    robot_desc::URDF::Link* link = *linkIt;
    robot_desc::URDF::Link::Geometry::Mesh* mesh = static_cast<robot_desc::URDF::Link::Geometry::Mesh*>(link->visual->geometry->shape);
    if ( mesh->filename.empty() )
    {
      continue;
    }

    std::string modelName = mesh->filename + "_hi.mesh";

    Ogre::SceneNode* node = root_node_->createChildSceneNode();
    try
    {
      static int count = 0;
      std::stringstream ss;
      ss << "RobotModelVis" << count++;

      printf( "link name: %s\n", link->name.c_str() );

      Ogre::Entity* entity = scene_manager_->createEntity( ss.str(), modelName );
      models_[ link->name ] = entity;

      // assign the material from the link
      uint16_t numSubEntities = entity->getNumSubEntities();
      for ( uint16_t i = 0; i < numSubEntities; ++i )
      {
        Ogre::SubEntity* subEntity = entity->getSubEntity( i );

        typedef std::vector<std::string> V_string;
        V_string gazebo_names;
        link->visual->data.getMapTagNames("gazebo", gazebo_names);

        V_string::iterator nameIt = gazebo_names.begin();
        V_string::iterator nameEnd = gazebo_names.end();
        for ( ; nameIt != nameEnd; ++nameIt )
        {
          typedef std::map<std::string, std::string> M_string;
          M_string m = link->visual->data.getMapTagValues("gazebo", *nameIt);

          M_string::iterator it = m.find( "material" );
          if ( it != m.end() )
          {
            subEntity->setMaterialName( it->second );
          }
        }

      }

      node->attachObject( entity );
    }
    catch( Ogre::Exception& e )
    {
      printf( "Could not load model '%s' for link '%s': %s\n", modelName.c_str(), link->name.c_str(), e.what() );
    }
  }

  UpdateTransforms();
}

void RobotModelVisualizer::onEnable()
{
  if ( !initialized_ )
  {
    return;
  }

  subscribe();
  root_node_->setVisible( true );

  load();
}

void RobotModelVisualizer::onDisable()
{
  if ( !initialized_ )
  {
    return;
  }

  unsubscribe();
  root_node_->setVisible( false );
}

void RobotModelVisualizer::subscribe()
{
  if ( !isEnabled() )
  {
    return;
  }

  if ( initialized_ && !transform_topic_.empty() )
  {
    ros_node_->subscribe( transform_topic_, message_, &RobotModelVisualizer::incomingTransform, this, 5 );
  }
}

void RobotModelVisualizer::unsubscribe()
{
  if ( initialized_ && !transform_topic_.empty() )
  {
    ros_node_->unsubscribe( transform_topic_ );
  }
}

void RobotModelVisualizer::incomingTransform()
{
  has_new_transforms_ = true;
}

void RobotModelVisualizer::UpdateTransforms()
{
  M_StringToEntity::iterator modelIt = models_.begin();
  M_StringToEntity::iterator modelEnd = models_.end();
  for ( ; modelIt != modelEnd; ++modelIt )
  {
    const std::string& name = modelIt->first;
    Ogre::Entity* entity = modelIt->second;
    Ogre::SceneNode* node = entity->getParentSceneNode();

    libTF::TFPose pose = { 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0, "" };

    pose.frame = name;

    try
    {
      pose = tf_client_->transformPose( target_frame_, pose );
    }
    catch(libTF::TransformReference::LookupException& e)
    {
      printf( "Error transforming from frame '%s' to frame '%s': %s\n", name.c_str(), target_frame_.c_str(), e.what() );
    }
    catch(libTF::TransformReference::ConnectivityException& e)
    {
      printf( "Error transforming from frame '%s' to frame '%s': %s\n", name.c_str(), target_frame_.c_str(), e.what() );
    }
    catch(libTF::TransformReference::ExtrapolateException& e)
    {
      printf( "Error transforming from frame '%s' to frame '%s': %s\n", name.c_str(), target_frame_.c_str(), e.what() );
    }
    catch(libTF::TransformReference::MaxDepthException& e)
    {
      printf( "Error transforming from frame '%s' to frame '%s': %s\n", name.c_str(), target_frame_.c_str(), e.what() );
    }

    Ogre::Vector3 position( pose.x, pose.y, pose.z );
    robotToOgre( position );
    node->setPosition( position );

    node->setOrientation( ogreMatrixFromRobotEulers( pose.yaw, pose.pitch, pose.roll ) );
  }
}

void RobotModelVisualizer::update( float dt )
{
  message_.lock();

  if ( has_new_transforms_ )
  {
    UpdateTransforms();
    causeRender();

    has_new_transforms_ = false;
  }

  message_.unlock();
}

} // namespace ogre_vis

