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

RobotModelVisualizer::RobotModelVisualizer( Ogre::SceneManager* sceneManager, ros::node* node, rosTFClient* tfClient, const std::string& name, bool enabled )
: VisualizerBase( sceneManager, node, tfClient, name, enabled )
, m_HasNewTransforms( false )
, m_Initialized( false )
{
  m_RootNode = sceneManager->getRootSceneNode()->createChildSceneNode();
}

RobotModelVisualizer::~RobotModelVisualizer()
{
  Unsubscribe();

  Clear();

  m_SceneManager->destroySceneNode( m_RootNode->getName() );
}

void RobotModelVisualizer::Clear()
{
  M_StringToEntity::iterator modelIt = m_Models.begin();
  M_StringToEntity::iterator modelEnd = m_Models.end();
  for ( ; modelIt != modelEnd; ++modelIt )
  {
    Ogre::Entity* entity = modelIt->second;
    Ogre::SceneNode* node = entity->getParentSceneNode();

    node->detachObject( entity );
    m_SceneManager->destroyEntity( entity );
  }

  m_Models.clear();

  m_RootNode->removeAndDestroyAllChildren();
}

void RobotModelVisualizer::Initialize( const std::string& descriptionParam, const std::string& transformTopic )
{
  m_TransformTopic = transformTopic;
  m_DescriptionParam = descriptionParam;

  m_Initialized = true;

  if ( IsEnabled() )
  {
    OnEnable();
    CauseRender();
  }
  else
  {
    m_RootNode->setVisible( false );
  }
}

void RobotModelVisualizer::Load()
{
  Clear();

  std::string content;
  m_ROSNode->get_param(m_DescriptionParam, content);
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

    Ogre::SceneNode* node = m_RootNode->createChildSceneNode();
    try
    {
      static int count = 0;
      std::stringstream ss;
      ss << "RobotModelVis" << count++;

      printf( "link name: %s\n", link->name.c_str() );

      Ogre::Entity* entity = m_SceneManager->createEntity( ss.str(), modelName );
      m_Models[ link->name ] = entity;

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

void RobotModelVisualizer::OnEnable()
{
  if ( !m_Initialized )
  {
    return;
  }

  Subscribe();
  m_RootNode->setVisible( true );

  Load();
}

void RobotModelVisualizer::OnDisable()
{
  if ( !m_Initialized )
  {
    return;
  }

  Unsubscribe();
  m_RootNode->setVisible( false );
}

void RobotModelVisualizer::Subscribe()
{
  if ( !IsEnabled() )
  {
    return;
  }

  if ( m_Initialized && !m_TransformTopic.empty() )
  {
    m_ROSNode->subscribe( m_TransformTopic, m_Message, &RobotModelVisualizer::IncomingTransform, this );
  }
}

void RobotModelVisualizer::Unsubscribe()
{
  if ( m_Initialized && !m_TransformTopic.empty() )
  {
    m_ROSNode->unsubscribe( m_TransformTopic );
  }
}

void RobotModelVisualizer::IncomingTransform()
{
  m_HasNewTransforms = true;
}

void RobotModelVisualizer::UpdateTransforms()
{
  M_StringToEntity::iterator modelIt = m_Models.begin();
  M_StringToEntity::iterator modelEnd = m_Models.end();
  for ( ; modelIt != modelEnd; ++modelIt )
  {
    const std::string& name = modelIt->first;
    Ogre::Entity* entity = modelIt->second;
    Ogre::SceneNode* node = entity->getParentSceneNode();

    libTF::TFPose pose = { 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0, "" };

    pose.frame = name;

    try
    {
      pose = m_TFClient->transformPose( m_TargetFrame, pose );
    }
    catch(libTF::TransformReference::LookupException& e)
    {
      printf( "Error transforming from frame '%s' to frame '%s': %s\n", name.c_str(), m_TargetFrame.c_str(), e.what() );
    }
    catch(libTF::TransformReference::ConnectivityException& e)
    {
      printf( "Error transforming from frame '%s' to frame '%s': %s\n", name.c_str(), m_TargetFrame.c_str(), e.what() );
    }
    catch(libTF::TransformReference::ExtrapolateException& e)
    {
      printf( "Error transforming from frame '%s' to frame '%s': %s\n", name.c_str(), m_TargetFrame.c_str(), e.what() );
    }

    Ogre::Vector3 position( pose.x, pose.y, pose.z );
    RobotToOgre( position );
    node->setPosition( position );

    node->setOrientation( OgreMatrixFromRobotEulers( pose.yaw, pose.pitch, pose.roll ) );
  }
}

void RobotModelVisualizer::Update( float dt )
{
  m_Message.lock();

  if ( m_HasNewTransforms )
  {
    UpdateTransforms();
    CauseRender();

    m_HasNewTransforms = false;
  }

  m_Message.unlock();
}

} // namespace ogre_vis

