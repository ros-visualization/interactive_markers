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

#ifndef OGRE_VISUALIZER_ROBOT_MODEL_VISUALIZER_H
#define OGRE_VISUALIZER_ROBOT_MODEL_VISUALIZER_H

#include "../visualizer_base.h"
#include "std_msgs/Empty.h"

#include <map>

namespace Ogre
{
class Entity;
class SceneNode;
}

namespace ogre_vis
{

class RobotModelVisualizer : public VisualizerBase
{
public:
  RobotModelVisualizer( Ogre::SceneManager* sceneManager, ros::node* node, rosTFClient* tfClient, const std::string& name, bool enabled );
  virtual ~RobotModelVisualizer();

  void Initialize( const std::string& descriptionParam, const std::string& transformTopic );

  virtual void Update( float dt );

protected:

  // overrides from VisualizerBase
  virtual void OnEnable();
  virtual void OnDisable();

  void Subscribe();
  void Unsubscribe();

  void IncomingTransform();
  void UpdateTransforms();

  std_msgs::Empty m_Message;
  std::string m_TransformTopic;

  typedef std::map< std::string, Ogre::Entity* > M_StringToEntity;
  M_StringToEntity m_Models;

  Ogre::SceneNode* m_RootNode;
  bool m_HasNewTransforms;

  bool m_Initialized;
};

} // namespace ogre_vis

 #endif

