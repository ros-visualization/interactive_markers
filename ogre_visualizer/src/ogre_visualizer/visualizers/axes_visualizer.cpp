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

#include "axes_visualizer.h"
#include "../common.h"

#include "ogre_tools/axes.h"

#include <Ogre.h>

namespace ogre_vis
{

AxesVisualizer::AxesVisualizer( Ogre::SceneManager* sceneManager, ros::node* node, rosTFClient* tfClient, const std::string& name, bool enabled )
: VisualizerBase( sceneManager, node, tfClient, name, enabled )
, m_Length( 1.0 )
, m_Radius( 0.1 )
{
  m_Axes = new ogre_tools::Axes( sceneManager, NULL, m_Length, m_Radius );

  m_Axes->GetSceneNode()->setVisible( IsEnabled() );

  Ogre::Quaternion orient( Ogre::Quaternion::IDENTITY );
  RobotToOgre( orient );
  m_Axes->SetOrientation( orient );
}

AxesVisualizer::~AxesVisualizer()
{
}

void AxesVisualizer::OnEnable()
{
  m_Axes->GetSceneNode()->setVisible( true );
}

void AxesVisualizer::OnDisable()
{
  m_Axes->GetSceneNode()->setVisible( false );
}

void AxesVisualizer::Create()
{
  m_Axes->Set( m_Length, m_Radius );

  CauseRender();
}

void AxesVisualizer::Set( float length, float radius )
{
  m_Length = length;
  m_Radius = radius;

  Create();
}

} // namespace ogre_vis
