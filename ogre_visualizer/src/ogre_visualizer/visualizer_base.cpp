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

#include "visualizer_base.h"

#include "ros/common.h"

namespace ogre_vis
{

VisualizerBase::VisualizerBase( Ogre::SceneManager* sceneManager, ros::node* node, rosTFClient* tfClient,
                                const std::string& name, bool enabled )
: m_SceneManager( sceneManager )
, m_Name( name )
, m_Enabled( enabled )
, m_TargetFrame( "base" )
, m_RenderCallback( NULL )
, m_RenderLock( NULL )
, m_RenderUnlock( NULL )
, m_ROSNode( node )
, m_TFClient( tfClient )
{
}

VisualizerBase::~VisualizerBase()
{
  delete m_RenderCallback;
}

void VisualizerBase::Enable()
{
  if ( m_Enabled )
  {
    return;
  }

  m_Enabled = true;

  OnEnable();
}

void VisualizerBase::Disable()
{
  if ( !m_Enabled )
  {
    return;
  }

  m_Enabled = false;

  OnDisable();
}

void VisualizerBase::SetRenderCallback( abstractFunctor* func )
{
  delete m_RenderCallback;

  m_RenderCallback = func;
}

void VisualizerBase::SetLockRenderCallback( abstractFunctor* func )
{
  delete m_RenderLock;

  m_RenderLock = func;
}

void VisualizerBase::SetUnlockRenderCallback( abstractFunctor* func )
{
  delete m_RenderUnlock;

  m_RenderUnlock = func;
}


void VisualizerBase::CauseRender()
{
  if ( m_RenderCallback )
  {
    m_RenderCallback->call();
  }
}

void VisualizerBase::LockRender()
{
  m_RenderLock->call();
}

void VisualizerBase::UnlockRender()
{
  m_RenderUnlock->call();
}

} // namespace ogre_vis
