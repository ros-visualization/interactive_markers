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

namespace ogre_vis
{

VisualizerBase::VisualizerBase( Ogre::SceneManager* scene_manager, ros::node* node, rosTFClient* tf_client,
                                const std::string& name, bool enabled )
: scene_manager_( scene_manager )
, name_( name )
, enabled_( enabled )
, target_frame_( "base" )
, ros_node_( node )
, tf_client_( tf_client )
{
}

VisualizerBase::~VisualizerBase()
{
}

void VisualizerBase::enable()
{
  if ( enabled_ )
  {
    return;
  }

  enabled_ = true;

  onEnable();
}

void VisualizerBase::disable()
{
  if ( !enabled_ )
  {
    return;
  }

  enabled_ = false;

  onDisable();
}

void VisualizerBase::setRenderCallback( boost::function<void ()> func )
{
  render_callback_ = func;
}

void VisualizerBase::setLockRenderCallback( boost::function<void ()> func )
{
  render_lock_ = func;
}

void VisualizerBase::setUnlockRenderCallback( boost::function<void ()> func )
{
  render_unlock_ = func;
}


void VisualizerBase::causeRender()
{
  if ( render_callback_ )
  {
    render_callback_();
  }
}

void VisualizerBase::lockRender()
{
  if ( render_lock_ )
  {
    render_lock_();
  }
}

void VisualizerBase::unlockRender()
{
  if ( render_unlock_ )
  {
    render_unlock_();
  }
}

} // namespace ogre_vis
