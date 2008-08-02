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

#ifndef OGRE_VISUALIZER_VISUALIZER_BASE
#define OGRE_VISUALIZER_VISUALIZER_BASE

#include <string>

namespace Ogre
{
class SceneManager;
}

namespace ros
{
class node;
}

class wxPanel;
class wxWindow;
class abstractFunctor;
class rosTFClient;

namespace ogre_vis
{

/** Abstract base class for all visualizers.  This provides a common interface for the visualization panel to interact with,
 * so that new visualizers can be added without the visualization panel knowing anything about them.
 * */
class VisualizerBase
{
public:
  VisualizerBase( Ogre::SceneManager* sceneManager, ros::node* node, rosTFClient* tfClient, const std::string& name, bool enabled = false );
  virtual ~VisualizerBase();

  /// Enable this visualizer
  void Enable();
  /// Disable this visualizer
  void Disable();

  bool IsEnabled() { return m_Enabled; }
  const std::string& GetName() { return m_Name; }

  /// Called periodically by the visualization panel
  virtual void Update( float dt ) {}

  /// Called by the visualization panel to tell set our functor used for causing a render to happen
  void SetRenderCallback( abstractFunctor* func );

  // HACK HACK HACK until single threaded ROS arrives
  void SetLockRenderCallback( abstractFunctor* func );
  void SetUnlockRenderCallback( abstractFunctor* func );
  void LockRender();
  void UnlockRender();

  /// Override this to provide an options panel for this visualization.  The panel is owned by the caller, so it must handle any cleanup.
  virtual wxPanel* GetOptionsPanel( wxWindow* parent ) { return NULL; } // default to no options

  void SetTargetFrame( const std::string& frame ) { m_TargetFrame = frame; }

protected:
  /// Derived classes override this to do the actual work of enabling themselves
  virtual void OnEnable() = 0;
  /// Derived classes override this to do the actual work of disabling themselves
  virtual void OnDisable() = 0;

  /// Called by derived classes to cause the scene we're in to be rendered.
  void CauseRender();

  Ogre::SceneManager* m_SceneManager;
  std::string m_Name;
  bool m_Enabled;

  std::string m_TargetFrame;

  abstractFunctor* m_RenderCallback;
  abstractFunctor* m_RenderLock;
  abstractFunctor* m_RenderUnlock;

  ros::node* m_ROSNode;
  rosTFClient* m_TFClient;
};

} // namespace ogre_vis

#endif
