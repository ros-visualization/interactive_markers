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

#ifndef OGRE_VISUALIZER_VISUALIZATION_PANEL_H
#define OGRE_VISUALIZER_VISUALIZATION_PANEL_H

#include "generated/visualization_panel_generated.h"

#include "rosthread/mutex.h"

#include "wx/stopwatch.h"

#include <vector>

namespace ogre_tools
{
class wxOgreRenderWindow;
class FPSCamera;
class OrbitCamera;
class CameraBase;
}

namespace Ogre
{
class Root;
class SceneManager;
class Camera;
}

namespace ros
{
class node;
}

class wxTimerEvent;
class wxTimer;
class VisualizerBase;

class rosTFClient;

class VisualizationPanel : public VisualizationPanelGenerated
{
public:
  VisualizationPanel( wxWindow* parent, Ogre::Root* root );
  virtual ~VisualizationPanel();

  void Render();

  // HACK HACK HACK until single threaded ROS
  void LockRender() { m_RenderMutex.lock(); }
  void UnlockRender() { m_RenderMutex.unlock(); }

  void AddVisualizer( VisualizerBase* visualizer );

  template< class T >
  T* CreateVisualizer( const std::string& name, bool enabled )
  {
    T* visualizer = new T( m_SceneManager, m_ROSNode, m_TFClient, name, enabled );

    AddVisualizer( visualizer );

    return visualizer;
  }

protected:
  void OnViewClicked( wxCommandEvent& event );
  void OnDisplayToggled( wxCommandEvent& event );
  void OnDisplaySelected( wxCommandEvent& event );
  void OnRenderWindowMouseEvents( wxMouseEvent& event );
  void OnUpdate( wxTimerEvent& event );

  Ogre::Root* m_OgreRoot;
  Ogre::SceneManager* m_SceneManager;

  wxTimer* m_UpdateTimer;
  wxStopWatch m_UpdateStopwatch;

  ogre_tools::wxOgreRenderWindow* m_RenderPanel;
  ogre_tools::CameraBase* m_CurrentCamera;
  ogre_tools::FPSCamera* m_FPSCamera;
  ogre_tools::OrbitCamera* m_OrbitCamera;

  ros::node* m_ROSNode;
  rosTFClient* m_TFClient;

  typedef std::vector< VisualizerBase* > V_Visualizer;
  V_Visualizer m_Visualizers;
  wxPanel* m_CurrentOptionsPanel;

  // Mouse handling
  bool m_LeftMouseDown;
  bool m_MiddleMouseDown;
  bool m_RightMouseDown;
  int m_MouseX;
  int m_MouseY;

  // HACK HACK HACK until single threaded ROS
  ros::thread::mutex m_RenderMutex;
};

#endif
