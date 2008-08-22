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
class wxPropertyGrid;
class wxPropertyGridEvent;

class rosTFClient;

namespace ogre_vis
{

class VisualizerBase;

class VisualizationPanel : public VisualizationPanelGenerated
{
public:
  VisualizationPanel( wxWindow* parent, Ogre::Root* root );
  virtual ~VisualizationPanel();

  void render();

  void lockRender() { render_mutex_.lock(); }
  void unlockRender() { render_mutex_.unlock(); }

  void addVisualizer( VisualizerBase* visualizer );

  template< class T >
  T* createVisualizer( const std::string& name, bool enabled )
  {
    T* visualizer = new T( scene_manager_, ros_node_, tf_client_, name, enabled );

    addVisualizer( visualizer );

    return visualizer;
  }

protected:
  void onViewClicked( wxCommandEvent& event );
  void onDisplayToggled( wxCommandEvent& event );
  void onDisplaySelected( wxCommandEvent& event );
  void onRenderWindowMouseEvents( wxMouseEvent& event );
  void onUpdate( wxTimerEvent& event );
  void onRender( wxCommandEvent& event );
  void onPropertyChanging( wxPropertyGridEvent& event );
  void onpropertyChanged( wxPropertyGridEvent& event );

  Ogre::Root* ogre_root_;
  Ogre::SceneManager* scene_manager_;

  wxTimer* update_timer_;
  wxStopWatch update_stopwatch_;

  wxPropertyGrid* property_grid_;

  ogre_tools::wxOgreRenderWindow* render_panel_;
  ogre_tools::CameraBase* current_camera_;
  ogre_tools::FPSCamera* fps_camera_;
  ogre_tools::OrbitCamera* orbit_camera_;

  ros::node* ros_node_;
  rosTFClient* tf_client_;

  typedef std::vector< VisualizerBase* > V_Visualizer;
  V_Visualizer visualizers_;
  VisualizerBase* selected_visualizer_;

  // Mouse handling
  bool left_mouse_down_;
  bool middle_mouse_down_;
  bool right_mouse_down_;
  int mouse_x_;
  int mouse_y_;

  ros::thread::mutex render_mutex_;
};

} // namespace ogre_vis

#endif
