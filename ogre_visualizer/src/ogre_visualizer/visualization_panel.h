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

/**
 * @mainpage
 *
 * @htmlinclude manifest.html
 *
 * @b ogre_visualizer is a 3D visualization framework that is embeddable anywhere, as a wxPanel
 *
 * @section vispanelusage VisualizationPanel Usage
 * The visualization panel needs an initialized Ogre::Root before it can be created.  For an example of this, see the visualizer_test.cpp test application.
 *
 * Once Ogre::Root is initialized, create a VisualizationPanel with:
 @verbatim
 ogre_tools::VisualizationPanel* visualization_panel = new VisualizationPanel( <parent wxWindow>, <Ogre::Root*> );
 @endverbatim
 * You can then add any number of visualizers:
 @verbatim
 visualization_panel->createVisualizer<AxesVisualizer>( "Origin Axes", true );

 pointCloud = visualization_panel->createVisualizer<PointCloudVisualizer>( "Head Full Cloud", false );
 pointCloud->setTopic( "full_cloud" );
 pointCloud->setColor( 1.0, 1.0, 0.0 );
 ...
 @endverbatim
 *
 * Writing your own visualizer is easy.  For a simple example, see http://pr.willowgarage.com/wiki/ogre_visualizer/SimpleVisualizerTutorial
 */

#include "generated/visualization_panel_generated.h"

#include "rosthread/mutex.h"

#include "wx/stopwatch.h"

#include <vector>
#include <map>

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
class RaySceneQuery;
class ParticleSystem;
}

namespace ros
{
class node;
}

class wxTimerEvent;
class wxTimer;
class wxPropertyGrid;
class wxPropertyGridEvent;
class wxConfigBase;

class rosTFClient;

namespace ogre_vis
{

class VisualizerBase;
class VisualizationManager;

/**
 * \class VisualizationPanel
 * \brief A self-contained wxPanel for 3D visualization of pretty much anything.
 *
 */
class VisualizationPanel : public VisualizationPanelGenerated
{
public:
  /**
   * \brief Constructor
   *
   * @param parent Parent window
   * @return
   */
  VisualizationPanel( wxWindow* parent );
  virtual ~VisualizationPanel();

  /**
   * \brief Queues a render.  Multiple calls before a render happens will only cause a single render.
   * \note This function can be called from any thread.
   */
  void queueRender();

  /**
   * \brief Locks the renderer
   */
  void lockRender() { render_mutex_.lock(); }
  /**
   * \brief Unlocks the renderer
   */
  void unlockRender() { render_mutex_.unlock(); }

  ogre_tools::wxOgreRenderWindow* getRenderPanel() { return render_panel_; }
  wxPropertyGrid* getPropertyGrid() { return property_grid_; }
  VisualizationManager* getManager() { return manager_; }
  ogre_tools::CameraBase* getCurrentCamera() { return current_camera_; }

  /**
   * \brief Load configuration.  Simply passes through to the VisualizationManager, here for convenience
   * @param config The wx config object to load from
   */
  void loadConfig( wxConfigBase* config );
  /**
   * \brief Save configuration.  Simply passes through to the VisualizationManager, here for convenience
   * @param config The wx config object to save to
   */
  void saveConfig( wxConfigBase* config );

protected:
  /// Called when a "view" (camera) is selected from the list
  void onViewClicked( wxCommandEvent& event );
  /// Called when a mouse event happens inside the render window
  void onRenderWindowMouseEvents( wxMouseEvent& event );
  /// Called when our custom EVT_RENDER is fired
  void onRender( wxCommandEvent& event );
  /// Called when a property from the wxPropertyGrid is changing
  void onPropertyChanging( wxPropertyGridEvent& event );
  /// Called when a property from the wxProperty
  void onPropertyChanged( wxPropertyGridEvent& event );
  /// Called when a property is selected
  void onPropertySelected( wxPropertyGridEvent& event );

  /// Called when the "New Display" button is pressed
  virtual void onNewDisplay( wxCommandEvent& event );
  /// Called when the "Delete Display" button is pressed
  virtual void onDeleteDisplay( wxCommandEvent& event );

  void onVisualizerStateChanged( VisualizerBase* visualizer );

  wxPropertyGrid* property_grid_;                         ///< Visualizer property grid

  ogre_tools::wxOgreRenderWindow* render_panel_;          ///< Render window

  ogre_tools::CameraBase* current_camera_;                ///< The current camera
  ogre_tools::FPSCamera* fps_camera_;                     ///< FPS camera
  ogre_tools::OrbitCamera* orbit_camera_;                 ///< Orbit camera

  // Mouse handling
  bool left_mouse_down_;                                  ///< Is the left mouse button down?
  bool middle_mouse_down_;                                ///< Is the middle mouse button down?
  bool right_mouse_down_;                                 ///< Is the right mouse button down?
  int mouse_x_;                                           ///< X position of the last mouse event
  int mouse_y_;                                           ///< Y position of the last mouse event

  std::string target_frame_;                              ///< Target coordinate frame we're displaying everything in

  ros::thread::mutex render_mutex_;                       ///< Render mutex

  VisualizationManager* manager_;
};

} // namespace ogre_vis

#endif
