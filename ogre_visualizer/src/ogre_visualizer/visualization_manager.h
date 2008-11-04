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


#ifndef OGRE_VISUALIZER_VISUALIZATION_MANAGER_H_
#define OGRE_VISUALIZER_VISUALIZATION_MANAGER_H_

#include <wx/event.h>
#include <wx/stopwatch.h>

#include <boost/signal.hpp>

#include <vector>
#include <map>

#include <rostools/Time.h>

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
class SceneNode;
class Camera;
class RaySceneQuery;
class ParticleSystem;
}

namespace ros
{
class node;
}

namespace tf
{
class TransformListener;
}

class wxTimerEvent;
class wxTimer;
class wxPropertyGrid;
class wxPropertyGridEvent;
class wxConfigBase;

namespace ogre_vis
{

class VisualizationPanel;
class PropertyManager;
class StringProperty;

class VisualizerBase;
class VisualizerFactory;

class Tool;

typedef boost::signal<void (VisualizerBase*)> VisualizerSignal;

class VisualizationManager : public wxEvtHandler
{
public:
  /**
   * \brief Constructor
   *
   * @param vis_panel The VisualizationPanel this manager is associated with
   * @return
   */
  VisualizationManager( VisualizationPanel* vis_panel );
  virtual ~VisualizationManager();

  void initialize();

  /**
   * \brief Create and then add a visualizer to this panel.
   * @param name Display name of the visualizer
   * @param enabled Whether to start enabled
   * @return A pointer to the new visualizer
   */
  template< class T >
  T* createVisualizer( const std::string& name, bool enabled, bool allow_deletion = false )
  {
    VisualizerBase* current_vis = getVisualizer( name );
    if ( current_vis )
    {
      return NULL;
    }

    T* visualizer = new T( name, this );
    addVisualizer( visualizer, allow_deletion, enabled );

    return visualizer;
  }

  /**
   * \brief Create and add a visualizer to this panel, by type name
   * @param type Type name of the visualizer
   * @param name Display name of the visualizer
   * @param enabled Whether to start enabled
   * @return A pointer to the new visualizer
   */
  VisualizerBase* createVisualizer( const std::string& type, const std::string& name, bool enabled, bool allow_deletion = false );

  /**
   * \brief Remove a visualizer
   * @param visualizer The visualizer to remove
   */
  void removeVisualizer( VisualizerBase* visualizer );
  /**
   * \brief Remove a visualizer by name
   * @param name The name of the visualizer to remove
   */
  void removeVisualizer( const std::string& name );

  template< class T >
  T* createTool( const std::string& name )
  {
    T* tool = new T( name, this );
    addTool( tool );

    return tool;
  }

  void addTool( Tool* tool );
  Tool* getCurrentTool() { return current_tool_; }
  Tool* getTool( int index );
  void setCurrentTool( Tool* tool );
  void setDefaultTool( Tool* tool );
  Tool* getDefaultTool() { return default_tool_; }

  /**
   * \brief Load configuration
   * @param config The wx config object to load from
   */
  void loadConfig( wxConfigBase* config );
  /**
   * \brief Save configuration
   * @param config The wx config object to save to
   */
  void saveConfig( wxConfigBase* config );

  /**
   * \brief Register a visualizer factory with the panel.  Allows you to create a visualizer by type.
   * @param type Type of the visualizer.  Must be unique.
   * @param factory The factory which will create this type of visualizer
   * @return Whether or not the registration succeeded.  The only failure condition is a non-unique type.
   */
  bool registerFactory( const std::string& type, VisualizerFactory* factory );

  /**
   * \brief Set the coordinate frame we should be displaying in
   * @param frame The string name -- must match the frame name broadcast to libTF
   */
  void setTargetFrame( const std::string& frame );
  const std::string& getTargetFrame() { return target_frame_; }

  /**
   * \brief Set the coordinate frame we should be transforming all fixed data to
   * @param frame The string name -- must match the frame name broadcast to libTF
   */
  void setFixedFrame( const std::string& frame );
  const std::string& getFixedFrame() { return fixed_frame_; }

  /**
   * \brief Performs a linear search to find a visualizer based on its name
   * @param name Name of the visualizer to search for
   */
  VisualizerBase* getVisualizer( const std::string& name );

  /**
   * \brief Enables/disables a visualizer.  Raises the signal retrieved through getVisualizerStateSignal()
   * @param visualizer The visualizer to act on
   * @param enabled Whether or not it should be enabled
   */
  void setVisualizerEnabled( VisualizerBase* visualizer, bool enabled );

  PropertyManager* getPropertyManager() { return property_manager_; }

  bool isDeletionAllowed( VisualizerBase* visualizer );
  bool isValidVisualizer( VisualizerBase* visualizer );

  ros::node* getROSNode() { return ros_node_; }
  tf::TransformListener* getTFClient() { return tf_; }
  Ogre::SceneManager* getSceneManager() { return scene_manager_; }

  void getRegisteredTypes( std::vector<std::string>& types );

  void pick( int mouse_x, int mouse_y );

  VisualizerSignal& getVisualizerStateSignal() { return visualizer_state_; }

  Ogre::SceneNode* getTargetRelativeNode() { return target_relative_node_; }

  VisualizationPanel* getVisualizationPanel() { return vis_panel_; }

  void resetVisualizers();

protected:
  /**
   * \brief Add a visualizer to be managed by this panel
   * @param visualizer The visualizer to be added
   */
  void addVisualizer( VisualizerBase* visualizer, bool allow_deletion, bool enabled );

  /**
   * \brief Performs a linear search to find a VisualizerInfo struct based on the visualizer contained inside it
   * @param visualizer The visualizer to find the info for
   */
  struct VisualizerInfo;
  VisualizerInfo* getVisualizerInfo( const VisualizerBase* visualizer );

  /// Called from the update timer
  void onUpdate( wxTimerEvent& event );

  void updateRelativeNode();

  void incomingROSTime();

  Ogre::Root* ogre_root_;                                 ///< Ogre Root
  Ogre::SceneManager* scene_manager_;                     ///< Ogre scene manager associated with this panel
  Ogre::RaySceneQuery* ray_scene_query_;                  ///< Used for querying the scene based on the mouse location
  Ogre::ParticleSystem* selection_bounds_particle_system_;

  wxTimer* update_timer_;                                 ///< Update timer.  VisualizerBase::update is called on each visualizer whenever this timer fires
  wxStopWatch update_stopwatch_;                          ///< Update stopwatch.  Stores how long it's been since the last update

  ros::node* ros_node_;                                   ///< Our ros::node
  tf::TransformListener* tf_;                             ///< Our rosTF client

  struct VisualizerInfo
  {
    VisualizerInfo()
    : visualizer_(NULL)
    , allow_deletion_(false)
    {}
    VisualizerBase* visualizer_;
    bool allow_deletion_;
  };
  typedef std::vector< VisualizerInfo > V_VisualizerInfo;
  V_VisualizerInfo visualizers_;                          ///< Our list of visualizers
  typedef std::map<std::string, VisualizerFactory*> M_Factory;
  M_Factory factories_;                                   ///< Factories by visualizer type name

  typedef std::vector< Tool* > V_Tool;
  V_Tool tools_;
  Tool* current_tool_;
  Tool* default_tool_;

  std::string target_frame_;                              ///< Target coordinate frame we're displaying everything in
  std::string fixed_frame_;                               ///< Frame to transform fixed data to

  PropertyManager* property_manager_;
  StringProperty* target_frame_property_;
  StringProperty* fixed_frame_property_;

  VisualizationPanel* vis_panel_;

  VisualizerSignal visualizer_state_;

  Ogre::SceneNode* target_relative_node_;

  rostools::Time time_message_;
  bool needs_reset_;
};

}

#endif /* OGRE_VISUALIZER_VISUALIZATION_MANAGER_H_ */
