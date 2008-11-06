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

#ifndef OGRE_VISUALIZER_MARKER_VISUALIZER_H
#define OGRE_VISUALIZER_MARKER_VISUALIZER_H

#include "visualizer_base.h"

#include <map>

#include <std_msgs/VisualizationMarker.h>

namespace Ogre
{
class SceneManager;
class SceneNode;
}

namespace ros
{
class node;
}

namespace ogre_tools
{
class Object;
}

namespace robot_desc
{
class URDF;
}

namespace planning_models
{
class KinematicModel;
}

namespace ogre_vis
{

namespace MarkerTypes
{
enum MarkerType
{
  Arrow,
  Cube,
  Sphere,
  Robot,
};
}
typedef MarkerTypes::MarkerType MarkerType;

namespace MarkerActions
{
enum MarkerAction
{
  Add,
  Modify,
  Delete,
};
}
typedef MarkerActions::MarkerAction MarkerAction;

/**
 * \class MarkerVisualizer
 * \brief Displays "markers" sent in by other ROS nodes on the "visualizationMarker" topic
 *
 * Markers come in as std_msgs::VisualizationMarker messages.  See the VisualizationMarker message for more information.
 */
class MarkerVisualizer : public VisualizerBase
{
public:
  MarkerVisualizer( const std::string& name, VisualizationManager* manager );
  virtual ~MarkerVisualizer();

  virtual void update( float dt );

  virtual bool isObjectPickable( const Ogre::MovableObject* object ) const { return true; }

  virtual void targetFrameChanged() {}
  virtual void fixedFrameChanged();
  virtual void reset();

  static const char* getTypeStatic() { return "Markers"; }
  virtual const char* getType() { return getTypeStatic(); }
  static const char* getDescription();

protected:
  virtual void onEnable();
  virtual void onDisable();

  /**
   * \brief Subscribes to the "visualizationMarker" topic
   */
  void subscribe();
  /**
   * \brief Unsubscribes from the "visualizationMarker" topic
   */
  void unsubscribe();

  /**
   * \brief Removes all the markers
   */
  void clearMarkers();

  /**
   * \brief Processes a marker message
   * @param message The message to process
   */
  void processMessage( const std_msgs::VisualizationMarker& message );
  /**
   * \brief Processes an "Add" marker message
   * @param message The message to process
   */
  void processAdd( const std_msgs::VisualizationMarker& message );
  /**
   * \brief Processes a "Modify" marker message
   * @param message The message to process
   */
  void processModify( const std_msgs::VisualizationMarker& message );
  /**
   * \brief Processes a "Delete" marker message
   * @param message The message to process
   */
  void processDelete( const std_msgs::VisualizationMarker& message );
  /**
   * \brief Set common values (position, orientation, scale, color) on a marker's object
   * @param message The message to get the values from
   * @param object The object to set the values on
   */
  void setCommonValues( const std_msgs::VisualizationMarker& message, ogre_tools::Object* object );

  /**
   * \brief ROS callback notifying us of a new marker
   */
  void incomingMarker();

  struct MarkerInfo
  {
    MarkerInfo( ogre_tools::Object* object, const std_msgs::VisualizationMarker& message )
    : object_(object)
    , message_(message)
    {}
    ogre_tools::Object* object_;
    std_msgs::VisualizationMarker message_;
  };

  typedef std::map<int, MarkerInfo> M_IDToMarker;
  M_IDToMarker markers_;                                ///< Map of marker id to the marker info structure

  std_msgs::VisualizationMarker current_message_;       ///< Incoming marker message

  typedef std::vector< std_msgs::VisualizationMarker > V_MarkerMessage;
  V_MarkerMessage message_queue_;                       ///< Marker message queue.  Messages are added to this as they are received, and then processed
                                                        ///< in our update() function

  Ogre::SceneNode* scene_node_;                         ///< Scene node all the marker objects are parented to

  robot_desc::URDF* urdf_;
  planning_models::KinematicModel* kinematic_model_;
};

} // namespace ogre_vis

#endif /* OGRE_VISUALIZER_MARKER_VISUALIZER_H */
