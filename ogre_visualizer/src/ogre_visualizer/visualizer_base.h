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
#include <boost/function.hpp>

#include <wx/string.h>

namespace Ogre
{
class SceneManager;
class MovableObject;
}

namespace ros
{
class node;
}

class wxPanel;
class wxWindow;
class rosTFClient;
class wxPropertyGrid;
class wxPropertyGridEvent;
class wxPGProperty;
class wxConfigBase;
class wxString;

namespace ogre_vis
{

class PropertyManager;
class CategoryProperty;
class BoolProperty;

class VisualizationManager;

/**
 * \class VisualizerBase
 * \brief Abstract base class for all visualizers.
 *
 * Provides a common interface for the visualization panel to interact with,
 * so that new visualizers can be added without the visualization panel knowing anything about them.
 */
class VisualizerBase
{
public:
  VisualizerBase( const std::string& name, VisualizationManager* manager );
  virtual ~VisualizerBase();

  /**
   * \brief Enable this visualizer
   * @param force If false, does not re-enable if this visualizer is already enabled.  If true, it does.
   */
  void enable( bool force = false );
  /**
   * \brief Disable this visualizer
   * @param force If false, does not re-disable if this visualizer is already disabled.  If true, it does.
   */
  void disable( bool force = false );

  bool isEnabled() { return enabled_; }
  const std::string& getName() { return name_; }

  /**
   * \brief Called periodically by the visualization panel
   * @param dt Time, in seconds, since the last time the update list was run through.
   */
  virtual void update( float dt ) {}

  ///
  /**
   * \brief Set the callback used for causing a render to happen
   * @param func a void(void) function that will cause a render to happen from the correct thread
   */
  void setRenderCallback( boost::function<void ()> func );

  /// Set the callback used to lock the renderer
  void setLockRenderCallback( boost::function<void ()> func );
  /// Set the callback used to unlock the renderer
  void setUnlockRenderCallback( boost::function<void ()> func );

  /**
   * \brief Sets the property manager and parent category for this visualizer
   * @param manager The property manager
   * @param parent The parent category
   */
  void setPropertyManager( PropertyManager* manager, CategoryProperty* parent );

  /**
   * \brief Called from setPropertyManager, gives the visualizer a chance to create some properties immediately.
   *
   * Once this function is called, the property_manager_ member is valid and will stay valid
   */
  virtual void createProperties() {}

  /// Set the target frame of this visualizer. This is a frame id which should match something being broadcast through libTF.
  void setTargetFrame( const std::string& frame );

  /**
   * \brief Called from within setTargetFrame, notifying child classes that the target frame has changed
   */
  virtual void targetFrameChanged() = 0;

  /**
   * \brief Returns whether an object owned by this visualizer is pickable/mouse selectable
   * @param object The Ogre::MovableObject to check
   */
  virtual bool isObjectPickable( const Ogre::MovableObject* object ) const { return false; }

  /**
   * \brief Returns the type name of this visualizer.  Does not need to be exactly the same as the class name.  Can contains spaces/punctuation, etc.
   * @return The type name
   */
  virtual const char* getType() = 0;

protected:
  /// Derived classes override this to do the actual work of enabling themselves
  virtual void onEnable() = 0;
  /// Derived classes override this to do the actual work of disabling themselves
  virtual void onDisable() = 0;

  ///
  /**
   * \brief Cause the scene we're in to be rendered.
   * \note This does not immediately cause a render -- instead, one is queued and happens next run through the event loop.
   */
  void causeRender();

  /// Lock the renderer
  void lockRender();
  /// Unlock the renderer
  void unlockRender();

  VisualizationManager* vis_manager_;

  Ogre::SceneManager* scene_manager_;                 ///< The scene manager we're associated with
  std::string name_;                                  ///< The name of this visualizer
  bool enabled_;                                      ///< Are we enabled?

  std::string target_frame_;                          ///< The frame we should transform everything into

  boost::function<void ()> render_callback_;          ///< Render callback
  boost::function<void ()> render_lock_;              ///< Render lock callback
  boost::function<void ()> render_unlock_;            ///< Render unlock callback

  ros::node* ros_node_;                               ///< ros node
  rosTFClient* tf_client_;                            ///< rosTF client

  std::string property_prefix_;                       ///< Prefix to prepend to our properties

  PropertyManager* property_manager_;                 ///< The property manager to use to create properties
  CategoryProperty* parent_category_;                 ///< The parent category to use when creating properties
  BoolProperty* enabled_property_;

  friend class RenderAutoLock;
};

/**
 * \class RenderAutoLock
 * \brief A scoped lock on the renderer
 *
 * Constructor calls VisualizerBase::lockRender<br>
 * Destructor calls VisualizerBase::unlockRender
 */
class RenderAutoLock
{
public:
  RenderAutoLock( VisualizerBase* visualizer )
  : visualizer_( visualizer )
  {
    visualizer_->lockRender();
  }

  ~RenderAutoLock()
  {
    visualizer_->unlockRender();
  }

private:
  VisualizerBase* visualizer_;
};

} // namespace ogre_vis

#endif
