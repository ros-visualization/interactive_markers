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

#ifndef OGRE_VISUALIZER_MAP_VISUALIZER_H
#define OGRE_VISUALIZER_MAP_VISUALIZER_H

#include "visualizer_base.h"

#include <OgreTexture.h>
#include <OgreMaterial.h>

namespace Ogre
{
class SceneNode;
class ManualObject;
}

namespace ogre_vis
{

class StringProperty;
class FloatProperty;

/**
 * \class MapVisualizer
 * \brief Displays a map along the XZ plane (XY in robot space)
 *
 */
class MapVisualizer : public VisualizerBase
{
public:
  MapVisualizer( const std::string& name, VisualizationManager* manager );
  virtual ~MapVisualizer();

  void setService( const std::string& service );
  const std::string& getService() { return service_; }

  float getResolution() { return resolution_; }
  float getWidth() { return width_; }
  float getHeight() { return height_; }

  // Overrides from VisualizerBase
  virtual void targetFrameChanged() {}
  virtual void fixedFrameChanged();
  virtual void createProperties();
  virtual void update( float dt );
  virtual void reset();

  static const char* getTypeStatic() { return "Map"; }
  virtual const char* getType() { return getTypeStatic(); }

protected:
  // overrides from VisualizerBase
  virtual void onEnable();
  virtual void onDisable();

  void clear();
  void load();
  void transformMap();

  Ogre::SceneNode* scene_node_;
  Ogre::ManualObject* manual_object_;
  Ogre::TexturePtr texture_;
  Ogre::MaterialPtr material_;
  bool loaded_;

  std::string service_;
  float resolution_;
  float width_;
  float height_;

  StringProperty* service_property_;
  FloatProperty* resolution_property_;
  FloatProperty* width_property_;
  FloatProperty* height_property_;
};

} // namespace ogre_vis

 #endif
