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

#ifndef OGRE_TOOLS_OGRE_POINT_CLOUD_H
#define OGRE_TOOLS_OGRE_POINT_CLOUD_H

#include <OgreMovableObject.h>
#include <OgreString.h>
#include <OgreAxisAlignedBox.h>

#include <stdint.h>

#include <vector>

namespace Ogre
{
class SceneManager;
class ManualObject;
class SceneNode;
class BillboardSet;
class RenderQueue;
class Camera;
}

namespace ogre_tools
{

class PointCloud : public Ogre::MovableObject
{
public:
  PointCloud( Ogre::SceneManager* manager );
  ~PointCloud();

  /// Clear this point cloud
  void Clear();

  struct Point
  {
    Point() {}
    Point( float x, float y, float z, float r, float g, float b ) : m_X( x ), m_Y( y ), m_Z( z ), r_( r ), g_( g ), b_( b ) {}
    float m_X;
    float m_Y;
    float m_Z;
    float r_;
    float g_;
    float b_;
  };

  /// Add points to this point cloud
  void AddPoints( Point* points, uint32_t numPoints );

  void SetVisible( bool visible );
  void SetUsePoints( bool usePoints );
  void SetBillboardDimensions( float width, float height );

  // overrides from MovableObject
  virtual const Ogre::String& getMovableType() const { return sm_Type; }
  virtual const Ogre::AxisAlignedBox& getBoundingBox() const;
  virtual float getBoundingRadius() const;
  virtual void _updateRenderQueue( Ogre::RenderQueue* queue );
  virtual void _notifyCurrentCamera( Ogre::Camera* camera );

private:
  Ogre::BillboardSet* CreateBillboardSet();

  Ogre::SceneManager* scene_manager_;
  Ogre::SceneNode* scene_node_;
  Ogre::AxisAlignedBox bounding_box_;
  float bounding_radius_;

  typedef std::vector<Ogre::BillboardSet*> V_BillboardSet;
  V_BillboardSet billboard_sets_;

  typedef std::vector<Point> V_Point;
  V_Point points_;
  uint32_t point_count_;
  uint32_t points_per_bbs_;

  bool use_points_;
  float billboard_width_;
  float billboard_height_;

  static Ogre::String sm_Type;
};

} // namespace ogre_tools

#endif
