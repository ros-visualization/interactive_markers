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

/**
 * \class PointCloud
 * \brief A visual representation of a set of points.
 *
 * Displays a set of points using any number of Ogre BillboardSets.  PointCloud is optimized for sets of points that change
 * rapidly, rather than for large clouds that never change.
 *
 * Most of the functions in PointCloud are not safe to call from any thread but the render thread.  Exceptions are clear() and addPoints(), which
 * are safe as long as we are not in the middle of a render (ie. Ogre::Root::renderOneFrame, or Ogre::RenderWindow::update)
 */
class PointCloud : public Ogre::MovableObject
{
public:
  PointCloud( Ogre::SceneManager* manager, Ogre::SceneNode* parent = NULL );
  ~PointCloud();

  /**
   * \brief Clear all the points
   */
  void clear();

  /**
   * \struct Point
   * \brief Representation of a point, with x/y/z position and r/g/b color
   */
  struct Point
  {
    Point() {}
    Point( float x, float y, float z, float r, float g, float b ) : x_( x ), y_( y ), z_( z ), r_( r ), g_( g ), b_( b ) {}
    float x_;
    float y_;
    float z_;
    float r_;
    float g_;
    float b_;
  };

  /**
   * \brief Add points to this point cloud
   *
   * @param points An array of Point structures
   * @param num_points The number of points in the array
   */
  void addPoints( Point* points, uint32_t num_points );

  /**
   * \brief Set whether to use points for rendering rather than billboards
   * @param usePoints If true, will use point rendering instead of billboards
   */
  void setUsePoints( bool usePoints );
  /**
   * \brief Set the dimensions of the billboards used to render each point
   * @param width Width of the billboards
   * @param height Height of the billboards
   * @note Only applicable if point rendering is off
   */
  void setBillboardDimensions( float width, float height );

  // overrides from MovableObject
  virtual const Ogre::String& getMovableType() const { return sm_Type; }
  virtual const Ogre::AxisAlignedBox& getBoundingBox() const;
  virtual float getBoundingRadius() const;
  virtual void getWorldTransforms( Ogre::Matrix4* xform ) const;
  virtual void _updateRenderQueue( Ogre::RenderQueue* queue );
  virtual void _notifyCurrentCamera( Ogre::Camera* camera );

private:
  /**
   * Creates an Ogre BillboardSet with the correct settings
   * @return A BillboardSet
   */
  Ogre::BillboardSet* createBillboardSet();

  Ogre::SceneManager* scene_manager_;       ///< The scene manager this point cloud is associated with
  Ogre::SceneNode* scene_node_;             ///< The scene node this point cloud is attached to
  Ogre::AxisAlignedBox bounding_box_;       ///< The bounding box of this point cloud
  float bounding_radius_;                   ///< The bounding radius of this point cloud

  typedef std::vector<Ogre::BillboardSet*> V_BillboardSet;
  V_BillboardSet billboard_sets_;           ///< The billboard sets we've allocated

  typedef std::vector<Point> V_Point;
  V_Point points_;                          ///< The list of points we're displaying.  Allocates to a high-water-mark.
  uint32_t point_count_;                    ///< The number of points currently in #points_
  uint32_t points_per_bbs_;                 ///< The number of points we can display per BillboardSet.  Changes based on the rendering style (point vs. billboard)

  bool use_points_;                         ///< Are we rendering as points?
  float billboard_width_;                   ///< Billboard width
  float billboard_height_;                  ///< Billboard height

  static Ogre::String sm_Type;              ///< The "renderable type" used by Ogre
};

} // namespace ogre_tools

#endif
