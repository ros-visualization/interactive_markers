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

#include <stdint.h>
 
#include <vector>

namespace Ogre
{
class SceneManager;

class ManualObject;
class SceneNode;
}

namespace ogre_tools
{

class PointCloud
{
public:
  PointCloud( Ogre::SceneManager* manager );
  ~PointCloud();
  
  /// Clear this point cloud
  void Clear();
  
  /// Commit any points added to this cloud. Must be called after adding points, to actually update the ogre ManualObject
  void Commit();

  /// Add a single point to this point cloud.  Must call Commit() for this to take effect
  void AddPoint( float x, float y, float z, float r, float g, float b );

  struct Point
  {
    Point() {}
    Point( float x, float y, float z, float r, float g, float b ) : m_X( x ), m_Y( y ), m_Z( z ), m_R( r ), m_G( g ), m_B( b ) {}
    float m_X;
    float m_Y;
    float m_Z;
    float m_R;
    float m_G;
    float m_B;
  };

  /// Add points to this point cloud.  Must call Commit() for this to take effect
  void AddPoints( Point* points, uint32_t numPoints );

private:
  Ogre::SceneManager* m_SceneManager;
  Ogre::SceneNode* m_SceneNode;
  Ogre::ManualObject* m_ManualObject;
  
  typedef std::vector<Point> V_Point;
  V_Point m_ScratchPoints;
};

} // namespace ogre_tools

#endif
