#ifndef __IL_POINT_CLOUD_HH
#define __IL_POINT_CLOUD_HH
///////////////////////////////////////////////////////////////////////////////
// 
// Copyright (C) 2008, Willow Garage Inc.
//
// Redistribution and use in source and binary forms, with or without 
// modification, are permitted provided that the following conditions are met:
//   * Redistributions of source code must retain the above copyright notice, 
//     this list of conditions and the following disclaimer.
//   * Redistributions in binary form must reproduce the above copyright 
//     notice, this list of conditions and the following disclaimer in the 
//     documentation and/or other materials provided with the distribution.
//   * Neither the name of Stanford University nor the names of its 
//     contributors may be used to endorse or promote products derived from 
//     this software without specific prior written permission.
//   
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" 
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE 
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE 
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE 
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR 
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF 
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS 
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN 
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) 
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE 
// POSSIBILITY OF SUCH DAMAGE.
//////////////////////////////////////////////////////////////////////////////

#include <irrlicht.h>
#include <iostream>
#include <math.h>

class ILPointCloud : public irr::scene::ISceneNode {
public:
  static const size_t MAX_RENDERABLE = 65535;

  ILPointCloud(irr::scene::ISceneNode* parent, irr::scene::ISceneManager* mgr, irr::s32 id);
  ~ILPointCloud();


  // Manipulators
  void addPoint(const double x, const double y, const double z, const int r, const int g, const int b);
  void addPoints(double *rgX, double *rgY, double *rgZ, int *rgR, int *rgG, int *rgB, const size_t numPoints);
  void resetCount();

  // Methods implemented for engine
  virtual void OnRegisterSceneNode();
  virtual void render();

  virtual const irr::core::aabbox3d<irr::f32>& getBoundingBox() const;
  virtual irr::u32 getMaterialCount();
  virtual irr::video::SMaterial& getMaterial(irr::u32 i);

private:
  // Memory management
  void preallocatePoints(const size_t numPoints);
  void deallocatePoints();


  size_t m_numPoints, m_numAllocPoints;
  irr::video::S3DVertex *m_points;

  irr::video::SMaterial m_material;
  irr::core::aabbox3d<irr::f32> m_box;
};

#endif // #ifndef __IL_POINT_CLOUD_HH
