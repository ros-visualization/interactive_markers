#ifndef __IL_GRID_HH
#define __IL_GRID_HH
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

class ILGrid : public irr::scene::ISceneNode {
public:
  static const size_t MAX_RENDERABLE = 65535;

  ILGrid(irr::scene::ISceneNode* parent, irr::scene::ISceneManager* mgr, irr::s32 id);
  ~ILGrid();

  // Manipulators
  void makegrid(const size_t gidsize, const double cellLength, int r, int g, int b);
  void deallocatePoints();

  // Methods implemented for engine
  virtual void OnRegisterSceneNode();
  virtual void render();

  virtual const irr::core::aabbox3d<irr::f32>& getBoundingBox() const;
  virtual irr::u32 getMaterialCount();
  virtual irr::video::SMaterial& getMaterial(irr::u32 i);

private:

  size_t m_gridSize;
  double m_cellLength;
  double m_extent;
  int m_r,m_g,m_b;

  irr::video::SMaterial m_material;
  irr::core::aabbox3d<irr::f32> m_box;
};

#endif // ifndef __IL_GRID_HH
