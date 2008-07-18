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

#include "ILGrid.hh"

#define MIN(a,b) (((a)<(b))?(a):(b))

using namespace irr;

ILGrid::ILGrid(irr::scene::ISceneNode* parent, irr::scene::ISceneManager* mgr, irr::s32 id) : scene::ISceneNode(parent, mgr, id)
{
  m_material.Lighting = false;
  m_material.Wireframe = true;
  m_material.BackfaceCulling = false;
  m_material.Thickness = 1;
}

ILGrid::~ILGrid() {
  deallocatePoints();
}

// Manipulators
void ILGrid::makegrid(const size_t gridSize, const double cellLength, int r, int g, int b) {
  m_gridSize = gridSize;
  m_cellLength = cellLength;
  m_extent = (cellLength*((double)gridSize))/2;
  m_r = r;
  m_g = g;
  m_b = b;
}

void ILGrid::deallocatePoints() {
}

void ILGrid::OnRegisterSceneNode() {
  if (IsVisible) {
    SceneManager->registerNodeForRendering(this);
  }

  ISceneNode::OnRegisterSceneNode();
}

void ILGrid::render() {
  video::IVideoDriver* driver = SceneManager->getVideoDriver();

  driver->setMaterial(m_material);
  driver->setTransform(video::ETS_WORLD, AbsoluteTransformation);

  double inc;
	//driver->draw3DLine(irr::core::vector3d<irr::f32>(0,0,0),irr::core::vector3d<irr::f32>(0,0,1),irr::video::SColor(255,255,0,0));
	//driver->draw3DLine(irr::core::vector3d<irr::f32>(0,0,0),irr::core::vector3d<irr::f32>(-1,0,0),irr::video::SColor(255,0,255,0));
	//driver->draw3DLine(irr::core::vector3d<irr::f32>(0,0,0),irr::core::vector3d<irr::f32>(0,1,0),irr::video::SColor(255,0,0,255));
  for(size_t i=0; i<=m_gridSize; i++) {
    inc = m_extent-i*m_cellLength;

    driver->draw3DLine(core::vector3df(inc,0,-1*m_extent), core::vector3df(inc,0,m_extent), video::SColor(255,m_r,m_g,m_b));
    driver->draw3DLine(core::vector3df(-1*m_extent,0,inc), core::vector3df(m_extent,0,inc), video::SColor(255,m_r,m_g,m_b));
    //driver->draw3DLine(core::vector3df(0,inc,-1*m_extent), core::vector3df(0,inc,m_extent), video::SColor(255,m_r,m_g,m_b));
    //driver->draw3DLine(core::vector3df(0,-1*m_extent,inc), core::vector3df(0,m_extent,inc), video::SColor(255,m_r,m_g,m_b));
  }
}

const irr::core::aabbox3d<irr::f32>& ILGrid::getBoundingBox() const {
  return m_box;
}

irr::u32 ILGrid::getMaterialCount() {
  return 1;
}

irr::video::SMaterial& ILGrid::getMaterial(irr::u32 i) {
  return m_material;
}	
