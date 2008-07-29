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

#include "ILLaserScan.hh"

#define MIN(a,b) (((a)<(b))?(a):(b))

using namespace irr;

ILLaserScan::ILLaserScan(irr::scene::ISceneNode* parent, irr::scene::ISceneManager* mgr, irr::s32 id) : scene::ISceneNode(parent, mgr, id), m_numPoints (0), m_numAllocPoints (0), m_points (NULL)
{
  m_material.Lighting = false;
  m_material.Wireframe = false;
  m_material.PointCloud = true;
  m_material.BackfaceCulling = false;
  m_material.Thickness = 2;
  
  m_points.push_back(new video::S3DVertex);
  preallocatePoints(1024);
}

ILLaserScan::~ILLaserScan() {
  deallocatePoints();
}

// Memory management
void ILLaserScan::preallocatePoints(const size_t numPoints) {
  if(m_numAllocPoints == 0) {
    m_points[m_numPoints/MAX_RENDERABLE] = (video::S3DVertex*)malloc(numPoints*sizeof(video::S3DVertex));
    m_numAllocPoints = numPoints;
  }
}

void ILLaserScan::deallocatePoints() {
  if(m_numAllocPoints > 0 || m_points.size() > 0) {
    for( int i = 0; i < m_points.size(); i++)
    {
		free(m_points[i]);
		
    }
    m_points.clear();
    m_numAllocPoints = 0;
    m_numPoints = 0;
  }
}

// Manipulators
void ILLaserScan::addPoint(const double x, const double y, const double z, const int r, const int g, const int b)
{
  if(m_numPoints/m_points.size() == MAX_RENDERABLE)
    {
      m_points.push_back(new video::S3DVertex);
      m_numAllocPoints = 0;
      preallocatePoints(1024);
    }
  if(m_numPoints - (m_points.size() - 1) * MAX_RENDERABLE == m_numAllocPoints) {
    m_numAllocPoints = MIN(2 * m_numAllocPoints, MAX_RENDERABLE);
    m_points[m_numPoints/MAX_RENDERABLE] = (video::S3DVertex*)realloc(m_points[m_numPoints/MAX_RENDERABLE],m_numAllocPoints*sizeof(video::S3DVertex));
  }
  m_points[m_numPoints/MAX_RENDERABLE][m_numPoints % MAX_RENDERABLE].Pos.set(-y,z,x);
  m_points[m_numPoints/MAX_RENDERABLE][m_numPoints % MAX_RENDERABLE].Color.set(255,r,g,b);
  m_numPoints++;
}

void ILLaserScan::addPoints(double *rgX, double *rgY, double *rgZ, int *rgR, int *rgG, int *rgB, const size_t numPoints)
{
  for(int i=0; i<numPoints; i++) {
    addPoint(-1*rgY[i],1*rgZ[i],1*rgX[i],1*rgR[i],1*rgG[i],1*rgB[i]);
  }
}

void ILLaserScan::resetCount() {
  m_numPoints = 0;
}

void ILLaserScan::OnRegisterSceneNode() {
  if (IsVisible) {
    SceneManager->registerNodeForRendering(this);
  }

  ISceneNode::OnRegisterSceneNode();
}

void ILLaserScan::render() {
  video::IVideoDriver* driver = SceneManager->getVideoDriver();

  driver->setMaterial(m_material);
  driver->setTransform(video::ETS_WORLD, AbsoluteTransformation);

  for(size_t i=0; i<m_numPoints; i+=MAX_RENDERABLE) {
    driver->drawVertexPrimitiveList(m_points[i/MAX_RENDERABLE],MIN(m_numPoints-i, MAX_RENDERABLE), NULL, MIN(m_numPoints-i,MAX_RENDERABLE), video::EVT_STANDARD, scene::EPT_POINTS);
  }
}

const irr::core::aabbox3d<irr::f32>& ILLaserScan::getBoundingBox() const {
  return m_box;
}

irr::u32 ILLaserScan::getMaterialCount() {
  return 1;
}

irr::video::SMaterial& ILLaserScan::getMaterial(irr::u32 i) {
  return m_material;
}	
