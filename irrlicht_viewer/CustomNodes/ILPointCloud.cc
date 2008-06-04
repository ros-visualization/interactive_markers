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

#include "ILPointCloud.hh"

#define MIN(a,b) (((a)<(b))?(a):(b))

using namespace irr;

ILPointCloud::ILPointCloud(irr::scene::ISceneNode* parent, irr::scene::ISceneManager* mgr, irr::s32 id) : scene::ISceneNode(parent, mgr, id), m_numPoints (0), m_numAllocPoints (0), m_points (NULL)
{
  m_material.Lighting = false;
  m_material.Wireframe = false;
  m_material.PointCloud = true;
  m_material.BackfaceCulling = false;
  m_material.Thickness = 2;

  preallocatePoints(1024);
}

ILPointCloud::~ILPointCloud() {
  deallocatePoints();
}

// Memory management
void ILPointCloud::preallocatePoints(const size_t numPoints) {
  if(m_numAllocPoints == 0) {
    m_points = (video::S3DVertex*)malloc(numPoints*sizeof(video::S3DVertex));
    m_numAllocPoints = numPoints;
  }
}

void ILPointCloud::deallocatePoints() {
  if(m_numAllocPoints > 0) {
    free(m_points);
    m_points = NULL;
    m_numAllocPoints = 0;
    m_numPoints = 0;
  }
}

// Manipulators
void ILPointCloud::addPoint(const double x, const double y, const double z, const int r, const int g, const int b)
{
  if(m_numPoints >= MAX_RENDERABLE)
    {
      //std::cerr<<"Overfilling"<< std::endl;
      return;
    }
  if(m_numPoints == m_numAllocPoints) {
    //std::cerr<<"Reallocating"<<std::endl;
    m_numAllocPoints = MIN(2 * m_numAllocPoints, MAX_RENDERABLE);
    m_points = (video::S3DVertex*)realloc(m_points,m_numAllocPoints*sizeof(video::S3DVertex));
  }

  m_points[m_numPoints].Pos.set(x,y,z);
  m_points[m_numPoints].Color.set(255,r,g,b);

  m_numPoints++;
}

void ILPointCloud::addPoints(double *rgX, double *rgY, double *rgZ, int *rgR, int *rgG, int *rgB, const size_t numPoints)
{
  if(m_numPoints+numPoints >= m_numAllocPoints) {
    m_numAllocPoints = MIN ( m_numPoints+numPoints, MAX_RENDERABLE);
    m_points = (video::S3DVertex*)realloc(m_points,m_numAllocPoints * sizeof(video::S3DVertex));
  }

  for(int i=0; i<numPoints; i++) {
    m_points[m_numPoints + i].Pos.set(rgX[i],rgY[i],rgZ[i]);
    m_points[m_numPoints + i].Color.set(255,rgR[i],rgG[i],rgB[i]);
  }
  
  m_numPoints += numPoints;
}

void ILPointCloud::resetCount() {
  m_numPoints = 0;
}

void ILPointCloud::OnRegisterSceneNode() {
  if (IsVisible) {
    SceneManager->registerNodeForRendering(this);
  }

  ISceneNode::OnRegisterSceneNode();
}

void ILPointCloud::render() {
  video::IVideoDriver* driver = SceneManager->getVideoDriver();

  driver->setMaterial(m_material);
  driver->setTransform(video::ETS_WORLD, AbsoluteTransformation);

  for(size_t i=0; i<m_numPoints; i+=MAX_RENDERABLE) {
    driver->drawVertexPrimitiveList(m_points,MIN(m_numPoints-i, MAX_RENDERABLE), NULL, MIN(m_numPoints-i,MAX_RENDERABLE), video::EVT_STANDARD, scene::EPT_POINTS);
  }
}

const irr::core::aabbox3d<irr::f32>& ILPointCloud::getBoundingBox() const {
  return m_box;
}

irr::u32 ILPointCloud::getMaterialCount() {
  return 1;
}

irr::video::SMaterial& ILPointCloud::getMaterial(irr::u32 i) {
  return m_material;
}	
