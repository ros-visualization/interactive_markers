#ifndef __PP_IL_RENDER_HH
#define __PP_IL_RENDER_HH
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

// Feeder includes
#include <feeder/UCData.hh>
#include <feeder/PostProcessor.hh>
#include "UCDPointCloud.hh"

// Renderer includes
#include <irrlicht.h>
#include "ILClient.hh"
#include "ILRender.hh"
#include "CustomNodes/ILPointCloud.hh"
#include "CustomNodes/ILGrid.hh"

class PPILRender :
  public PostProcessor<UCDPointCloud, UCData>
{
public:
  typedef UCDPointCloud	input_t;
  typedef UCData	output_t;

  // Constructor
  PPILRender(PPInfo<input_t> p_info, int scans_to_plot) : PostProcessor<input_t, output_t>(p_info) {
    m_name = "PPILRender";
    m_pOutputBuffer = new UCRingBuffer<output_t>(8);
  };

  ~PPILRender() {
    disable();
  };

  void activeLoop() {
    input_t* p_ibuff;

    ILClient localClient;
    ILRender *pLocalRenderer = ILClient::getSingleton();

    pLocalRenderer->lock();
    ILPointCloud *ilCloud = new ILPointCloud(pLocalRenderer->manager()->getRootSceneNode(), pLocalRenderer->manager(), 666);
    ILGrid *ilGrid = new ILGrid(pLocalRenderer->manager()->getRootSceneNode(), pLocalRenderer->manager(), 667);
    pLocalRenderer->unlock();

    ilCloud->preallocatePoints(100000);
    pLocalRenderer->addNode(ilCloud);
    pLocalRenderer->enable(ilCloud);

    ilGrid->makegrid(100,1.0f,50,50,50);
    pLocalRenderer->addNode(ilGrid);
    pLocalRenderer->enable(ilGrid);

    while (m_enabled) {
      p_ibuff = m_rInputBuffer.getLatestAndRetain();	// Block for new data

      // Process data (this takes about 2500 usec for a 10Hz velodyne cycle)
      pLocalRenderer->lock();
      ilCloud->resetCount();
      for(size_t i=0; i<p_ibuff->num_points; i++) {
        ilCloud->addPoint(p_ibuff->rgX[i],
                          p_ibuff->rgZ[i],
                          p_ibuff->rgY[i],
                          (int)(50+205*(p_ibuff->rgZ[i]/3)),0,0);
      }
      pLocalRenderer->unlock();

      m_rInputBuffer.release(p_ibuff);
      incrementMonitor();
    }
  }
};

#endif // ifndef __PP_IL_RENDER_HH
