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

#include "rosthread/member_thread.h"

// Renderer includes
#include <irrlicht.h>
#include "ILClient.hh"
#include "ILRender.hh"
#include "CustomNodes/ILPointCloud.hh"
#include "CustomNodes/ILGrid.hh"

#include "ros/node.h"
#include "std_msgs/MsgPointCloudFloat32.h"
#include "std_msgs/MsgEmpty.h"

class PPILRender :
  public ros::node
{
public:

  MsgPointCloudFloat32 cloudIn;
  MsgEmpty shutter;

  ILClient localClient;
  ILRender *pLocalRenderer;
  ILPointCloud *ilCloud;
  ILPointCloud *ilCloud1;
  ILPointCloud *ilCloud2;
  ILPointCloud *ilCloud3;
  ILGrid *ilGrid;
  
  // Constructor
  PPILRender() : ros::node("ILRender"), localClient() {

    pLocalRenderer = ILClient::getSingleton();

    pLocalRenderer->lock();
    ilCloud = new ILPointCloud(pLocalRenderer->manager()->getRootSceneNode(), pLocalRenderer->manager(), 666);
    ilCloud1 = new ILPointCloud(pLocalRenderer->manager()->getRootSceneNode(), pLocalRenderer->manager(), 668);
    ilCloud2 = new ILPointCloud(pLocalRenderer->manager()->getRootSceneNode(), pLocalRenderer->manager(), 669);
    ilCloud3 = new ILPointCloud(pLocalRenderer->manager()->getRootSceneNode(), pLocalRenderer->manager(), 670);
    ilGrid = new ILGrid(pLocalRenderer->manager()->getRootSceneNode(), pLocalRenderer->manager(), 667);
    pLocalRenderer->unlock();

    pLocalRenderer->addNode(ilCloud);
    pLocalRenderer->enable(ilCloud);
    pLocalRenderer->addNode(ilCloud1);
    pLocalRenderer->enable(ilCloud1);
    pLocalRenderer->addNode(ilCloud2);
    pLocalRenderer->enable(ilCloud2);
    pLocalRenderer->addNode(ilCloud3);
    pLocalRenderer->enable(ilCloud3);

    ilGrid->makegrid(100,1.0f,50,50,50);
    pLocalRenderer->addNode(ilGrid);
    pLocalRenderer->enable(ilGrid);

    subscribe("cloud", cloudIn, &PPILRender::callback, this);
    subscribe("shutter", shutter, &PPILRender::clear, this);


  };
  
  ~PPILRender() {
    delete ilCloud;
    delete ilGrid;
  };

  bool isEnabled(){return pLocalRenderer->isEnabled();};

  void clear()
  {
    std::cerr<< "Shutter"<<std::endl;
    pLocalRenderer->lock();
    ilCloud->resetCount();
    ilCloud1->resetCount();
    ilCloud2->resetCount();
    ilCloud3->resetCount();
    pLocalRenderer->unlock();
  }

  void callback() {
    //std::cerr<<"recieved cloud"<<std::endl;
    // Process data (this takes about 2500 usec for a 10Hz velodyne cycle)
    pLocalRenderer->lock();
    //    unsigned int temp = ilCloud->getCount();
    //if (temp + cloudIn.get_pts_size() > ILPointCloud::MAX_RENDERABLE)
    // {
    //	std::cerr<< "Point Cloud Overflow, clearing" << std::endl;
    //	ilCloud->resetCount();
    // }

    //    std::cerr<<"Points: " << cloudIn.get_pts_size() << std::endl;
    for(size_t i=0; i < cloudIn.get_pts_size(); i ++) { //FIXME protect form cloud overruns 65k
      //      usleep(10);
      //std::cerr<<"i = " << i << std::endl;
      if (i%4 == 0)
	ilCloud->addPoint(-cloudIn.pts[i].y,
			  cloudIn.pts[i].z,
			  cloudIn.pts[i].x,
			  (int)(cloudIn.pts[i].z * 32.0)+64,(int)(cloudIn.chan[0].vals[i]/16.0),128);
      else if ((i + 1)%4 == 0)
	ilCloud1->addPoint(-cloudIn.pts[i].y,
			   cloudIn.pts[i].z,
			   cloudIn.pts[i].x,
			   (int)(cloudIn.pts[i].z * 32.0)+64,(int)(cloudIn.chan[0].vals[i]/16.0),128);
      else if ((i + 2)%4 == 0)
	ilCloud2->addPoint(-cloudIn.pts[i].y,
			   cloudIn.pts[i].z,
			   cloudIn.pts[i].x,
			   (int)(cloudIn.pts[i].z * 32.0)+64,(int)(cloudIn.chan[0].vals[i]/16.0),128);
      else 
	ilCloud3->addPoint(-cloudIn.pts[i].y,
			   cloudIn.pts[i].z,
			   cloudIn.pts[i].x,
			   (int)(cloudIn.pts[i].z * 32.0)+64,(int)(cloudIn.chan[0].vals[i]/16.0),128);
    }
    pLocalRenderer->unlock();
  };
    
};

#endif // ifndef __PP_IL_RENDER_HH
