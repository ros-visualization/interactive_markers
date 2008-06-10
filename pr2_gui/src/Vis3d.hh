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

#ifndef __PP_IL_RENDER_HH
#define __PP_IL_RENDER_HH
#define cloudArrayLength 400
#include "rosthread/member_thread.h"
#include <irrlicht.h>
#include "ILClient.hh"
#include "ILRender.hh"
#include "CustomNodes/ILPointCloud.hh"
#include "CustomNodes/ILGrid.hh"

#include "ros/node.h"
#include "std_msgs/MsgPointCloudFloat32.h"
#include "std_msgs/MsgEmpty.h"

using namespace irr;
using namespace core;
using namespace scene;
using namespace video;
using namespace io;
using namespace gui;

class Vis3d
{
	public:
	ros::node *myNode;
	MsgEmpty shutHead;
	MsgEmpty shutFloor;
	MsgEmpty shutStereo;
	MsgPointCloudFloat32 ptCldHead;
	MsgPointCloudFloat32 ptCldFloor;
	MsgPointCloudFloat32 ptCldStereo;

	int headVertScanCount;
	ILClient localClient;
	ILRender *pLocalRenderer;
	ILPointCloud *ilHeadCloud[cloudArrayLength];
	ILPointCloud *ilFloorCloud;
	ILPointCloud *ilStereoCloud;
	ILGrid *ilGrid;
	IAnimatedMesh *base;
	IAnimatedMeshSceneNode *baseNode;

	Vis3d(ros::node *aNode) : localClient()
	{
		myNode = aNode;
	    std::cerr<<"Constructing Vis3D" << std::endl;
	    headVertScanCount = 0;
	    pLocalRenderer = ILClient::getSingleton();
	    std::cerr<<"Got singleton" <<std::endl;
	    pLocalRenderer->lock();
	    ilFloorCloud = new ILPointCloud(pLocalRenderer->manager()->getRootSceneNode(),pLocalRenderer->manager(),667);
	    ilStereoCloud = new ILPointCloud(pLocalRenderer->manager()->getRootSceneNode(),pLocalRenderer->manager(),666);
	    ilGrid = new ILGrid(pLocalRenderer->manager()->getRootSceneNode(), pLocalRenderer->manager(), 668);
	    for(int i = 0; i < cloudArrayLength; i++)
	    {
		ilHeadCloud[i] = new ILPointCloud(pLocalRenderer->manager()->getRootSceneNode(),pLocalRenderer->manager(),669 + i);
	    }
	    pLocalRenderer->unlock();
	    pLocalRenderer->addNode(ilFloorCloud);
	    pLocalRenderer->addNode(ilStereoCloud);
	    for(int i = 0; i < cloudArrayLength; i++)
	    {
		pLocalRenderer->addNode(ilHeadCloud[i]);
	    }
	    ilGrid->makegrid(100,1.0f,50,50,50);
	    pLocalRenderer->addNode(ilGrid);
	    std::cerr<<"Done Constructing Vis3D"<<std::endl;
	    //take me out
	    //enableBody();
	}

	~Vis3d()
	{
	    std::cout << "destroying Vis3D\n";
	    pLocalRenderer->manager()->clear();
	    //delete pLocalRenderer->manager();
	    for(int i = 0; i < cloudArrayLength; i++)
	    {
		delete ilHeadCloud[i];
	    }
	    delete ilFloorCloud;
	    delete ilStereoCloud;
	    delete ilGrid;
	    delete base;
	    delete baseNode;
	    //delete localClient;
	    //delete pLocalRenderer;
	    std::cout << "destroyed Vis3D\n";
	    //ros::fini();
	}

	bool isEnabled()
	{
	    return pLocalRenderer->isEnabled();
	}

	void enableHead()
	{
	    myNode->subscribe("cloud", ptCldHead, &Vis3d::addHeadCloud,this);
	    myNode->subscribe("shutter", shutHead, &Vis3d::shutterHead,this);
	    for(int i = 0; i < cloudArrayLength; i++)
	    {
		pLocalRenderer->enable(ilHeadCloud[i]);
	    }   
	}

	void enableModel()
	{
	    base = pLocalRenderer->manager()->getMesh("../pr2_models/base.3DS");
	    baseNode = pLocalRenderer->manager()->addAnimatedMeshSceneNode(base);
	    baseNode->setMaterialFlag(EMF_LIGHTING, false);
	}

	void disableModel()
	{
	    baseNode->remove();
	}

	void enableFloor()
	{
	    myNode->subscribe("cloudFloor", ptCldFloor, &Vis3d::addFloorCloud,this);
	    myNode->subscribe("shutterFloor", shutFloor, &Vis3d::shutterFloor,this);
	    pLocalRenderer->enable(ilFloorCloud);
	}

	void enableStereo()
	{
	    myNode->subscribe("cloudStereo", ptCldStereo, &Vis3d::addStereoCloud,this);
	    myNode->subscribe("shutterStereo", shutStereo, &Vis3d::shutterStereo,this);
	    pLocalRenderer->enable(ilStereoCloud);
	}

	void disableHead()
	{
	    myNode->unsubscribe("cloud");
	    myNode->unsubscribe("shutter");
	    shutterHead();
	    for(int i = 0; i < cloudArrayLength; i++)
	    {
		pLocalRenderer->disable(ilHeadCloud[i]);
	    }   
	}

	void disableFloor()
	{
	    myNode->unsubscribe("cloudFloor");
	    myNode->unsubscribe("shutterFloor");
	    shutterFloor();
	    pLocalRenderer->disable(ilFloorCloud);
	}

	void disableStereo()
	{
	    myNode->unsubscribe("cloudStereo");
	    myNode->unsubscribe("shutterStereo");
	    shutterStereo();
	    pLocalRenderer->disable(ilStereoCloud);
	}

	void shutterHead()
	{
	    pLocalRenderer->lock();
	    for(int i = 0; i < cloudArrayLength; i++)
	    {
		ilHeadCloud[i]->resetCount();
	    }
	    pLocalRenderer->unlock();
	    headVertScanCount = 0;
	}

	void shutterFloor()
	{
	    pLocalRenderer->lock();
	    ilFloorCloud->resetCount();
	    pLocalRenderer->unlock();
	}

	void shutterStereo()
	{
	    pLocalRenderer->lock();
	    ilStereoCloud->resetCount();
	    pLocalRenderer->unlock();
	}

	void addHeadCloud()
	{
	    pLocalRenderer->lock();
	    if(headVertScanCount < cloudArrayLength)
	    {
		if(ptCldHead.get_pts_size() > 65535)
		{
		    for(int i = 0; i < 65535; i++)
		    {
			ilHeadCloud[headVertScanCount]->addPoint(-ptCldHead.pts[i].y, ptCldHead.pts[i].z, ptCldHead.pts[i].x, 255 ,(int)(ptCldHead.chan[0].vals[i]/16.0),(int)(ptCldHead.chan[0].vals[i]/16.0));
		    }
		}
		else
		{
		    for(size_t i = 0; i < ptCldHead.get_pts_size(); i++)
		    {
			ilHeadCloud[headVertScanCount]->addPoint(-ptCldHead.pts[i].y, ptCldHead.pts[i].z, ptCldHead.pts[i].x, 255,(int)(ptCldHead.chan[0].vals[i]/16.0),(int)(ptCldHead.chan[0].vals[i]/16.0));
		    }
		}
		headVertScanCount++;
	    }
	    pLocalRenderer->unlock();
	}

	void addFloorCloud()
	{
	    pLocalRenderer->lock();
	    if(ptCldFloor.get_pts_size() > 65535)
	    {
		for(int i = 0; i < 65535; i++)
		{
		    ilFloorCloud->addPoint(-ptCldFloor.pts[i].y, ptCldFloor.pts[i].z, ptCldFloor.pts[i].x, (int)(ptCldFloor.chan[0].vals[i]/16.0),255,(int)(ptCldFloor.chan[0].vals[i]/16.0));
		}
	    }
	    else
	    {
		for(size_t i = 0; i < ptCldFloor.get_pts_size(); i++)
		{
		    ilFloorCloud->addPoint(-ptCldFloor.pts[i].y, ptCldFloor.pts[i].z, ptCldFloor.pts[i].x, (int)(ptCldFloor.chan[0].vals[i]/16.0),255,(int)(ptCldFloor.chan[0].vals[i]/16.0));
		}
	    }
	    pLocalRenderer->unlock();
	}

	void addStereoCloud()
	{
	    pLocalRenderer->lock();
	    if(ptCldStereo.get_pts_size() > 65535)
	    {
		for(int i = 0; i < 65535; i++)
		{
		    ilStereoCloud->addPoint(-ptCldStereo.pts[i].y, ptCldStereo.pts[i].z, ptCldStereo.pts[i].x, (int)(ptCldStereo.chan[0].vals[i]/16.0),(int)(ptCldStereo.chan[0].vals[i]/16.0),255);
		}
	    }
	    else
	    {
		for(size_t i = 0; i < ptCldStereo.get_pts_size(); i++)
		{
		    ilStereoCloud->addPoint(-ptCldStereo.pts[i].y, ptCldStereo.pts[i].z, ptCldStereo.pts[i].x, (int)(ptCldStereo.chan[0].vals[i]/16.0),(int)(ptCldStereo.chan[0].vals[i]/16.0),255);
		}
	    }
	    pLocalRenderer->unlock();
	}
};
    
    
#endif // ifndef __PP_IL_RENDER_HH
    
