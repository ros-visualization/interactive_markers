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
#include <rosthread/member_thread.h>
#include <irrlicht.h>
#include "ILClient.hh"
#include "ILRender.hh"
#include "CustomNodes/ILPointCloud.hh"
#include "CustomNodes/ILGrid.hh"

#include <ros/node.h>
#include <std_msgs/PointCloudFloat32.h>
#include <std_msgs/Empty.h>

using namespace irr;
using namespace core;
using namespace scene;
using namespace video;
using namespace io;
using namespace gui;
using namespace std_msgs;

class Vis3d
{
	public:
	ros::node *myNode;
	std_msgs::Empty shutHead;
	std_msgs::Empty shutFloor;
	std_msgs::Empty shutStereo;
	std_msgs::PointCloudFloat32 ptCldHead;
	std_msgs::PointCloudFloat32 ptCldFloor;
	std_msgs::PointCloudFloat32 ptCldStereo;

	int headVertScanCount;
	ILClient *localClient;
	ILRender *pLocalRenderer;
	ILPointCloud *ilHeadCloud[cloudArrayLength];
	ILPointCloud *ilFloorCloud;
	ILPointCloud *ilStereoCloud;
	ILGrid *ilGrid;
	IAnimatedMesh *base;
	IAnimatedMeshSceneNode *baseNode;
	IAnimatedMesh *body;
	IAnimatedMeshSceneNode *bodyNode;
	IAnimatedMesh *wheel;
	IAnimatedMeshSceneNode *wheelFLNode;
	//IAnimatedMesh *wheelRL;
	IAnimatedMeshSceneNode *wheelRLNode;
	//IAnimatedMesh *wheelFR;
	IAnimatedMeshSceneNode *wheelFRNode;
	//IAnimatedMesh *wheelRR;
	IAnimatedMeshSceneNode *wheelRRNode;
	

	Vis3d(ros::node *aNode) : localClient(NULL)
	{
		for(int i = 0; i < cloudArrayLength; i++)
		{
			ilHeadCloud[i] = NULL;
			//delete ilHeadCloud[i];
		}
		ilFloorCloud = NULL;
		ilStereoCloud = NULL;
		ilGrid = NULL;
		
		base = NULL;
		baseNode = NULL;
		body = NULL;
		bodyNode = NULL;
		wheel = NULL;
		wheelFLNode = NULL;
		wheelRLNode = NULL;
		wheelFRNode = NULL;
		wheelRRNode = NULL;
		
		if (localClient)
		{
		      printf("deleting local client");
		      delete localClient;
		}
		
		for(int i = 0; i < cloudArrayLength; i++)
	    {
		    if(ilHeadCloud[i])
			    delete ilHeadCloud[i];
	    }
	    if(ilFloorCloud)
		    delete ilFloorCloud;
	    if(ilStereoCloud)
		    delete ilStereoCloud;
	    if(ilGrid)
		    delete ilGrid;
		
		myNode = aNode;

		printf("Creating new client\n");

		localClient = new ILClient();

		printf("Accessing Renderer\n");

		pLocalRenderer = ILClient::getSingleton();

		printf("Getting lock\n");

		pLocalRenderer->lock();

		headVertScanCount = 0;
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
		//disableModel();
		disableHead();
		disableStereo();
		disableFloor();
		delete localClient;
	    for(int i = 0; i < cloudArrayLength; i++)
	    {
		    //if(ilHeadCloud[i])
			delete ilHeadCloud[i];
	    }
	    //if(ilFloorCloud)
		delete ilFloorCloud;
	    //if(ilStereoCloud)
		delete ilStereoCloud;
	    //if(ilGrid)
		delete ilGrid;
		
		
	    std::cout << "destroyed Vis3D\n";
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
	    base = pLocalRenderer->manager()->getMesh("../pr2_models/base1000.3DS");
	    baseNode = pLocalRenderer->manager()->addAnimatedMeshSceneNode(base);
	    baseNode->setMaterialFlag(EMF_LIGHTING, false);
		baseNode->setMaterialFlag(EMF_WIREFRAME,true);
		
		body = pLocalRenderer->manager()->getMesh("../pr2_models/body1000.3DS");
	    bodyNode = pLocalRenderer->manager()->addAnimatedMeshSceneNode(body);
	    bodyNode->setMaterialFlag(EMF_LIGHTING, false);
		bodyNode->setMaterialFlag(EMF_WIREFRAME,true);
		
		wheel = pLocalRenderer->manager()->getMesh("../pr2_models/caster1000.3DS");
	    wheelFLNode = pLocalRenderer->manager()->addAnimatedMeshSceneNode(wheel);
	    wheelFLNode->setMaterialFlag(EMF_LIGHTING, false);
		wheelFLNode->setMaterialFlag(EMF_WIREFRAME,true);
		
		//wheelRL = pLocalRenderer->manager()->getMesh("../pr2_models/caster1000.3DS");
	    wheelRLNode = pLocalRenderer->manager()->addAnimatedMeshSceneNode(wheel);
	    wheelRLNode->setMaterialFlag(EMF_LIGHTING, false);
		wheelRLNode->setMaterialFlag(EMF_WIREFRAME,true);
		
		//wheelFR = pLocalRenderer->manager()->getMesh("../pr2_models/caster1000.3DS");
	    wheelFRNode = pLocalRenderer->manager()->addAnimatedMeshSceneNode(wheel);
	    wheelFRNode->setMaterialFlag(EMF_LIGHTING, false);
		wheelFRNode->setMaterialFlag(EMF_WIREFRAME,true);
		
		//wheelRR = pLocalRenderer->manager()->getMesh("../pr2_models/caster1000.3DS");
	    wheelRRNode = pLocalRenderer->manager()->addAnimatedMeshSceneNode(wheel);
	    wheelRRNode->setMaterialFlag(EMF_LIGHTING, false);
		wheelRRNode->setMaterialFlag(EMF_WIREFRAME,true);
	}

	void disableModel()
	{
		pLocalRenderer->lock();
	    baseNode->remove();
		bodyNode->remove();
		wheelFLNode->remove();
		wheelRLNode->remove();
		wheelFRNode->remove();
		wheelRRNode->remove();
		pLocalRenderer->unlock();
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
		pLocalRenderer->lock();
	    for(int i = 0; i < cloudArrayLength; i++)
	    {
		    
		pLocalRenderer->disable(ilHeadCloud[i]);
		    
	    }  
		pLocalRenderer->unlock();	    
	}

	void disableFloor()
	{
	    myNode->unsubscribe("cloudFloor");
	    myNode->unsubscribe("shutterFloor");
	    shutterFloor();
		pLocalRenderer->lock();
	    pLocalRenderer->disable(ilFloorCloud);
		pLocalRenderer->unlock();
	}

	void disableStereo()
	{
	    myNode->unsubscribe("cloudStereo");
	    myNode->unsubscribe("shutterStereo");
	    shutterStereo();
		pLocalRenderer->lock();
	    pLocalRenderer->disable(ilStereoCloud);
		pLocalRenderer->unlock();
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
    
