#include "Vis3d.h"

/**Initializes all Irrlicht models, cameras, and lights */
Vis3d::Vis3d(ros::node *aNode) : tfClient(*aNode)
{
	objectsVisibility = true;
	myNode = aNode;
	headVertScanCount = 0;
	scanDir = -1;
	scanT = Wipe;
	
	localClient = new ILClient();
	pLocalRenderer = ILClient::getSingleton();
	pLocalRenderer->lock();

	ilFloorCloud = new ILLaserScan(pLocalRenderer->manager()->getRootSceneNode(),pLocalRenderer->manager(),200);
	ilStereoCloud = new ILPointCloud(pLocalRenderer->manager()->getRootSceneNode(),pLocalRenderer->manager(),201);
	ilGrid = new ILGrid(pLocalRenderer->manager()->getRootSceneNode(), pLocalRenderer->manager(), 202);
	ilucs = new ILUCS(pLocalRenderer->manager()->getRootSceneNode(), pLocalRenderer->manager(),true, 203);
	for(int i = 0; i < cloudArrayLength; i++)
	{
		ilHeadCloud[i] = new ILPointCloud(pLocalRenderer->manager()->getRootSceneNode(),pLocalRenderer->manager(),206 + i);
	}
	pLocalRenderer->unlock();
	pLocalRenderer->addNode(ilFloorCloud);
	pLocalRenderer->addNode(ilStereoCloud);
	for(int i = 0; i < cloudArrayLength; i++)
	{
		pLocalRenderer->addNode(ilHeadCloud[i]);
	}
	ilGrid->makegrid(100,.1f,50,50,50);
	pLocalRenderer->addNode(ilGrid);
	irr::SKeyMap keyMap[8];
	{
             keyMap[0].Action = irr::EKA_MOVE_FORWARD;
             keyMap[0].KeyCode = irr::KEY_UP;
             keyMap[1].Action = irr::EKA_MOVE_FORWARD;
             keyMap[1].KeyCode = irr::KEY_KEY_W;

             keyMap[2].Action = irr::EKA_MOVE_BACKWARD;
             keyMap[2].KeyCode = irr::KEY_DOWN;
             keyMap[3].Action = irr::EKA_MOVE_BACKWARD;
             keyMap[3].KeyCode = irr::KEY_KEY_S;

             keyMap[4].Action = irr::EKA_STRAFE_LEFT;
             keyMap[4].KeyCode = irr::KEY_LEFT;
             keyMap[5].Action = irr::EKA_STRAFE_LEFT;
             keyMap[5].KeyCode = irr::KEY_KEY_A;

             keyMap[6].Action = irr::EKA_STRAFE_RIGHT;
             keyMap[6].KeyCode = irr::KEY_RIGHT;
             keyMap[7].Action = irr::EKA_STRAFE_RIGHT;
             keyMap[7].KeyCode = irr::KEY_KEY_D;
	}
	cameras[Maya] = pLocalRenderer->manager()->addCameraSceneNodeMaya(NULL,-150.0f,50.0f,25.0f,Maya);
	cameras[FPS] = pLocalRenderer->manager()->addCameraSceneNodeFPS(NULL,50,5,FPS,keyMap,8);
	cameras[TFL] = pLocalRenderer->manager()->addCameraSceneNode(NULL,irr::core::vector3df(-3,3,3),irr::core::vector3df(-1,1,1),TFL);
	cameras[TFR] = pLocalRenderer->manager()->addCameraSceneNode(NULL,irr::core::vector3df(3,3,3),irr::core::vector3df(1,1,1),TFR);
	cameras[TRL] = pLocalRenderer->manager()->addCameraSceneNode(NULL,irr::core::vector3df(-3,3,-3),irr::core::vector3df(-1,1,-1),TRL);
	cameras[TRR] = pLocalRenderer->manager()->addCameraSceneNode(NULL,irr::core::vector3df(3,3,-3),irr::core::vector3df(1,1,-1),TRR);
	cameras[Front] = pLocalRenderer->manager()->addCameraSceneNode(NULL,irr::core::vector3df(0,1,3),irr::core::vector3df(0,1,0),Front);
	cameras[Rear] = pLocalRenderer->manager()->addCameraSceneNode(NULL,irr::core::vector3df(0,1,-3),irr::core::vector3df(0,1,0),Rear);
	cameras[Top] = pLocalRenderer->manager()->addCameraSceneNode(NULL,irr::core::vector3df(0,3,0),irr::core::vector3df(0,0,0),Top);
	cameras[Bottom] = pLocalRenderer->manager()->addCameraSceneNode(NULL,irr::core::vector3df(0,-3,0),irr::core::vector3df(0,0,0),Bottom);
	cameras[Left] = pLocalRenderer->manager()->addCameraSceneNode(NULL,irr::core::vector3df(-3,1,0),irr::core::vector3df(0,1,0),Left);
	cameras[Right] = pLocalRenderer->manager()->addCameraSceneNode(NULL,irr::core::vector3df(3,1,0),irr::core::vector3df(0,1,0),Right);
	pLocalRenderer->manager()->setActiveCamera(cameras[Rear]);
	changeView(Maya);
	
	irr::video::SColorf ambColor(0.2f,0.2f,0.2f,1.0f);
	pLocalRenderer->manager()->setAmbientLight(ambColor);
	light[0] = pLocalRenderer->manager()->addLightSceneNode(NULL,irr::core::vector3df(50,50,50),irr::video::SColorf(.9f,.9f,.9f,1.0f));
	//light[1] = pLocalRenderer->manager()->addLightSceneNode(NULL,irr::core::vector3df(-50,-50,-50),irr::video::SColorf(.5f,.5f,.5f,1.0f));
	
	intermediate = pLocalRenderer->manager()->addCubeSceneNode(0.0);
	intermediate->setRotation(irr::core::vector3df(90, 90, 0));
	intermediate->setScale(irr::core::vector3df(1, 1, 1));
	
	ArrowMesh = pLocalRenderer->manager()->addArrowMesh("GeneralArrow",irr::video::SColor::SColor(255,0,0,0),irr::video::SColor::SColor(255,0,0,0)); //for use with random object
	
	myNode->subscribe("visualizationMarker", visMarker, &Vis3d::newMarker,this);
	//test
	/*std::cout << "maxRand is " << RAND_MAX << std::endl;
	visMarker.x = markers.size();
	visMarker.y = 0;
	visMarker.z = 0;
	visMarker.roll = 0;
	visMarker.pitch = 0;
	visMarker.yaw = 0;
	visMarker.alpha = 255;
	visMarker.r = 100+int(155.0*rand()/(RAND_MAX + 1.0));
	visMarker.g = 100+int(155.0*rand()/(RAND_MAX + 1.0));
	visMarker.b = 100+int(155.0*rand()/(RAND_MAX + 1.0));
	visMarker.xScale = 1;
	visMarker.yScale = 1;
	visMarker.zScale = 1;
	visMarker.text = "cheese!";
	visMarker.id = 1000+markers.size();
	visMarker.type = markers.size()%4;
	visMarker.action = 0;
	newMarker();*/
	//end test
}

///Vis3d destructor
/**Disables all drawings, then deletes (hopefully) all models*/
Vis3d::~Vis3d()
{
	disable();
	//delete localClient;
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
	delete ilucs;
	//delete tfClient;
}

/**Disables all drawings, but does not delete them.  This allows for future use of the window*/
void Vis3d::disable()
{
	disableUCS();
	if(model[0])
		disableModel();
	disableHead();
	disableStereo();
	disableFloor();		
}

/**Almost always true*/
bool Vis3d::isEnabled()
{
    return pLocalRenderer->isEnabled();
}

/**If the camera is not stationary, it takes the location and focus point from the previous camera*/
void Vis3d::changeView(int id)
{
	pLocalRenderer->lock();
	irr::core::vector3df pos = pLocalRenderer->manager()->getActiveCamera()->getPosition();
	irr::core::vector3df tar = pLocalRenderer->manager()->getActiveCamera()->getTarget();
	switch(id)
	{
		case Maya:
		case FPS:
			cameras[id]->setTarget(tar);
			cameras[id]->setPosition(pos);
			break;
			
	}
	pLocalRenderer->manager()->setActiveCamera(cameras[id]);
	irr::core::rect<irr::s32> viewPort = pLocalRenderer->driver()->getViewPort();
	pLocalRenderer->manager()->getActiveCamera()->setAspectRatio(((float)viewPort.getWidth())/((float)viewPort.getHeight()));
	pLocalRenderer->unlock();
	//this is for testing purposes only
	/*std::cerr << std::endl;
	visMarker.x = markers.size();
	visMarker.y = 0;
	visMarker.z = 0;
	visMarker.roll = 0;
	visMarker.pitch = 0;
	visMarker.yaw = 0;
	visMarker.alpha = 255;
	visMarker.r = 100+int(155.0*rand()/(RAND_MAX + 1.0));
	visMarker.g = 100+int(155.0*rand()/(RAND_MAX + 1.0));
	visMarker.b = 100+int(155.0*rand()/(RAND_MAX + 1.0));
	visMarker.xScale = 1;
	visMarker.yScale = 1;
	visMarker.zScale = 1;
	visMarker.text = "cheese!";
	visMarker.id = 1000+markers.size();
	visMarker.type = markers.size()%4;
	visMarker.action = 0;
	newMarker();*/
}

void Vis3d::enableHead()
{
	scanDir = 1;
	headVertScanCount = 0;
    changeHeadLaser(scanT);
    for(int i = 0; i < cloudArrayLength; i++)
    {
		pLocalRenderer->enable(ilHeadCloud[i]);
		ilHeadCloud[i]->setVisible(true);
    }   
}

void Vis3d::enableModel()
{
	//TiXmlDocument robodesc( descPath );
	//robodesc.LoadFile();
	//static const char *modelPaths[] = {"../pr2_models/base1000.3DS","../pr2_models/body1000.3DS","../pr2_models/caster1000r2.3DS","../pr2_models/caster1000r2.3DS","../pr2_models/caster1000r2.3DS","../pr2_models/caster1000r2.3DS"};
	//static const char *modelPaths[] = {"pr2_models/caster1000r2.3DS","","","pr2_models/caster1000r2.3DS","","","pr2_models/caster1000r2.3DS","","","pr2_models/caster1000r2.3DS","","","pr2_models/body1000.3DS","pr2_models/sh-pan1000.3DS","pr2_models/sh-pitch1000.3DS","pr2_models/sh-roll1000.3DS","","","","","pr2_models/sh-pan1000.3DS","pr2_models/sh-pitch1000.3DS","pr2_models/sh-roll1000.3DS","","","","","","","pr2_models/head-pan1000.3DS","pr2_models/head-tilt1000.3DS","","","","","","pr2_models/base1000.3DS","",""};
	//static const char *modelPaths[] = {"","","pr2_models/caster1000r2.3DS","","","pr2_models/caster1000r2.3DS","","","pr2_models/caster1000r2.3DS","","","pr2_models/caster1000r2.3DS","pr2_models/base1000r.3DS","pr2_models/body1000r.3DS","pr2_models/sh-pan1000.3DS","pr2_models/sh-pitch1000.3DS","pr2_models/sh-roll1000.3DS","","","","","","","pr2_models/sh-pan1000.3DS","pr2_models/sh-pitch1000.3DS","pr2_models/sh-roll1000.3DS","","","","","","","pr2_models/head-pan1000.3DS","pr2_models/head-tilt1000.3DS","","",""};
	static const char *modelPaths[] = {"../pr2_models/wheel.3DS","../pr2_models/wheel.3DS","../pr2_models/caster.3DS","../pr2_models/wheel.3DS","../pr2_models/wheel.3DS","../pr2_models/caster.3DS","../pr2_models/wheel.3DS","../pr2_models/wheel.3DS","../pr2_models/caster.3DS","../pr2_models/wheel.3DS","../pr2_models/wheel.3DS","../pr2_models/caster.3DS","../pr2_models/base.3DS","../pr2_models/body.3DS","../pr2_models/sh-pan.3DS","../pr2_models/sh-pitch.3DS","../pr2_models/sh-roll.3DS","../pr2_models/el-pitch.3DS","../pr2_models/fa-roll.3DS","../pr2_models/wr-pitch.3DS","../pr2_models/wr-roll.3DS","","","../pr2_models/sh-pan.3DS","../pr2_models/sh-pitch.3DS","../pr2_models/sh-roll.3DS","../pr2_models/el-pitch.3DS","../pr2_models/fa-roll.3DS","../pr2_models/wr-pitch.3DS","../pr2_models/wr-roll.3DS","","","../pr2_models/head-pan.3DS","../pr2_models/head-tilt.3DS","","",""};
	pLocalRenderer->lock();
	for(int i = 0; i < PR2::MAX_FRAMEIDS-PR2::FRAMEID_CASTER_FL_WHEEL_L; i++)
	{
		if (i != 21 && i != 22 && i != 30 && i != 31 && i < 31){
		libTF::TFPose aPose;
		aPose.x = 0;
		aPose.y = 0;
		aPose.z = 0;
		aPose.roll = 0;
		aPose.pitch = 0;
		aPose.yaw = 0;
		aPose.time = 0;
		aPose.frame = PR2::FRAMEID_CASTER_FL_WHEEL_L + i;
		libTF::TFPose inBaseFrame;
		try
		{
			inBaseFrame = this->tfClient.transformPose(PR2::FRAMEID_BASE, aPose);
		}
		catch(libTF::TransformReference::LookupException e)
		{
			inBaseFrame;
			inBaseFrame.x = 0;
			inBaseFrame.y = 0;
			inBaseFrame.z = 0;
			inBaseFrame.roll = 0;
			inBaseFrame.pitch = 0;
			inBaseFrame.yaw = 0;
			inBaseFrame.time = 0;
			inBaseFrame.frame = PR2::FRAMEID_BASE;
		}
		std::cout << "Coordinates for : " << i << "; "<<inBaseFrame.x << ", " <<  inBaseFrame.y << ", " << inBaseFrame.z << "; "<<inBaseFrame.roll << ", " <<  inBaseFrame.pitch << ", " << inBaseFrame.yaw <<std::endl;
		ILModel *tempModel = new ILModel(pLocalRenderer->manager(), intermediate, (irr::c8*)modelPaths[i], PR2::FRAMEID_CASTER_FL_WHEEL_L + i, (float)inBaseFrame.x,(float)inBaseFrame.y, (float)inBaseFrame.z, (float)inBaseFrame.roll,(float)(inBaseFrame.pitch), (float)(inBaseFrame.yaw));
		tempModel->getNode()->getMaterial(0).AmbientColor.set(255,100+int(155.0*rand()/(RAND_MAX + 1.0)),100+int(155.0*rand()/(RAND_MAX + 1.0)),100+int(155.0*rand()/(RAND_MAX + 1.0)));
		model.push_back(tempModel);
		}
		else
			model.push_back(NULL);
	}
	myNode->subscribe("transform",transform, &Vis3d::newTransform,this);
	pLocalRenderer->unlock();
}

/**Red = x, green = y, blue = z*/
void Vis3d::enableUCS()
{
	pLocalRenderer->lock();
	/*if(!ilucs)
		ilucs = new ILUCS(pLocalRenderer->manager(),true);*/
	ilucs->setVisible(true);
	pLocalRenderer->unlock();
}

void Vis3d::enableGrid()
{
	pLocalRenderer->lock();
	pLocalRenderer->enable(ilGrid);
	ilGrid->setVisible(true);
	std::cout<<"Enabling Grid\n";
	pLocalRenderer->unlock();
}

void Vis3d::enableFloor()
{
    myNode->subscribe("scan", ptCldFloor, &Vis3d::addFloorCloud,this);
    myNode->subscribe("shutterFloor", shutFloor, &Vis3d::shutterFloor,this);
    pLocalRenderer->enable(ilFloorCloud);
    ilFloorCloud->setVisible(true);
}

/**Data type should be changed from a point cloud to a ???...big point cloud?*/
void Vis3d::enableStereo()
{
    myNode->subscribe("cloudStereo", ptCldStereo, &Vis3d::addStereoCloud,this);
    myNode->subscribe("shutterStereo", shutStereo, &Vis3d::shutterStereo,this);
    pLocalRenderer->enable(ilStereoCloud);
    ilStereoCloud->setVisible(true);
}

void Vis3d::enableObjects()
{
	pLocalRenderer->lock();
	for(int i = 0; i < markers.size(); i++)
	{
		markers[i]->setVisible(true);
		//if markers get deleted and not just removed, should do an if to check if markers[i] exists
	}
	objectsVisibility = true;
	pLocalRenderer->unlock();
}

void Vis3d::disableHead()
{
    myNode->unsubscribe("cloud");
    myNode->unsubscribe("shutter");
    myNode->unsubscribe("cloud_full");
    shutterHead();
	pLocalRenderer->lock();
    for(int i = 0; i < cloudArrayLength; i++)
    {
	    
		pLocalRenderer->disable(ilHeadCloud[i]);
	    ilHeadCloud[i]->setVisible(false);
    }  
	pLocalRenderer->unlock();	    
}

void Vis3d::disableFloor()
{
    myNode->unsubscribe("scan");
    myNode->unsubscribe("shutterFloor");
    shutterFloor();
	pLocalRenderer->lock();
    pLocalRenderer->disable(ilFloorCloud);
    ilFloorCloud->setVisible(false);
	pLocalRenderer->unlock();
}

void Vis3d::disableStereo()
{
    myNode->unsubscribe("cloudStereo");
    myNode->unsubscribe("shutterStereo");
    shutterStereo();
	pLocalRenderer->lock();
    pLocalRenderer->disable(ilStereoCloud);
    ilStereoCloud->setVisible(false);
	pLocalRenderer->unlock();
}

void Vis3d::disableUCS()
{
	pLocalRenderer->lock();
	/*if(ilucs)
	{
		delete ilucs;
		ilucs = 0;
	}*/
	ilucs->setVisible(false);
	pLocalRenderer->unlock();
}

void Vis3d::disableGrid()
{
	pLocalRenderer->lock();
	pLocalRenderer->disable(ilGrid);
	ilGrid->setVisible(false);
	std::cout<<"Disabling Grid\n";
	pLocalRenderer->unlock();
}

void Vis3d::disableModel()
{
	std::cout << "killing model\n";
	myNode->unsubscribe("transform");
	pLocalRenderer->lock();
	/*for(int i = 0; i < PR2::MAX_JOINTS; i++)
	{
		delete model[i];
		model[i] = 0;
	}*/
	model.clear();
	pLocalRenderer->unlock();
	
}

void Vis3d::disableObjects()
{
	pLocalRenderer->lock();
	for(int i = 0; i < markers.size(); i++)
	{
		markers[i]->setVisible(false);
		//if markers get deleted and not just removed, should do an if to check if markers[i] exists
	}
	objectsVisibility = false;
	pLocalRenderer->unlock();
}


void Vis3d::shutterHead()
{
	//std::cout << "shutter\n";
    pLocalRenderer->lock();
    switch(scanT)
    {
    	case Wipe:
    	case AtOnce:
			for(int i = 0; i < cloudArrayLength; i++)
			{
				ilHeadCloud[i]->resetCount();
			}
			headVertScanCount = 0;
			break;
		case Replace:
			scanDir *= -1;
			headVertScanCount += scanDir;
			break;
	}
    pLocalRenderer->unlock();
}

void Vis3d::shutterFloor()
{
    pLocalRenderer->lock();
    ilFloorCloud->resetCount();
    pLocalRenderer->unlock();
}

void Vis3d::shutterStereo()
{
    pLocalRenderer->lock();
    ilStereoCloud->resetCount();
    pLocalRenderer->unlock();
}

void Vis3d::addHeadCloud()
{
	libTF::TFPose aPose;
	aPose.x = 0;
	aPose.y = 0;
	aPose.z = 0;
	aPose.roll = 0;
	aPose.pitch = 0;
	aPose.yaw = 0;
	aPose.time = 0;
	aPose.frame = PR2::FRAMEID_TILT_LASER_BLOCK;
	libTF::TFPose inBaseFrame;
	try
	{
		inBaseFrame = this->tfClient.transformPose(PR2::FRAMEID_BASE, aPose);
	}
	catch(libTF::TransformReference::LookupException e)
	{
		inBaseFrame;
		inBaseFrame.x = 0;
		inBaseFrame.y = 0;
		inBaseFrame.z = 0;
		inBaseFrame.roll = 0;
		inBaseFrame.pitch = 0;
		inBaseFrame.yaw = 0;
		inBaseFrame.time = 0;
		inBaseFrame.frame = PR2::FRAMEID_BASE;
	}
    switch(scanT)
    {
    	case Wipe:
    	pLocalRenderer->lock();
			if(headVertScanCount < cloudArrayLength)
			{
				for(int i = 0; i < min((uint32_t)65535,ptCldHead.get_pts_size()); i++)
				{
					ilHeadCloud[headVertScanCount]->addPoint(inBaseFrame.x + ptCldHead.pts[i].x, inBaseFrame.y + ptCldHead.pts[i].y, inBaseFrame.z + ptCldHead.pts[i].z, 255 ,min((int)(ptCldHead.chan[0].vals[i]/intensityRange),255),min((int)(ptCldHead.chan[0].vals[i]/intensityRange),255));
				}
				headVertScanCount++;
			}
			break;
		case Replace:
		pLocalRenderer->lock();
			if(headVertScanCount < cloudArrayLength && headVertScanCount > -1)
			{
				ilHeadCloud[headVertScanCount]->resetCount();
				for(int i = 0; i < min((uint32_t)65535,ptCldHead.get_pts_size()); i++)
				{
					ilHeadCloud[headVertScanCount]->addPoint(inBaseFrame.x + ptCldHead.pts[i].x, inBaseFrame.y + ptCldHead.pts[i].y, inBaseFrame.z + ptCldHead.pts[i].z, 255 ,min((int)(ptCldHead.chan[0].vals[i]/intensityRange),255),min((int)(ptCldHead.chan[0].vals[i]/intensityRange),255));
				}
				headVertScanCount += scanDir;
			}
			break;
		case AtOnce:
			shutterHead();
			pLocalRenderer->lock();
			//std::cout << "add head cloud full\n";
			for(int i = 0; i < min(ptCldHead.get_pts_size(),((uint32_t)cloudArrayLength*65535)); i++)
			{
				if(((float)i)/((float)(headVertScanCount + 1)) > 65535)
					headVertScanCount++;
				ilHeadCloud[headVertScanCount]->addPoint(inBaseFrame.x + ptCldHead.pts[i].x, inBaseFrame.y + ptCldHead.pts[i].y, inBaseFrame.z + ptCldHead.pts[i].z, 255 ,min((int)(ptCldHead.chan[0].vals[i]/intensityRange),255),min((int)(ptCldHead.chan[0].vals[i]/intensityRange),255));
			}
			break;
		default: break;
	}
	pLocalRenderer->unlock();
}

void Vis3d::addFloorCloud()
{
	libTF::TFPose aPose;
	aPose.x = 0;
	aPose.y = 0;
	aPose.z = 0;
	aPose.roll = 0;
	aPose.pitch = 0;
	aPose.yaw = 0;
	aPose.time = 0;
	aPose.frame = PR2::FRAMEID_BASE_LASER_BLOCK;
	libTF::TFPose inBaseFrame;
	try
	{
		inBaseFrame = this->tfClient.transformPose(PR2::FRAMEID_BASE, aPose);
	}
	catch(libTF::TransformReference::LookupException e)
	{
		inBaseFrame;
		inBaseFrame.x = 0;
		inBaseFrame.y = 0;
		inBaseFrame.z = 0;
		inBaseFrame.roll = 0;
		inBaseFrame.pitch = 0;
		inBaseFrame.yaw = 0;
		inBaseFrame.time = 0;
		inBaseFrame.frame = PR2::FRAMEID_BASE;
	}
	shutterFloor();
    pLocalRenderer->lock();
    
	for(int i = 0; i < min((uint32_t)65535,(uint32_t)((ptCldFloor.angle_max-ptCldFloor.angle_min)/ptCldFloor.angle_increment+.5)); i++)
	{//CHANGE MY X,Y,Z for transform!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
		ilFloorCloud->addPoint(inBaseFrame.x + cos(i*ptCldFloor.angle_increment + ptCldFloor.angle_min)*ptCldFloor.ranges[i],inBaseFrame.y + sin(i*ptCldFloor.angle_increment + ptCldFloor.angle_min)*ptCldFloor.ranges[i],inBaseFrame.z, min((int)(ptCldFloor.intensities[i]/intensityRange),255),255,min((int)(ptCldFloor.intensities[i]/intensityRange),255));
	}
	
    pLocalRenderer->unlock();
}

void Vis3d::addStereoCloud()
{
    pLocalRenderer->lock();
    if(ptCldStereo.get_pts_size() > 65535)
    {
		for(int i = 0; i < 65535; i++)
		{
			ilStereoCloud->addPoint(ptCldStereo.pts[i].x, ptCldStereo.pts[i].y, ptCldStereo.pts[i].z, min((int)(ptCldStereo.chan[0].vals[i]),255),min((int)(ptCldStereo.chan[0].vals[i]),255),255);
		}
    }
    else
    {
		for(size_t i = 0; i < ptCldStereo.get_pts_size(); i++)
		{
			ilStereoCloud->addPoint(ptCldStereo.pts[i].x, ptCldStereo.pts[i].y, ptCldStereo.pts[i].z, min((int)(ptCldStereo.chan[0].vals[i]),255),min((int)(ptCldStereo.chan[0].vals[i]),255),255);
		}
    }
    pLocalRenderer->unlock();
}

template <class T> const T& max ( const T& a, const T& b ) {
	return (b<a)?a:b;
}

template <class T> const T& min ( const T& a, const T& b ) {
	return (a<b)?a:b;
}

/**Wipe shows a scan from the top to the bottom or bottom to top then erases everything.  Replace overwrites the points the corresponding points from the previous scan.  AtOnce shows a full top to bottom or bottom to top scan at once, instead of line by line*/
void Vis3d::changeHeadLaser(int choice)
{
	//std::cout << "Wipe " << Wipe << " Replace " << Replace << " At Once " << AtOnce  << " Choice " << choice << std::endl;
	switch(choice)
	{
		case AtOnce:
			//std::cout << "At Once\n";
			myNode->subscribe("full_cloud", ptCldHead, &Vis3d::addHeadCloud,this);
			myNode->unsubscribe("cloud");
    		myNode->unsubscribe("shutter");
    		scanT = choice;
    		break;
		case Wipe:
		case Replace:
			//std::cout << "Wipe/Replace\n";
			myNode->unsubscribe("full_cloud");
			myNode->subscribe("cloud", ptCldHead, &Vis3d::addHeadCloud,this);
    		myNode->subscribe("shutter", shutHead, &Vis3d::shutterHead,this);
			scanT = choice;
			break;
	}
}

void Vis3d::newTransform()
{
	for(int i = 0; i < PR2::MAX_FRAMEIDS-PR2::FRAMEID_CASTER_FL_WHEEL_L; i++)
	{
		if (i != 21 && i != 22 && i != 30 && i != 31 && i < 31){
			libTF::TFPose aPose;
			aPose.x = 0;
			aPose.y = 0;
			aPose.z = 0;
			aPose.roll = 0;
			aPose.pitch = 0;
			aPose.yaw = 0;
			aPose.time = 0;
			aPose.frame = PR2::FRAMEID_CASTER_FL_WHEEL_L + i;
			libTF::TFPose inBaseFrame;
			try
			{
				inBaseFrame = this->tfClient.transformPose(PR2::FRAMEID_BASE, aPose);
			}
			catch(libTF::TransformReference::LookupException e)
			{
				inBaseFrame;
				inBaseFrame.x = 0;
				inBaseFrame.y = 0;
				inBaseFrame.z = 0;
				inBaseFrame.roll = 0;
				inBaseFrame.pitch = 0;
				inBaseFrame.yaw = 0;
				inBaseFrame.time = 0;
				inBaseFrame.frame = PR2::FRAMEID_BASE;
			}
			model[i]->setPosition((float)inBaseFrame.x,(float)inBaseFrame.y, (float)inBaseFrame.z);
			model[i]->setRotation((float)inBaseFrame.roll,(float)(inBaseFrame.pitch), (float)(inBaseFrame.yaw));
		}
	}
}

/**Displays objects (arrows, cubes, spheres, text, and meshes) with user controlled parameters.  Check enums in header and VisualizationMarker.msg for details*/
void Vis3d::newMarker()
{
	std::cerr << "numMarkers " << markers.size() <<  std::endl;
	pLocalRenderer->lock();
	switch(visMarker.action)
	{
		
		case newObject: //0
			switch(visMarker.type)
			{
				case Arrow: //0
					{
					ILModel *tempModel = new ILModel(pLocalRenderer->manager(), intermediate, pLocalRenderer->manager()->addAnimatedMeshSceneNode(ArrowMesh), visMarker.id, visMarker.x, visMarker.y, visMarker.z, visMarker.roll, visMarker.pitch, visMarker.yaw);
					tempModel->setScale(visMarker.xScale,visMarker.yScale,visMarker.zScale);
					tempModel->getNode()->getMaterial(0).AmbientColor.set(visMarker.alpha,visMarker.r,visMarker.g,visMarker.b);
					tempModel->getNode()->getMaterial(1).AmbientColor.set(visMarker.alpha,visMarker.r,visMarker.g,visMarker.b);
					tempModel->setVisible(objectsVisibility);
					markers.push_back(tempModel);
					}
					break;
				case Cube: //1
					{
					ILModel *tempModel = new ILModel(pLocalRenderer->manager(), intermediate, pLocalRenderer->manager()->addCubeSceneNode(1.0f), visMarker.id, visMarker.x, visMarker.y, visMarker.z, visMarker.roll, visMarker.pitch, visMarker.yaw);
					tempModel->setScale(visMarker.xScale,visMarker.yScale,visMarker.zScale);
					tempModel->getNode()->getMaterial(0).AmbientColor.set(visMarker.alpha,visMarker.r,visMarker.g,visMarker.b);		
					tempModel->setVisible(objectsVisibility);				
					markers.push_back(tempModel);
					}
					break;
				case Sphere: //2
					{
					ILModel *tempModel = new ILModel(pLocalRenderer->manager(), intermediate, pLocalRenderer->manager()->addSphereSceneNode(.5f), visMarker.id, visMarker.x, visMarker.y, visMarker.z, visMarker.roll, visMarker.pitch, visMarker.yaw);
					tempModel->setScale(visMarker.xScale,visMarker.yScale,visMarker.zScale);
					tempModel->getNode()->getMaterial(0).AmbientColor.set(visMarker.alpha,visMarker.r,visMarker.g,visMarker.b);	
					tempModel->setVisible(objectsVisibility);
					markers.push_back(tempModel);
					}
					break;
				case Text: //3
					{
						wchar_t *temp = new wchar_t[visMarker.text.size()+1];
						for( int j = 0; j < visMarker.text.size(); j++)
						{
							temp[j]=visMarker.text[j];
						}
						temp[visMarker.text.size()] = NULL;
						ILModel *tempModel = new ILModel(pLocalRenderer->manager(), intermediate, pLocalRenderer->manager()->addTextSceneNode(pLocalRenderer->device()->getGUIEnvironment()->getBuiltInFont(),temp,irr::video::SColor(visMarker.alpha,visMarker.r,visMarker.g,visMarker.b)), visMarker.id, visMarker.x, visMarker.y, visMarker.z, visMarker.roll, visMarker.pitch, visMarker.yaw);
						tempModel->setScale(visMarker.xScale,visMarker.yScale,visMarker.zScale);
						tempModel->setVisible(objectsVisibility);
						markers.push_back(tempModel);
					}
					break;
				case Custom: //4
					{
						char *temp = new char[visMarker.text.size()];
						for( int j = 0; j < visMarker.text.size(); j++)
						{
							temp[j]=visMarker.text[j];
						}
						ILModel *tempModel = new ILModel(pLocalRenderer->manager(), intermediate, (irr::c8*)temp, visMarker.id, visMarker.x, visMarker.y, visMarker.z, visMarker.roll, visMarker.pitch, visMarker.yaw);
						tempModel->setScale(visMarker.xScale,visMarker.yScale,visMarker.zScale);
						tempModel->getNode()->getMaterial(0).AmbientColor.set(visMarker.alpha,visMarker.r,visMarker.g,visMarker.b);
						tempModel->setVisible(objectsVisibility);
						markers.push_back(tempModel);
					}
					break;
			}
			break;
		case modifyObject: //1
			{
				int num = -1;
				for(int i = 0; i < markers.size(); i++)
				{
					if(markers[i]->getID() == visMarker.id)
					{
						num = i;
						break;
					}
				}
				if(num != -1)
				{
					markers[num]->setPosition(visMarker.x,visMarker.y,visMarker.z);
					markers[num]->setRotation(visMarker.roll,visMarker.pitch,visMarker.yaw);
					markers[num]->setScale(visMarker.xScale,visMarker.yScale,visMarker.zScale);
					if(visMarker.type != Text)
					{
						markers[num]->getNode()->getMaterial(0).AmbientColor.set(visMarker.alpha,visMarker.r,visMarker.g,visMarker.b);
						if(visMarker.type == Arrow)
							markers[markers.size()-1]->getNode()->getMaterial(1).AmbientColor.set(visMarker.alpha,visMarker.r,visMarker.g,visMarker.b);
					}
					//allow you to change text here with an else statement???
				}
			}
			break;
		case deleteObject: //2
			int num = -1;
			for(int i = 0; i < markers.size(); i++)
			{
				if(markers[i]->getID() == visMarker.id)
				{
					num = i;
					break;
				}
			}
			if(num != -1)
			{
				markers[num]->kill();
				//delete the rest!!!???
			}
			break;
	}
	pLocalRenderer->unlock();
} 

