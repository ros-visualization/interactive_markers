#include "Vis3d.h"
#include "urdf/URDF.h"

/**Initializes all Irrlicht models, cameras, and lights */
Vis3d::Vis3d(ros::node *aNode) : tfClient(*aNode)
{
	objectsVisibility = true;
	myNode = aNode;
	headVertScanCount = 0;
	stereoVertScanCount = 0;
	scanDir = -1;
	scanT = Wipe;

	localClient = new ILClient();
	pLocalRenderer = ILClient::getSingleton();
	pLocalRenderer->lock();
	ilHeadCloud = new ILLaserScan(pLocalRenderer->manager()->getRootSceneNode(),pLocalRenderer->manager(),199);
	ilFloorCloud = new ILLaserScan(pLocalRenderer->manager()->getRootSceneNode(),pLocalRenderer->manager(),200);
	ilStereoCloud = new ILLaserScan(pLocalRenderer->manager()->getRootSceneNode(),pLocalRenderer->manager(),201);
	ilGrid = new ILGrid(pLocalRenderer->manager()->getRootSceneNode(), pLocalRenderer->manager(), 202);
	ilucs = new ILUCS(pLocalRenderer->manager()->getRootSceneNode(), pLocalRenderer->manager(),true, 203);
	pLocalRenderer->unlock();
	pLocalRenderer->addNode(ilFloorCloud);
	pLocalRenderer->addNode(ilStereoCloud);
	pLocalRenderer->addNode(ilHeadCloud);
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
}

///Vis3d destructor
/**Disables all drawings, then deletes (hopefully) all models*/
Vis3d::~Vis3d()
{
	disable();
}

/**Disables all drawings, but does not delete them.  This allows for future use of the window*/
void Vis3d::disable()
{
	std::cerr << "Disable 1\n";
	disableUCS();
	std::cerr << "Disable 2\n";
	//if(model[0])
	disableModel();
	std::cerr << "Disable 3\n";
	disableHead();
	std::cerr << "Disable 4\n";
	disableStereo();
	std::cerr << "Disable 5\n";
	disableFloor();
	std::cerr << "Disable 6\n";
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
}

void Vis3d::enableHead()
{
    changeHeadLaser(scanT);
    pLocalRenderer->enable(ilHeadCloud);
	ilHeadCloud->setVisible(true);
}

void Vis3d::enableModel()
{
	std::string pathnamePrefix("../pr2_models/");
	std::string pathnameSuffix(".3DS");
	std::string content;
	myNode->get_param("robotdesc/pr2", content);
	robot_desc::URDF *file = new robot_desc::URDF();
	file->loadString(content.c_str());
	std::vector <robot_desc::URDF::Link*> links;
	file->getLinks(links);
	//m_nameMap.clear();
	m_modelMap.clear();
	pLocalRenderer->lock();
	for(unsigned int i = 0; i < links.size(); i++)
	{
		//m_nameMap[links[i]->name] = pathnamePrefix + links[i]->visual->geometry->filename + pathnameSuffix;
		std::cout << links[i]->name << ": " << pathnamePrefix + links[i]->visual->geometry->filename + pathnameSuffix << std::endl;
	//}


	//static const char *modelPaths[] = {"../pr2_models/wheel.3DS","../pr2_models/wheel.3DS","../pr2_models/caster.3DS","../pr2_models/wheel.3DS","../pr2_models/wheel.3DS","../pr2_models/caster.3DS","../pr2_models/wheel.3DS","../pr2_models/wheel.3DS","../pr2_models/caster.3DS","../pr2_models/wheel.3DS","../pr2_models/wheel.3DS","../pr2_models/caster.3DS","../pr2_models/base.3DS","../pr2_models/body.3DS","../pr2_models/sh-pan.3DS","../pr2_models/sh-pitch.3DS","../pr2_models/sh-roll.3DS","../pr2_models/el-pitch.3DS","../pr2_models/fa-roll.3DS","../pr2_models/wr-pitch.3DS","../pr2_models/wr-roll.3DS","","","../pr2_models/sh-pan.3DS","../pr2_models/sh-pitch.3DS","../pr2_models/sh-roll.3DS","../pr2_models/el-pitch.3DS","../pr2_models/fa-roll.3DS","../pr2_models/wr-pitch.3DS","../pr2_models/wr-roll.3DS","","","../pr2_models/head-pan.3DS","../pr2_models/head-tilt.3DS","","",""};

	//for(int i = 0; i < PR2::PR2_FRAMEID_COUNT; i++)
	//{
		libTF::TFPose aPose;
		aPose.x = 0;
		aPose.y = 0;
		aPose.z = 0;
		aPose.roll = 0;
		aPose.pitch = 0;
		aPose.yaw = 0;
		aPose.time = 0;
		//aPose.frame = this->tfClient.lookup(PR2::PR2_FRAMEID[i]);// + i
		aPose.frame = this->tfClient.lookup(links[i]->name);
		libTF::TFPose inBaseFrame;
		try
		{
			inBaseFrame = this->tfClient.transformPose("base", aPose);
		}
		catch(libTF::TransformReference::LookupException e)
		{
			inBaseFrame.x = 0;
			inBaseFrame.y = 0;
			inBaseFrame.z = 0;
			inBaseFrame.roll = 0;
			inBaseFrame.pitch = 0;
			inBaseFrame.yaw = 0;
			inBaseFrame.time = 0;
			inBaseFrame.frame = this->tfClient.lookup("base");
		}
		std::cout << "Coordinates for : " << i << "; "<<inBaseFrame.x << ", " <<  inBaseFrame.y << ", " << inBaseFrame.z << "; "<<inBaseFrame.roll << ", " <<  inBaseFrame.pitch << ", " << inBaseFrame.yaw <<std::endl;
		try{
			if(links[i]->visual->geometry->filename != "")
			{
				ILModel *tempModel = new ILModel(pLocalRenderer->manager(), intermediate, (irr::c8*)(pathnamePrefix + links[i]->visual->geometry->filename + pathnameSuffix).c_str(), this->tfClient.lookup(links[i]->name), (float)inBaseFrame.x,(float)inBaseFrame.y, (float)inBaseFrame.z, (float)inBaseFrame.roll,(float)(inBaseFrame.pitch), (float)(inBaseFrame.yaw));
				if(tempModel->getNode() != NULL)
				{
					tempModel->getNode()->getMaterial(0).AmbientColor.set(255,100+int(155.0*rand()/(RAND_MAX + 1.0)),100+int(155.0*rand()/(RAND_MAX + 1.0)),100+int(155.0*rand()/(RAND_MAX + 1.0)));
					//model.push_back(tempModel);
					m_modelMap[links[i]->name] = tempModel;
				}
			}
			else
				m_modelMap[links[i]->name] = NULL;
		}
		catch(...)
		{
			//model.push_back(NULL);
			m_modelMap[links[i]->name] = NULL;
		}
	}
	delete file;
	std::cout << "Loaded all models\n";
	myNode->subscribe("transform",transform, &Vis3d::newTransform,this);
	pLocalRenderer->unlock();
}

/**Red = x, green = y, blue = z*/
void Vis3d::enableUCS()
{
	pLocalRenderer->lock();
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
    myNode->subscribe("shutterScan", shutFloor, &Vis3d::shutterFloor,this);
    pLocalRenderer->enable(ilFloorCloud);
    ilFloorCloud->setVisible(true);
}

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
    ilHeadCloud->setVisible(false);
    pLocalRenderer->disable(ilHeadCloud);
	pLocalRenderer->unlock();
}

void Vis3d::disableFloor()
{
    myNode->unsubscribe("scan");
    myNode->unsubscribe("shutterScan");
    shutterFloor();
	pLocalRenderer->lock();
	ilFloorCloud->setVisible(false);
    pLocalRenderer->disable(ilFloorCloud);
	pLocalRenderer->unlock();
}

void Vis3d::disableStereo()
{
    myNode->unsubscribe("cloudStereo");
    myNode->unsubscribe("shutterStereo");
    shutterStereo();
	pLocalRenderer->lock();
    ilStereoCloud->setVisible(false);
    pLocalRenderer->disable(ilStereoCloud);
	pLocalRenderer->unlock();
}

void Vis3d::disableUCS()
{
	pLocalRenderer->lock();
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
	if(m_modelMap.size() > 0)
	{
		for( map<std::string, ILModel*>::iterator iter = m_modelMap.begin(); iter != m_modelMap.end(); iter++ )
		{
			delete (*iter).second;
			//model[i] = 0;
		}
		m_modelMap.clear();
	}
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
	//deleteObjects();
}

void Vis3d::deleteObjects()
{
	disableObjects();
	pLocalRenderer->lock();
	for(int i = 0; i < markers.size(); i++)
	{
		delete markers[i];
	}
	markers.clear();
	pLocalRenderer->unlock();
}


void Vis3d::shutterHead()
{
    pLocalRenderer->lock();
	ilHeadCloud->resetCount();
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
	aPose.frame = this->tfClient.lookup("FRAMEID_TILT_LASER_BLOCK"); //TODO: put me in pr2.xml and change my string
	libTF::TFPose inBaseFrame;
	try
	{
		inBaseFrame = this->tfClient.transformPose("base", aPose);
	}
	catch(libTF::TransformReference::LookupException e)
	{
		inBaseFrame.x = 0;
		inBaseFrame.y = 0;
		inBaseFrame.z = 0;
		inBaseFrame.roll = 0;
		inBaseFrame.pitch = 0;
		inBaseFrame.yaw = 0;
		inBaseFrame.time = 0;
		inBaseFrame.frame = this->tfClient.lookup("base");
	}
    switch(scanT)
    {
    	case Wipe:
			pLocalRenderer->lock();
			for(int i = 0; i < ptCldHead.get_pts_size(); i++)
			{
				ilHeadCloud->addPoint(inBaseFrame.x + ptCldHead.pts[i].x, inBaseFrame.y + ptCldHead.pts[i].y, inBaseFrame.z + ptCldHead.pts[i].z, 255 ,min((int)(ptCldHead.chan[0].vals[i]/intensityRange),255),min((int)(ptCldHead.chan[0].vals[i]/intensityRange),255));
			}
			break;
		case AtOnce:
			shutterHead();
			pLocalRenderer->lock();
			for(int i = 0; i < ptCldHead.get_pts_size(); i++)
			{
				ilHeadCloud->addPoint(inBaseFrame.x + ptCldHead.pts[i].x, inBaseFrame.y + ptCldHead.pts[i].y, inBaseFrame.z + ptCldHead.pts[i].z, 255 ,min((int)(ptCldHead.chan[0].vals[i]/intensityRange),255),min((int)(ptCldHead.chan[0].vals[i]/intensityRange),255));
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
	aPose.frame = this->tfClient.lookup("FRAMEID_BASE_LASER_BLOCK"); //TODO: put me in pr2.xml and change my string
	libTF::TFPose inBaseFrame;
	try
	{
		inBaseFrame = this->tfClient.transformPose("base", aPose);
	}
	catch(libTF::TransformReference::LookupException e)
	{
		inBaseFrame.x = 0;
		inBaseFrame.y = 0;
		inBaseFrame.z = 0;
		inBaseFrame.roll = 0;
		inBaseFrame.pitch = 0;
		inBaseFrame.yaw = 0;
		inBaseFrame.time = 0;
		inBaseFrame.frame = this->tfClient.lookup("base");
	}
	shutterFloor();
    pLocalRenderer->lock();

	//for(int i = 0; i < min((uint32_t)65535,(uint32_t)((ptCldFloor.angle_max-ptCldFloor.angle_min)/ptCldFloor.angle_increment+.5)); i++)
	for(int i = 0; i < (uint32_t)((ptCldFloor.angle_max-ptCldFloor.angle_min)/ptCldFloor.angle_increment+.5); i++)
	{
		ilFloorCloud->addPoint(inBaseFrame.x + cos(i*ptCldFloor.angle_increment + ptCldFloor.angle_min)*ptCldFloor.ranges[i],inBaseFrame.y + sin(i*ptCldFloor.angle_increment + ptCldFloor.angle_min)*ptCldFloor.ranges[i],inBaseFrame.z, min((int)(ptCldFloor.intensities[i]/intensityRange),255),255,min((int)(ptCldFloor.intensities[i]/intensityRange),255));
	}

    pLocalRenderer->unlock();
}

void Vis3d::addStereoCloud()
{
	libTF::TFPose aPose;
	aPose.x = 0;
	aPose.y = 0;
	aPose.z = 0;
	aPose.roll = 0;
	aPose.pitch = 0;
	aPose.yaw = 0;
	aPose.time = 0;
	aPose.frame = this->tfClient.lookup("FRAMEID_STEREO_BLOCK"); //TODO: put me in pr2.xml and change my string
	libTF::TFPose inBaseFrame;
	try
	{
		inBaseFrame = this->tfClient.transformPose("base", aPose);
	}
	catch(libTF::TransformReference::LookupException e)
	{
		inBaseFrame.x = 0;
		inBaseFrame.y = 0;
		inBaseFrame.z = 0;
		inBaseFrame.roll = 0;
		inBaseFrame.pitch = 0;
		inBaseFrame.yaw = 0;
		inBaseFrame.time = 0;
		inBaseFrame.frame = this->tfClient.lookup("base");
	}
    pLocalRenderer->lock();
    /*for(int i = 0; i < ilStereoCloud.size(); i++)
    {
    	ilStereoCloud[i]->resetCount();
    }*/
    ilStereoCloud->resetCount();
    //if(ilStereoCloud.size() == 0)
    	//ilStereoCloud.push_back(new ILPointCloud(pLocalRenderer->manager()->getRootSceneNode(),pLocalRenderer->manager(),-1));
	for(int i = 0; i < ptCldStereo.get_pts_size(); i++)
	{
		/*if(((float)i)/((float)(stereoVertScanCount + 1)) > 65535)
		{
			stereoVertScanCount++;
			if(stereoVertScanCount == ilHeadCloud.size())
			{
				ilStereoCloud.push_back(new ILPointCloud(pLocalRenderer->manager()->getRootSceneNode(),pLocalRenderer->manager(),-1));
				pLocalRenderer->unlock();
				pLocalRenderer->addNode(ilStereoCloud[stereoVertScanCount]);
				pLocalRenderer->lock();
			}
		}*/
		//ilStereoCloud[stereoVertScanCount]->addPoint(inBaseFrame.x + ptCldStereo.pts[i].x, inBaseFrame.y + ptCldStereo.pts[i].y, inBaseFrame.z + ptCldStereo.pts[i].z, min((int)(ptCldStereo.chan[0].vals[i]),255),min((int)(ptCldStereo.chan[0].vals[i]),255),255);
		ilStereoCloud->addPoint(inBaseFrame.x + ptCldStereo.pts[i].x, inBaseFrame.y + ptCldStereo.pts[i].y, inBaseFrame.z + ptCldStereo.pts[i].z, min((int)(ptCldStereo.chan[0].vals[i]),255),min((int)(ptCldStereo.chan[0].vals[i]),255),255);
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
		//case Replace:
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
	//std::cout << "New Transform!\n";
	//for(int i = 0; i < PR2::PR2_FRAMEID_COUNT; i++)
	for( map<std::string, ILModel*>::iterator iter = m_modelMap.begin(); iter != m_modelMap.end(); iter++ )
	{
		//std::cout << "i = " << i << std::endl;
		//if (i != 21 && i != 22 && i != 30 && i != 31 && i < 31){
			libTF::TFPose aPose;
			aPose.x = 0;
			aPose.y = 0;
			aPose.z = 0;
			aPose.roll = 0;
			aPose.pitch = 0;
			aPose.yaw = 0;
			aPose.time = 0;
			aPose.frame = this->tfClient.lookup((*iter).first);
			libTF::TFPose inBaseFrame;
			try
			{
				inBaseFrame = this->tfClient.transformPose("base", aPose);
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
				inBaseFrame.frame = this->tfClient.lookup("base");
			}
			if((*iter).second != NULL)
			{
				(*iter).second->setPosition((float)inBaseFrame.x,(float)inBaseFrame.y, (float)inBaseFrame.z);
				(*iter).second->setRotation((float)inBaseFrame.roll,(float)(inBaseFrame.pitch), (float)(inBaseFrame.yaw));
			}
		//}
	}
}

/**Displays objects (arrows, cubes, spheres, text, and meshes) with user controlled parameters.  Check enums in header and VisualizationMarker.msg for details*/
void Vis3d::newMarker()
{
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

