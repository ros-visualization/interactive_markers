#include "launcherimpl.h"
//#include <iostream>
//

LauncherImpl::LauncherImpl( QWidget * parent, Qt::WFlags f) : QMainWindow(parent, f)
{
	printf("setting up");
	//Initial setup
	setupUi(this);
	Visualization_DW->setVisible(false);
	PTZL_DW->setVisible(false);
	PTZR_DW->setVisible(false);
	WristL_DW->setVisible(false);
	WristR_DW->setVisible(false);
	Stereo_DW->setVisible(false);
	Status_DW->setVisible(false);
	Topdown_DW->setVisible(false);

	viewGroup = new QButtonGroup(Views_GB);
	viewGroup->addButton(ViewMaya_RB, Maya);
	viewGroup->addButton(ViewFPS_RB, FPS);
	viewGroup->addButton(ViewTFL_RB, TFL);
	viewGroup->addButton(ViewTFR_RB, TFR);
	viewGroup->addButton(ViewTRL_RB, TRL);
	viewGroup->addButton(ViewTRR_RB, TRR);
	viewGroup->addButton(ViewF_RB, Front);
	viewGroup->addButton(ViewR_RB, Rear);
	viewGroup->addButton(ViewT_RB, Top);
	viewGroup->addButton(ViewB_RB, Bottom);
	viewGroup->addButton(ViewD_RB, Right);
	viewGroup->addButton(ViewS_RB, Left);
	
	//Docked window button connections
	QObject::connect(Visualization_CB, SIGNAL(toggled(bool)),this, SLOT(startStop_Visualization(bool)));
	QObject::connect(PTZL_CB, SIGNAL(toggled(bool)),this, SLOT(startStop_PTZL(bool)));
	QObject::connect(PTZR_CB, SIGNAL(toggled(bool)),this, SLOT(startStop_PTZR(bool)));
	QObject::connect(WristL_CB, SIGNAL(toggled(bool)),this, SLOT(startStop_WristL(bool)));
	QObject::connect(WristR_CB, SIGNAL(toggled(bool)),this, SLOT(startStop_WristR(bool)));
	QObject::connect(Stereo_CB, SIGNAL(toggled(bool)),this, SLOT(startStop_Stereo(bool)));
	QObject::connect(Status_CB, SIGNAL(toggled(bool)),this, SLOT(startStop_Status(bool)));
	QObject::connect(Topdown_CB, SIGNAL(toggled(bool)),this, SLOT(startStop_Topdown(bool)));
	//Docked window opening/closing
	QObject::connect(PTZL_DW, SIGNAL(visibilityChanged(bool)),this, SLOT(PTZLClosing(bool)));
	QObject::connect(PTZR_DW, SIGNAL(visibilityChanged(bool)),this, SLOT(PTZRClosing(bool)));
	QObject::connect(WristL_DW, SIGNAL(visibilityChanged(bool)),this, SLOT(WristLClosing(bool)));
	QObject::connect(WristR_DW, SIGNAL(visibilityChanged(bool)),this, SLOT(WristRClosing(bool)));
	//Visualization extras
	QObject::connect(viewGroup, SIGNAL(buttonClicked(int)),this, SLOT(viewChanged(int)));

	vis3d_Window = 0;
	myNode = new ros::node("guiNode");
	printf("set up");
}

void LauncherImpl::consoleOut(QString line)
{
    Console_TE->append(line);
}

void LauncherImpl::startStop_Visualization( bool checked )
{
    if(checked)
    {
		consoleOut("Opening Visualizer");
		std::cout << "Opening Visualizer\n";
		HeadLaser_CB->setChecked(false);
		FloorLaser_CB->setChecked(false);
		StereoCloud_CB->setChecked(false);
		Model_CB->setChecked(false);
		Visualization_DW->setVisible(true);
		QObject::connect(Visualization_DW, SIGNAL(visibilityChanged(bool)),this, SLOT(visualizationClosing(bool)));
		QObject::connect(HeadLaser_CB, SIGNAL(toggled(bool)),this,SLOT(startStopHeadPtCld(bool)));
		QObject::connect(FloorLaser_CB, SIGNAL(toggled(bool)),this,SLOT(startStopFloorPtCld(bool)));
		QObject::connect(StereoCloud_CB, SIGNAL(toggled(bool)),this,SLOT(startStopStereoPtCld(bool)));
		QObject::connect(Model_CB, SIGNAL(toggled(bool)),this,SLOT(startStopModel(bool)));
		QObject::connect(UCS_CB, SIGNAL(toggled(bool)),this,SLOT(startStopUCS(bool)));
		QObject::connect(Grid_CB, SIGNAL(toggled(bool)),this,SLOT(startStopGrid(bool)));
		vis3d_Window = new Vis3d(myNode);
    }
    else
    {
		consoleOut("Closing Visualizer");
		delete vis3d_Window;
		vis3d_Window = 0;
		std::cout << "almost closed\n";
		Visualization_DW->setVisible(false);
		QObject::disconnect(Visualization_DW, SIGNAL(visibilityChanged(bool)),this, SLOT(visualizationClosing(bool)));
		QObject::disconnect(HeadLaser_CB, SIGNAL(toggled(bool)),this,SLOT(startStopHeadPtCld(bool)));
		QObject::disconnect(FloorLaser_CB, SIGNAL(toggled(bool)),this,SLOT(startStopFloorPtCld(bool)));
		QObject::disconnect(StereoCloud_CB, SIGNAL(toggled(bool)),this,SLOT(startStopStereoPtCld(bool)));
		QObject::disconnect(Model_CB, SIGNAL(toggled(bool)),this,SLOT(startStopModel(bool)));
		QObject::disconnect(UCS_CB, SIGNAL(toggled(bool)),this,SLOT(startStopUCS(bool)));
		QObject::disconnect(Grid_CB, SIGNAL(toggled(bool)),this,SLOT(startStopGrid(bool)));
		std::cout << "closed\n";
    }
}

void LauncherImpl::startStop_PTZL( bool checked)
{
	if(checked)
	{
		consoleOut("Opening Left Pan-Tilt-Zoom");
		PTZL_DW->setVisible(true);
		QObject::connect(this, SIGNAL(incomingPTZLImageSig(QPixmap*, uint8_t**, uint)),this, SLOT(incomingPTZLImage(QPixmap*, uint8_t**, uint)),Qt::QueuedConnection);
		QObject::connect(panPTZL_S, SIGNAL(valueChanged(int)),this,SLOT(PTZL_ptzChanged(int)));
		QObject::connect(tiltPTZL_S, SIGNAL(valueChanged(int)),this,SLOT(PTZL_ptzChanged(int)));
		QObject::connect(zoomPTZL_S, SIGNAL(valueChanged(int)),this,SLOT(PTZL_ptzChanged(int)));
		myNode->subscribe("PTZL_image", PTZLImage, &LauncherImpl::incomingPTZLImageConn,this);
		myNode->advertise<std_msgs::PTZActuatorCmd>("PTZL_cmd");
	}
	else
	{
		consoleOut("Closing Left Pan-Tilt-Zoom");
		PTZL_DW->setVisible(false);
		myNode->unsubscribe("PTZL_image");
		//QObject::disconnect(PTZL_DW, SIGNAL(visibilityChanged(bool)),this, SLOT(PTZLClosing(bool)));
		QObject::disconnect(this, SIGNAL(incomingPTZLImageSig(QPixmap*, uint8_t**, uint)),this, SLOT(incomingPTZLImage(QPixmap*, uint8_t**, uint)));
	}
}

void LauncherImpl::startStop_PTZR( bool checked)
{
	if(checked)
	{
		consoleOut("Opening Right Pan-Tilt-Zoom");
		PTZR_DW->setVisible(true);
		QObject::connect(this, SIGNAL(incomingPTZRImageSig(QPixmap*, uint8_t**, uint)),this, SLOT(incomingPTZRImage(QPixmap*, uint8_t**, uint)),Qt::QueuedConnection);
		myNode->subscribe("PTZR_image", PTZRImage, &LauncherImpl::incomingPTZRImageConn,this);
	}
	else
	{
		consoleOut("Closing Right Pan-Tilt-Zoom");
		PTZR_DW->setVisible(false);
		myNode->unsubscribe("PTZR_image");
		//QObject::disconnect(PTZR_DW, SIGNAL(visibilityChanged(bool)),this, SLOT(PTZRClosing(bool)));
		QObject::disconnect(this, SIGNAL(incomingPTZRImageSig(QPixmap*, uint8_t**, uint)),this, SLOT(incomingPTZRImage(QPixmap*, uint8_t**, uint)));
	}
}

void LauncherImpl::startStop_WristL( bool checked)
{
	if(checked)
	{
		consoleOut("Opening Left Wrist");
		WristL_DW->setVisible(true);
		QObject::connect(this, SIGNAL(incomingWristLImageSig(QPixmap*, uint8_t**, uint)),this, SLOT(incomingWristLImage(QPixmap*, uint8_t**, uint)),Qt::QueuedConnection);
		myNode->subscribe("WristL_image", WristLImage, &LauncherImpl::incomingWristLImageConn,this);
	}
	else
	{
		consoleOut("Closing Left Wrist");
		WristL_DW->setVisible(false);
		myNode->unsubscribe("WristL_image");
		//QObject::disconnect(WristL_DW, SIGNAL(visibilityChanged(bool)),this, SLOT(WristLClosing(bool)));
		QObject::disconnect(this, SIGNAL(incomingWristLImageSig(QPixmap*, uint8_t**, uint)),this, SLOT(incomingWristLImage(QPixmap*, uint8_t**, uint)));
	}
}

void LauncherImpl::startStop_WristR( bool checked)
{
	if(checked)
	{
		consoleOut("Opening Right Wrist");
		WristR_DW->setVisible(true);
		QObject::connect(this, SIGNAL(incomingWristRImageSig(QPixmap*, uint8_t**, uint)),this, SLOT(incomingWristRImage(QPixmap*, uint8_t**, uint)),Qt::QueuedConnection);
		myNode->subscribe("WristR_image", WristRImage, &LauncherImpl::incomingWristRImageConn,this);
	}
	else
	{
		consoleOut("Closing Right Wrist");
		WristR_DW->setVisible(false);
		//QObject::disconnect(WristR_DW, SIGNAL(visibilityChanged(bool)),this, SLOT(WristRClosing(bool)));
		QObject::disconnect(this, SIGNAL(incomingWristRImageSig(QPixmap*, uint8_t**, uint)),this, SLOT(incomingWristRImage(QPixmap*, uint8_t**, uint)));
	}
}

void LauncherImpl::startStop_Stereo( bool checked)
{
	if(checked)
	{
		consoleOut("Opening Stereo");
		Stereo_DW->setVisible(true);
		QObject::connect(Stereo_DW, SIGNAL(visibilityChanged(bool)),this, SLOT(StereoClosing(bool)));
	}
	else
	{
		consoleOut("Closing Stereo");
		Stereo_DW->setVisible(false);
	}
}

void LauncherImpl::startStop_Status( bool checked)
{
	if(checked)
	{
		consoleOut("Opening Status");
		Status_DW->setVisible(true);
		QObject::connect(Status_DW, SIGNAL(visibilityChanged(bool)),this, SLOT(StatusClosing(bool)));
	}
	else
	{
		consoleOut("Closing Status");
		Status_DW->setVisible(false);
	}
}

void LauncherImpl::startStop_Topdown( bool checked)
{
	if(checked)
	{
		consoleOut("Opening Topdown");
		Topdown_DW->setVisible(true);
		QObject::connect(Topdown_DW, SIGNAL(visibilityChanged(bool)),this, SLOT(TopdownClosing(bool)));
	}
	else
	{
		consoleOut("Closing Topdown");
		Topdown_DW->setVisible(false);
	}
}

void LauncherImpl::visualizationClosing(bool vis)
{
	if (!vis)
		Visualization_CB->setChecked(false);
}

void LauncherImpl::PTZLClosing(bool vis)
{
	if (!vis)
		PTZL_CB->setChecked(false);
}

void LauncherImpl::PTZRClosing(bool vis)
{
	if (!vis)
		PTZR_CB->setChecked(false);
}

void LauncherImpl::WristLClosing(bool vis)
{
	if (!vis)
		WristL_CB->setChecked(false);
}

void LauncherImpl::WristRClosing(bool vis)
{
	if (!vis)
		WristR_CB->setChecked(false);
}

void LauncherImpl::StereoClosing(bool vis)
{
	if (!vis)
		Stereo_CB->setChecked(false);
}

void LauncherImpl::StatusClosing(bool vis)
{
	if (!vis)
		Status_CB->setChecked(false);
}

void LauncherImpl::TopdownClosing(bool vis)
{
	if (!vis)
		Topdown_CB->setChecked(false);
}

void LauncherImpl::startStopHeadPtCld( bool checked )
{
    if(checked)
    {
		consoleOut("Enabling Head Laser Cloud");
		vis3d_Window->enableHead();
    }
    else
    {
		consoleOut("Disabling Head Laser Cloud");
		vis3d_Window->disableHead();
    }
}

void LauncherImpl::startStopFloorPtCld( bool checked )
{
    if(checked)
    {
		consoleOut("Enabling Floor Laser Cloud");
		vis3d_Window->enableFloor();
    }
    else
    {
		consoleOut("Disabling Floor Laser Cloud");
		vis3d_Window->disableFloor();
    }
}

void LauncherImpl::startStopStereoPtCld( bool checked )
{
    if(checked)
    {
		consoleOut("Enabling Stereo Laser Cloud");
		vis3d_Window->enableStereo();
    }
    else
    {
		consoleOut("Disabling Stereo Laser Cloud");
		vis3d_Window->disableStereo();
    }
}

void LauncherImpl::startStopModel( bool checked )
{
    if(checked)
    {
		consoleOut("Enabling 3D Model");
		vis3d_Window->enableModel();
    }
    else
    {
		consoleOut("Disabling 3D Model");
		vis3d_Window->disableModel();
    }
}

void LauncherImpl::startStopUCS( bool checked )
{
	if(checked)
	{
		consoleOut("Enabling UCS");
		vis3d_Window->enableUCS();
	}
	else
	{
		consoleOut("Disabling UCS");
		vis3d_Window->disableUCS();
	}
}

void LauncherImpl::startStopGrid( bool checked )
{
	if(checked)
	{
		consoleOut("Enabling Grid");
		vis3d_Window->enableGrid();
	}
	else
	{
		consoleOut("Disabling Grid");
		vis3d_Window->disableGrid();
	}
}

//main thread equivalent of the below callback
void LauncherImpl::incomingPTZLImage(QPixmap *im, uint8_t **data, uint len)
{
  if(im->loadFromData(*data,len,"jpeg"))
  {
  	PTZL_IL->setPixmap(*im);
  }
  else
  {
  	consoleOut("Can't load left PTZ image");
  }
  delete im;
  delete *data;
  *data = NULL;
}

//ros callback
void LauncherImpl::incomingPTZLImageConn()
{
	if(!PTZLImageData)
	{
		QPixmap *im = new QPixmap();
		const uint32_t count = PTZLImage.get_data_size();
		PTZLImageData = new uint8_t[count];
		memcpy(PTZLImageData, PTZLImage.data, sizeof(uint8_t) * count);
		emit incomingPTZLImageSig(im, &PTZLImageData, PTZLImage.get_data_size());
	}
}

void LauncherImpl::incomingPTZRImage(QPixmap *im, uint8_t **data, uint len)
{
  if(im->loadFromData(*data,len,"jpeg"))
  {
  	PTZR_IL->setPixmap(*im);
  }
  else
  {
  	consoleOut("Can't load right PTZ image");
  }
  delete im;
  delete *data;
  *data = NULL;
}

void LauncherImpl::incomingPTZRImageConn()
{
	if(!PTZRImageData)
	{
		QPixmap *im = new QPixmap();
		const uint32_t count = PTZRImage.get_data_size();
		PTZRImageData = new uint8_t[count];
		memcpy(PTZRImageData, PTZRImage.data, sizeof(uint8_t) * count);
		emit incomingPTZRImageSig(im, &PTZRImageData, PTZRImage.get_data_size());
	}
}

void LauncherImpl::incomingWristLImage(QPixmap *im, uint8_t **data, uint len)
{
  if(im->loadFromData(*data,len,"jpeg"))
  {
  	WristL_IL->setPixmap(*im);
  }
  else
  {
  	consoleOut("Can't load left wrist image");
  }
  delete im;
  delete *data;
  *data = NULL;
}

void LauncherImpl::incomingWristLImageConn()
{
	if(!WristLImageData)
	{
		QPixmap *im = new QPixmap();
		const uint32_t count = WristLImage.get_data_size();
		WristLImageData = new uint8_t[count];
		memcpy(WristLImageData, WristLImage.data, sizeof(uint8_t) * count);
		emit incomingWristLImageSig(im, &WristLImageData, WristLImage.get_data_size());
	}
}

void LauncherImpl::incomingWristRImage(QPixmap *im, uint8_t **data, uint len)
{
  if(im->loadFromData(*data,len,"jpeg"))
  {
  	WristR_IL->setPixmap(*im);
  }
  else
  {
  	consoleOut("Can't load right wrist image");
  }
  delete im;
  delete *data;
  *data = NULL;
}

void LauncherImpl::incomingWristRImageConn()
{
	if(!WristRImageData)
	{
		QPixmap *im = new QPixmap();
		const uint32_t count = WristRImage.get_data_size();
		WristRImageData = new uint8_t[count];
		memcpy(WristRImageData, WristRImage.data, sizeof(uint8_t) * count);
		emit incomingWristRImageSig(im, &WristRImageData, WristRImage.get_data_size());
	}
}

void LauncherImpl::viewChanged(int id)
{
	if(vis3d_Window)
	{
		//std::cout << "changing view to " << id << std::endl;
		vis3d_Window->changeView(id);
	}
	else
	{
		consoleOut("Cannot change view.  3D window does not exist.");
	}
}

void LauncherImpl::PTZL_ptzChanged(int unused)
{
	ptz_cmd.pan.valid = 1;
	ptz_cmd.pan.cmd = panPTZL_S->value();
	ptz_cmd.tilt.valid = 1;
	ptz_cmd.tilt.cmd = tiltPTZL_S->value();
	ptz_cmd.zoom.valid = 1;
	ptz_cmd.zoom.cmd = zoomPTZL_S->value();
	myNode->publish("PTZL_cmd",ptz_cmd);
}
	
//
