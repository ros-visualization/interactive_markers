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
		std::cout << "closed\n";
    }
}

void LauncherImpl::startStop_PTZL( bool checked)
{
	if(checked)
	{
		consoleOut("Opening Left Pan-Tilt-Zoom");
		PTZL_DW->setVisible(true);
		QObject::connect(this, SIGNAL(incomingPTZLImageSig()),this, SLOT(incomingPTZLImage()),Qt::QueuedConnection);
		myNode->subscribe("PTZL_image", PTZLImage, &LauncherImpl::incomingPTZLImageConn,this);
	}
	else
	{
		consoleOut("Closing Left Pan-Tilt-Zoom");
		PTZL_DW->setVisible(false);
		myNode->unsubscribe("PTZL_image");
		//QObject::disconnect(PTZL_DW, SIGNAL(visibilityChanged(bool)),this, SLOT(PTZLClosing(bool)));
		QObject::disconnect(this, SIGNAL(incomingPTZLImageSig()),this, SLOT(incomingPTZLImage()));
	}
}

void LauncherImpl::startStop_PTZR( bool checked)
{
	if(checked)
	{
		consoleOut("Opening Right Pan-Tilt-Zoom");
		PTZR_DW->setVisible(true);
		QObject::connect(this, SIGNAL(incomingPTZRImageSig()),this, SLOT(incomingPTZRImage()),Qt::QueuedConnection);
		myNode->subscribe("PTZR_image", PTZRImage, &LauncherImpl::incomingPTZRImageConn,this);
	}
	else
	{
		consoleOut("Closing Right Pan-Tilt-Zoom");
		PTZR_DW->setVisible(false);
		myNode->unsubscribe("PTZR_image");
		//QObject::disconnect(PTZR_DW, SIGNAL(visibilityChanged(bool)),this, SLOT(PTZRClosing(bool)));
		QObject::disconnect(this, SIGNAL(incomingPTZRImageSig()),this, SLOT(incomingPTZRImage()));
	}
}

void LauncherImpl::startStop_WristL( bool checked)
{
	if(checked)
	{
		consoleOut("Opening Left Wrist");
		WristL_DW->setVisible(true);
		QObject::connect(this, SIGNAL(incomingWristLImageSig()),this, SLOT(incomingWristLImage()),Qt::QueuedConnection);
		myNode->subscribe("WristL_image", wristLImage, &LauncherImpl::incomingWristLImageConn,this);
	}
	else
	{
		consoleOut("Closing Left Wrist");
		WristL_DW->setVisible(false);
		myNode->unsubscribe("WristL_image");
		//QObject::disconnect(WristL_DW, SIGNAL(visibilityChanged(bool)),this, SLOT(WristLClosing(bool)));
		QObject::disconnect(this, SIGNAL(incomingWristLImageSig()),this, SLOT(incomingWristLImage()));
	}
}

void LauncherImpl::startStop_WristR( bool checked)
{
	if(checked)
	{
		consoleOut("Opening Right Wrist");
		WristR_DW->setVisible(true);
		QObject::connect(this, SIGNAL(incomingWristRImageSig()),this, SLOT(incomingWristRImage()),Qt::QueuedConnection);
		myNode->subscribe("WristR_image", wristRImage, &LauncherImpl::incomingWristRImageConn,this);
	}
	else
	{
		consoleOut("Closing Right Wrist");
		WristR_DW->setVisible(false);
		//QObject::disconnect(WristR_DW, SIGNAL(visibilityChanged(bool)),this, SLOT(WristRClosing(bool)));
		QObject::disconnect(this, SIGNAL(incomingWristRImageSig()),this, SLOT(incomingWristRImage()));
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

void LauncherImpl::incomingPTZLImage()
{
  QPixmap *im = new QPixmap();
  if(im->loadFromData(PTZLImage.data,PTZLImage.get_data_size()))
  {
  	PTZL_IL->setPixmap(*im);
  }
  else
  {
  	consoleOut("Can't load left PTZ image");
  }
  delete im;
}

void LauncherImpl::incomingPTZLImageConn()
{
	emit incomingPTZLImageSig();
}

void LauncherImpl::incomingPTZRImage()
{
  QPixmap *im = new QPixmap();
  if(im->loadFromData(PTZRImage.data,PTZRImage.get_data_size()))
  {
  	PTZR_IL->setPixmap(*im);
  }
  else
  {
  	consoleOut("Can't load right PTZ image");
  }
  delete im;
}

void LauncherImpl::incomingPTZRImageConn()
{
	emit incomingPTZRImageSig();
}

void LauncherImpl::incomingWristLImage()
{
  QPixmap *im = new QPixmap();
  if(im->loadFromData(wristLImage.data,wristLImage.get_data_size()))
  {
  	WristL_IL->setPixmap(*im);
  }
  else
  {
  	consoleOut("Can't load left wrist image");
  }
  delete im;
}

void LauncherImpl::incomingWristLImageConn()
{
	emit incomingWristLImageSig();
}

void LauncherImpl::incomingWristRImage()
{
  QPixmap *im = new QPixmap();
  if(im->loadFromData(wristRImage.data,wristRImage.get_data_size()))
  {
  	WristR_IL->setPixmap(*im);
  }
  else
  {
  	consoleOut("Can't load right wrist image");
  }
  delete im;
}

void LauncherImpl::incomingWristRImageConn()
{
	emit incomingWristRImageSig();
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
//
