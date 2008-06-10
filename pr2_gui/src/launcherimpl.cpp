#include "launcherimpl.h"
//#include <iostream>
//
ros::node *myNode;
LauncherImpl::LauncherImpl( QWidget * parent, Qt::WFlags f) : QMainWindow(parent, f)
{
	setupUi(this);
	Visualization_DW->setVisible(false);
	PTZL_DW->setVisible(false);
	PTZR_DW->setVisible(false);
	WristL_DW->setVisible(false);
	WristR_DW->setVisible(false);
	Stereo_DW->setVisible(false);
	Status_DW->setVisible(false);
	Topdown_DW->setVisible(false);
	
	QObject::connect(Visualization_CB, SIGNAL(toggled(bool)),this, SLOT(startStop_Visualization(bool)));
	QObject::connect(PTZL_CB, SIGNAL(toggled(bool)),this, SLOT(startStop_PTZL(bool)));
	QObject::connect(PTZR_CB, SIGNAL(toggled(bool)),this, SLOT(startStop_PTZR(bool)));
	QObject::connect(WristL_CB, SIGNAL(toggled(bool)),this, SLOT(startStop_WristL(bool)));
	QObject::connect(WristR_CB, SIGNAL(toggled(bool)),this, SLOT(startStop_WristR(bool)));
	QObject::connect(Stereo_CB, SIGNAL(toggled(bool)),this, SLOT(startStop_Stereo(bool)));
	QObject::connect(Status_CB, SIGNAL(toggled(bool)),this, SLOT(startStop_Status(bool)));
	QObject::connect(Topdown_CB, SIGNAL(toggled(bool)),this, SLOT(startStop_Topdown(bool)));
	
	myNode = new ros::node("guiNode");
	//subscribe("roserr", 
	std::cout << "constructed\n";
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
		std::cout << "Opening Visualizer";
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
		Visualization_DW->setVisible(false);
		std::cout << "almost closed\n";
		delete vis3d_Window;
		std::cout << "closed\n";
    }
}

void LauncherImpl::startStop_PTZL( bool checked)
{
	if(checked)
	{
		consoleOut("Opening Left Pan-Tilt-Zoom");
		PTZL_DW->setVisible(true);
		QObject::connect(PTZL_DW, SIGNAL(visibilityChanged(bool)),this, SLOT(PTZLClosing(bool)));
	}
	else
	{
		consoleOut("Closing Left Pan-Tilt-Zoom");
		PTZL_DW->setVisible(false);
	}
}

void LauncherImpl::startStop_PTZR( bool checked)
{
	if(checked)
	{
		consoleOut("Opening Right Pan-Tilt-Zoom");
		PTZR_DW->setVisible(true);
		QObject::connect(PTZR_DW, SIGNAL(visibilityChanged(bool)),this, SLOT(PTZRClosing(bool)));
	}
	else
	{
		consoleOut("Closing Right Pan-Tilt-Zoom");
		PTZR_DW->setVisible(false);
	}
}

void LauncherImpl::startStop_WristL( bool checked)
{
	if(checked)
	{
		consoleOut("Opening Left Wrist");
		WristL_DW->setVisible(true);
		QObject::connect(WristL_DW, SIGNAL(visibilityChanged(bool)),this, SLOT(WristLClosing(bool)));
	}
	else
	{
		consoleOut("Closing Left Wrist");
		WristL_DW->setVisible(false);
	}
}

void LauncherImpl::startStop_WristR( bool checked)
{
	if(checked)
	{
		consoleOut("Opening Right Wrist");
		WristR_DW->setVisible(true);
		QObject::connect(WristR_DW, SIGNAL(visibilityChanged(bool)),this, SLOT(WristRClosing(bool)));
	}
	else
	{
		consoleOut("Closing Right Wrist");
		WristR_DW->setVisible(false);
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
//
