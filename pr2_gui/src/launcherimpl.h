#ifndef LAUNCHERIMPL_H
#define LAUNCHERIMPL_H

//Other stuff
#include <cstdio>
//#include <vector>
#include "image_utils/image_codec.h"
//#include "opencv/cxcore.h"
//#include "opencv/cv.h"
//#include "opencv/highgui.h"
//GUI stuff
#include <QMainWindow>
#include "ui_launcher.h"
#include "Vis3d.hh"

//ros stuff
#include "ros/node.h"
#include "std_msgs/Image.h"


class LauncherImpl : public QMainWindow, public Ui::Launcher
{
Q_OBJECT
public:
	ros::node *myNode;
	std_msgs::Image PTZLImage;
	std_msgs::Image PTZRImage;
	std_msgs::Image wristLImage;
	std_msgs::Image wristRImage;
	ImageCodec<std_msgs::Image> *codec;
	QButtonGroup *viewGroup;
	LauncherImpl( QWidget * parent = 0, Qt::WFlags f = 0 );
	void consoleOut(QString line);
	Vis3d *vis3d_Window;
	void incomingPTZLImageConn();
	void incomingPTZRImageConn();
	void incomingWristLImageConn();
	void incomingWristRImageConn();
	
	enum viewEnum{Maya,FPS,TFL,TFR,TRL,TRR,Top,Bottom,Front,Rear,Left,Right};
	
private slots:
	void startStop_Visualization( bool checked );
	void startStop_PTZL( bool checked);
	void startStop_PTZR( bool checked);
	void startStop_WristL( bool checked);
	void startStop_WristR( bool checked);
	void startStop_Stereo( bool checked);
	void startStop_Status( bool checked);
	void startStop_Topdown( bool checked);
	
	void visualizationClosing(bool vis);
	void PTZLClosing(bool vis);
	void PTZRClosing(bool vis);
	void WristLClosing(bool vis);
	void WristRClosing(bool vis);
	void StereoClosing(bool vis);
	void StatusClosing(bool vis);
	void TopdownClosing(bool vis);
	
	void startStopHeadPtCld( bool checked );
	void startStopFloorPtCld( bool checked );
	void startStopStereoPtCld( bool checked );
	void startStopModel( bool checked );
	
	void incomingPTZLImage();
	void incomingPTZRImage();
	void incomingWristLImage();
	void incomingWristRImage();

	void viewChanged(int id);
	
signals:
	incomingPTZLImageSig();
	incomingPTZRImageSig();
	incomingWristLImageSig();
	incomingWristRImageSig();
};
#endif




