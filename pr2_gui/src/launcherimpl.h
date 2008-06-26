#ifndef LAUNCHERIMPL_H
#define LAUNCHERIMPL_H

//Other stuff
#include <cstdio>
//#include <vector>
//#include "image_utils/image_codec.h"
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
#include "std_msgs/PTZActuatorCmd.h"


class LauncherImpl : public QMainWindow, public Ui::Launcher
{
Q_OBJECT
public:
	ros::node *myNode;
	std_msgs::Image PTZLImage;
	std_msgs::Image PTZRImage;
	std_msgs::Image WristLImage;
	std_msgs::Image WristRImage;
	std_msgs::PTZActuatorCmd ptz_cmd;
	QButtonGroup *viewGroup;
	LauncherImpl( QWidget * parent = 0, Qt::WFlags f = 0 );
	void consoleOut(QString line);
	Vis3d *vis3d_Window;
	void incomingPTZLImageConn();
	void incomingPTZRImageConn();
	void incomingWristLImageConn();
	void incomingWristRImageConn();
	
	uint8_t *PTZLImageData;
	uint8_t *PTZRImageData;
	uint8_t *WristLImageData;
	uint8_t *WristRImageData;
	
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
	void startStopUCS( bool checked );
	void startStopGrid( bool checked );
	
	void incomingPTZLImage(QPixmap *im, uint8_t **data, uint len);
	void incomingPTZRImage(QPixmap *im, uint8_t **data, uint len);
	void incomingWristLImage(QPixmap *im, uint8_t **data, uint len);
	void incomingWristRImage(QPixmap *im, uint8_t **data, uint len);

	void viewChanged(int id);
	
	void PTZL_ptzChanged(int unused);
	
signals:
	void incomingPTZLImageSig(QPixmap *im, uint8_t **data, uint len);
	void incomingPTZRImageSig(QPixmap *im, uint8_t **data, uint len);
	void incomingWristLImageSig(QPixmap *im, uint8_t **data, uint len);
	void incomingWristRImageSig(QPixmap *im, uint8_t **data, uint len);
};
#endif




