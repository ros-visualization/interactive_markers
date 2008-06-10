#ifndef LAUNCHERIMPL_H
#define LAUNCHERIMPL_H
//
#include <QMainWindow>
#include "ui_launcher.h"
#include "Vis3d.hh"
//ros stuff
#include "ros/node.h"

//
class LauncherImpl : public QMainWindow, public Ui::Launcher
{
Q_OBJECT
public:
	
	LauncherImpl( QWidget * parent = 0, Qt::WFlags f = 0 );
	void consoleOut(QString line);
	Vis3d *vis3d_Window;
	
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
};
#endif




