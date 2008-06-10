#include <QApplication>
#include "launcherimpl.h"
//
int main(int argc, char ** argv)
{
	ros::init(argc,argv);
	QApplication app( argc, argv );
	LauncherImpl win;
	win.show(); 
	app.connect( &app, SIGNAL( lastWindowClosed() ), &app, SLOT( quit() ) );
	int ret = app.exec();
	std::cout << "fini in main\n";
	ros::fini();
	return ret;
}
