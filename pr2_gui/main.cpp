#include <qapplication.h>
#include "Launcher.h"

int main( int argc, char ** argv )
{
    ros::init(argc,argv);
    QApplication a( argc, argv );
    Launcher w;
    w.show();
    a.connect( &a, SIGNAL( lastWindowClosed() ), &a, SLOT( quit() ) );
    int ret = a.exec();
    ros::fini();
    return ret;
}
