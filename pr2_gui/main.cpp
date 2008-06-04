#include <qapplication.h>
#include "Launcher.h"

int main( int argc, char ** argv )
{
    QApplication a( argc, argv );
    Launcher w;
    w.show();
    a.connect( &a, SIGNAL( lastWindowClosed() ), &a, SLOT( quit() ) );
    return a.exec();
}
