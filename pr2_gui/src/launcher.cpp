#include "launcher.h"


IMPLEMENT_APP(Launcher)

Launcher::Launcher()
{
}

Launcher::~Launcher()
{
}

bool Launcher::OnInit()
{
    LauncherImpl* dialog = new LauncherImpl( (wxWindow*)NULL );
    dialog ->Show();
    SetTopWindow( dialog );
    return true;
}
