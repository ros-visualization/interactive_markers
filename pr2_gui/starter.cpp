#include "starter.h"
#include "launcherimpl.h"


IMPLEMENT_APP(Starter)

Starter::Starter()
{
	int a = 0;
	ros::init(a,NULL);
}

Starter::~Starter()
{
	ros::fini();
}

bool Starter::OnInit()
{
    LauncherImpl* dialog = new LauncherImpl( (wxWindow*)NULL );
    dialog ->Show();
    SetTopWindow( dialog );
    return true;
}
