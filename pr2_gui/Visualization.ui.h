/****************************************************************************
** ui.h extension file, included from the uic-generated form implementation.
**
** If you want to add, delete, or rename functions or slots, use
** Qt Designer to update this file, preserving your code.
**
** You should not define a constructor or destructor in this file.
** Instead, write your code in functions called init() and destroy().
** These will automatically be called by the form's constructor and
** destructor.
*****************************************************************************/

void Visualization::death()
{
    destroy();
}

void Visualization::init()
{  
    vis3d_window = new Vis3d();
}


void Visualization::startStopHeadPtCld( int state )
{
    if(state == QButton::On)
    {
	emit ConsoleOut("Enabling Head Laser Cloud");
	Vis3d->enableHead();
    }
    else
    {
	emit ConsoleOut("Disabling Head Laser Cloud");
	Vis3d->disableHead();
    }
}


void Visualization::startStopFloorPtCld( int state )
{
      if(state == QButton::On)
    {
	emit ConsoleOut("Enabling Floor Laser Cloud");
	Vis3d->enableFloor();
    }
    else
    {
	emit ConsoleOut("Disabling Floor Laser Cloud");
	Vis3d->disableFloor();
    }
}


void Visualization::startStopStereoPtCld( int state )
{
     if(state == QButton::On)
    {
	emit ConsoleOut("Enabling Stereo Laser Cloud");
	Vis3d->enableStereo();
    }
    else
    {
	emit ConsoleOut("Disabling Stereo Laser Cloud");
	Vis3d->disableStereo();
    }
}

void ConsoleOut(QString line){}
