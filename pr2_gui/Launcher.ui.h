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
void Launcher::init()
{
    Vis_Window = 0;
    PTZL_Window = 0;
    PTZR_Window = 0;
    Stereo_Window = 0;
    WristL_Window = 0;
    WristR_Window = 0;
    Topdown_Window = 0;
    Status_Window = 0;
}



void Launcher::destroy()
{
    if(Vis_Window)
	Vis_Window->death();
    if(Stereo_Window)
	Stereo_Window->death();
    if(PTZL_Window)
	PTZL_Window->death();
    if(PTZR_Window)
	PTZR_Window->death();
    if(WristL_Window)
	WristL_Window->death();
    if(WristR_Window)
	WristR_Window->death();
    if(Topdown_Window)
	Topdown_Window->death();
    if(Status_Window)
	Status_Window->death();
    this->close();
}

void Launcher::death()
{
    destroy();
}

void Launcher::helpIndex()
{

}


void Launcher::helpContents()
{

}


void Launcher::helpAbout()
{

}

void Launcher::consoleOut(QString line)
{
    Console_TE->append(line);
}

void Launcher::startStop_Visualization( int state )
{
    if(state == QButton::On)
    {
	Console_TE->append("Opening Visualizer");
	if(! Vis_Window)
	{
	    Vis_Window = new Visualization(this);
	    connect(Vis_Window,SIGNAL(ConsoleOut(QString)),this,SLOT(consoleOut(QString)));
	}
	else
	{
	    Console_TE->append("Visualizer already open");
	}
	Vis_Window->show();
    }
    else
    {
	Console_TE->append("Closing Visualizer");
	if(Vis_Window)
	{
	    Vis_Window->death();
	    Vis_Window = 0;
	}
	else
	{
	    Console_TE->append("Visualizer already closed");
	}
    }
}


void Launcher::startStop_PTZL( int state )
{
    if(state == QButton::On)
    {
	Console_TE->append("Opening PTZL");
	if(! PTZL_Window)
	{
	    PTZL_Window = new PTZ(this);
	}
	else
	{
	    Console_TE->append("PTZL already open");
	}
	PTZL_Window->show();
    }
    else
    {
	Console_TE->append("Closing PTZL");
	if(PTZL_Window)
	{
	    PTZL_Window->death();
	    PTZL_Window = 0;
	}
	else
	{
	    Console_TE->append("PTZL already closed");
	}
    }
}


void Launcher::startStop_PTZR( int state )
{
    if(state == QButton::On)
    {
	Console_TE->append("Opening PTZR");
	if(! PTZR_Window)
	{
	    PTZR_Window = new PTZ(this);
	}
	else
	{
	    Console_TE->append("PTZR already open");
	}
	PTZR_Window->show();
    }
    else
    {
	Console_TE->append("Closing PTZR");
	if(PTZR_Window)
	{
	    PTZR_Window->death();
	    PTZR_Window = 0;
	}
	else
	{
	    Console_TE->append("PTZR already closed");
	}
    }
}


void Launcher::startStop_WristL( int state )
{
    if(state == QButton::On)
    {
	Console_TE->append("Opening WristL");
	if(! WristL_Window)
	{
	    WristL_Window = new Wrist(this);
	}
	else
	{
	    Console_TE->append("WristL already open");
	}
	WristL_Window->show();
    }
    else
    {
	Console_TE->append("Closing WristL");
	if(WristL_Window)
	{
	    WristL_Window->death();
	    WristL_Window = 0;
	}
	else
	{
	    Console_TE->append("WristL already closed");
	}
    }
}


void Launcher::startStop_WristR( int state )
{
    if(state == QButton::On)
    {
	Console_TE->append("Opening WristR");
	if(! WristR_Window)
	{
	    WristR_Window = new Wrist(this);
	}
	else
	{
	    Console_TE->append("WristR already open");
	}
	WristR_Window->show();
    }
    else
    {
	Console_TE->append("Closing WristR");
	if(WristR_Window)
	{
	    WristR_Window->death();
	    WristR_Window = 0;
	}
	else
	{
	    Console_TE->append("WristR already closed"); 
	}
    }
}

void Launcher::startStop_Stereo( int state )
{
    if(state == QButton::On)
    {
	Console_TE->append("Opening Stereo");
	if(! Stereo_Window)
	{
	    Stereo_Window = new Stereo(this);
	}
	else
	{
	    Console_TE->append("Stereo already open");
	}
	Stereo_Window->show();
    }
    else
    {
	Console_TE->append("Closing Stereo");
	if(Stereo_Window)
	{
	    Stereo_Window->death();
	    Stereo_Window = 0;
	}
	else
	{
	    Console_TE->append("Stereo already closed");
	}
    }
}


void Launcher::startStop_Topdown( int state )
{
    if(state == QButton::On)
    {
	Console_TE->append("Opening Topdown");
	if(! Topdown_Window)
	{
	    Topdown_Window = new Topdown(this);
	}
	else
	{
	    Console_TE->append("Topdown already open");
	}
	Topdown_Window->show();
    }
    else
    {
	Console_TE->append("Closing Topdown");
	if(Topdown_Window)
	{
	    Topdown_Window->death();
	    Topdown_Window = 0;
	}
	else
	{
	    Console_TE->append("Topdown already closed");
	}
    }
}


void Launcher::startStop_Status( int state )
{
    if(state == QButton::On)
    {
	Console_TE->append("Opening Status");
	if(! Status_Window)
	{
	    Status_Window = new Status(this);
	}
	else
	{
	    Console_TE->append("Status already open");
	}
	Status_Window->show();
    }
    else
    {
	Console_TE->append("Closing Status");
	if(Status_Window)
	{
	    Status_Window->death();
	    Status_Window = 0;
	}
	else
	{
	    Console_TE->append("Status already closed");
	}
    }
}
