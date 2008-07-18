#include "launcherimpl.h"

LauncherImpl::LauncherImpl( wxWindow* parent )
:
launcher( parent )
{

	wxInitAllImageHandlers();
	LeftDock_FGS->Hide(HeadLaser_RB,true);
	LeftDock_FGS->Hide(Visualization_SBS,true);
	LeftDock_FGS->Hide(PTZL_SBS,true);
	LeftDock_FGS->Hide(WristL_SBS,true);
	RightDock_FGS->Hide(Topdown_SBS,true);
	RightDock_FGS->Hide(PTZR_SBS,true);
	RightDock_FGS->Hide(WristR_SBS,true);
	Layout();
	Fit();
	
	
	PTZL_GET_NEW_IMAGE = true;
	PTZR_GET_NEW_IMAGE = true;
	WristR_GET_NEW_IMAGE = true;
	WristL_GET_NEW_IMAGE = true;
	vis3d_Window = NULL;
	myNode = new ros::node("guiNode");
	this->Connect(PTZL_B->GetId(), wxEVT_COMMAND_BUTTON_CLICKED, wxCommandEventHandler(LauncherImpl::PTZLDrawPic));
	this->Connect(PTZR_B->GetId(), wxEVT_COMMAND_BUTTON_CLICKED, wxCommandEventHandler(LauncherImpl::PTZRDrawPic));
	this->Connect(WristL_B->GetId(), wxEVT_COMMAND_BUTTON_CLICKED, wxCommandEventHandler(LauncherImpl::WristLDrawPic));
	this->Connect(WristR_B->GetId(), wxEVT_COMMAND_BUTTON_CLICKED, wxCommandEventHandler(LauncherImpl::WristRDrawPic));
}

void LauncherImpl::consoleOut(wxString Line)
{
    Console_TC->AppendText(Line);
}

void LauncherImpl::startStopHeadPtCld( wxCommandEvent& event )
{
	if(HeadLaser_CB->IsChecked())
    {
		consoleOut(wxT("Enabling Head Laser Cloud\n"));
		vis3d_Window->enableHead();
		HeadLaser_RB->Show(true);
		Layout();
		Fit();
		/*if(LeftDock_FGS->Show(HeadLaser_RB,true))
			std::cout << "found HeadLaser_RB show\n";
		Layout();
		Fit();*/
		//std::cout << "showing HeadLaser_RB\n";
    }
    else
    {
		consoleOut(wxT("Disabling Head Laser Cloud\n"));
		vis3d_Window->disableHead();
		HeadLaser_RB->Show(false);
		Layout();
		Fit();
		/*if(LeftDock_FGS->Show(HeadLaser_RB,true))
			std::cout << "found HeadLaser_RB hide\n";
		Layout();
		Fit();*/
    }
}

void LauncherImpl::startStopFloorPtCld( wxCommandEvent& event )
{
	if(FloorLaser_CB->IsChecked())
    {
		consoleOut(wxT("Enabling Floor Laser Cloud\n"));
		vis3d_Window->enableFloor();
    }
    else
    {
		consoleOut(wxT("Disabling Floor Laser Cloud\n"));
		vis3d_Window->disableFloor();
    }
}

void LauncherImpl::startStopStereoPtCld( wxCommandEvent& event )
{
	if(Stereo_CB->IsChecked())
    {
		consoleOut(wxT("Enabling Stereo Laser Cloud\n"));
		vis3d_Window->enableStereo();
    }
    else
    {
		consoleOut(wxT("Disabling Stereo Laser Cloud\n"));
		vis3d_Window->disableStereo();
    }
}

void LauncherImpl::startStopModel( wxCommandEvent& event )
{
	if(Model_CB->IsChecked())
    {
		consoleOut(wxT("Enabling 3D Model\n"));
		vis3d_Window->enableModel();
    }
    else
    {
		consoleOut(wxT("Disabling 3D Model\n"));
		vis3d_Window->disableModel();
    }
}

void LauncherImpl::startStopUCS( wxCommandEvent& event )
{
	if(UCS_CB->IsChecked())
	{
		consoleOut(wxT("Enabling UCS\n"));
		vis3d_Window->enableUCS();
	}
	else
	{
		consoleOut(wxT("Disabling UCS\n"));
		vis3d_Window->disableUCS();
	}
}

void LauncherImpl::startStopGrid( wxCommandEvent& event )
{
	if(Grid_CB->IsChecked())
	{
		consoleOut(wxT("Enabling Grid\n"));
		vis3d_Window->enableGrid();
	}
	else
	{
		consoleOut(wxT("Disabling Grid\n"));
		vis3d_Window->disableGrid();
	}
}

void LauncherImpl::startStopObjects( wxCommandEvent& event )
{
	if(Objects_CB->IsChecked())
	{
		consoleOut(wxT("Enabling Objects\n"));
		vis3d_Window->enableObjects();
	}
	else
	{
		consoleOut(wxT("Disabling Objects\n"));
		vis3d_Window->disableObjects();
	}
}

void LauncherImpl::viewChanged( wxCommandEvent& event )
{
	if(vis3d_Window)
	{
		//std::cout << "changing view to " << id << std::endl;
		vis3d_Window->changeView(Views_RB->GetSelection());
	}
	else
	{
		consoleOut(wxT("Cannot change view.  3D window does not exist.\n"));
	}
}

void LauncherImpl::HeadLaserChanged( wxCommandEvent& event )
{
	if(vis3d_Window)
	{
		std::cout << "Selection: " << HeadLaser_RB->GetSelection() << std::endl;
		//vis3d_Window->scanT = HeadLaser_RB->GetSelection();
		vis3d_Window->changeHeadLaser(HeadLaser_RB->GetSelection());
	}
	else
		consoleOut(wxT("Cannot change shutter type.  3D window does not exist.\n"));
}

void LauncherImpl::startStop_Visualization( wxCommandEvent& event )
{
	if(Visualization_CB->IsChecked())
    {
		consoleOut(wxT("Opening Visualizer\n"));
		//std::cout << "Opening Visualizer\n";
		HeadLaser_CB->SetValue(false);
		FloorLaser_CB->SetValue(false);
		Stereo_CB->SetValue(false);
		Model_CB->SetValue(false);
		UCS_CB->SetValue(false);
		Grid_CB->SetValue(true);
		Objects_CB->SetValue(true);
		LeftDock_FGS->Show(Visualization_SBS,true);
		//LeftDock_FGS->Hide(HeadLaser_RB,true);
		HeadLaser_RB->Show(false);
		Layout();
		Fit();
		//LeftDock_FGS->Layout();
		//Window_FGS->Layout();
		HeadLaser_CB->Enable(true);
		FloorLaser_CB->Enable(true);
		Stereo_CB->Enable(true);
		Model_CB->Enable(true);
		UCS_CB->Enable(true);
		Grid_CB->Enable(true);
		Objects_CB->Enable(true);
		Views_RB->Enable(true);
		if(!vis3d_Window)
			vis3d_Window = new Vis3d(myNode);
    }
    else
    {
		consoleOut(wxT("Closing Visualizer\n"));
		delete vis3d_Window;
		vis3d_Window = 0;
		LeftDock_FGS->Hide(Visualization_SBS,true);
		Layout();
		Fit();
		//LeftDock_FGS->Layout();
		//Window_FGS->Layout();
		HeadLaser_CB->Enable(false);
		FloorLaser_CB->Enable(false);
		Stereo_CB->Enable(false);
		Model_CB->Enable(false);
		UCS_CB->Enable(false);
		Grid_CB->Enable(false);
		Objects_CB->Enable(false);
		Views_RB->Enable(false);
		vis3d_Window->disable();
    }
}

void LauncherImpl::startStop_Topdown( wxCommandEvent& event )
{
	if(Topdown_CB->IsChecked())
	{
		consoleOut(wxT("Opening Topdown\n"));
		RightDock_FGS->Show(Topdown_SBS,true);
		Layout();
		Fit();
		//Window_FGS->Layout();
		PLACEHOLDER_B->Enable(true);
	}
	else
	{
		consoleOut(wxT("Closing Topdown\n"));
		RightDock_FGS->Hide(Topdown_SBS,true);
		Layout();
		Fit();
		//Window_FGS->Layout();
		PLACEHOLDER_B->Enable(false);
	}
}

void LauncherImpl::startStop_PTZL( wxCommandEvent& event )
{
	if(PTZL_CB->IsChecked())
	{
		consoleOut(wxT("Opening Left Pan-Tilt-Zoom\n"));
		LeftDock_FGS->Show(PTZL_SBS,true);
		Layout();
		Fit();
		panPTZL_S->Enable(true);
		tiltPTZL_S->Enable(true);
		zoomPTZL_S->Enable(true);
		PTZL_B->Enable(true);
		myNode->subscribe("PTZL_image", PTZLImage, &LauncherImpl::incomingPTZLImageConn,this);
		myNode->subscribe("PTZL_state", PTZL_state, &LauncherImpl::incomingPTZLState,this);
		myNode->advertise<std_msgs::PTZActuatorCmd>("PTZL_cmd");
		
		//*PTZR_bmp = NULL;
	}
	else
	{
		consoleOut(wxT("Closing Left Pan-Tilt-Zoom\n"));
		LeftDock_FGS->Hide(PTZL_SBS,true);
		myNode->unsubscribe("PTZL_image");
		panPTZL_S->Enable(false);
		tiltPTZL_S->Enable(false);
		zoomPTZL_S->Enable(false);
		PTZL_B->Enable(false);
		wxSize size(0,0);
		PTZL_B->SetMinSize(size);
		Layout();
		Fit();
		PTZL_bmp == NULL;
	}
}

void LauncherImpl::incomingPTZLState()
{
	//std::cout << "receiving position L\n";
	if(PTZL_state.zoom.pos_valid)
		zoomPTZL_S->SetValue(round(PTZL_state.zoom.pos));
	if(PTZL_state.tilt.pos_valid)
		tiltPTZL_S->SetValue(round(PTZL_state.tilt.pos));
	if(PTZL_state.pan.pos_valid)
		panPTZL_S->SetValue(round(PTZL_state.pan.pos));
}

void LauncherImpl::incomingPTZLImageConn()
{
    if(PTZL_GET_NEW_IMAGE)
    {
    	PTZL_GET_NEW_IMAGE = false;
    	const uint32_t count = PTZLImage.get_data_size();
    	delete PTZLImageData;
		PTZLImageData = new uint8_t[count];
		memcpy(PTZLImageData, PTZLImage.data, sizeof(uint8_t) * count);
		wxMemoryInputStream mis(PTZLImageData,PTZLImage.get_data_size());
		delete PTZL_im;
		PTZL_im = new wxImage(mis,wxBITMAP_TYPE_JPEG,-1);
    	
    	//Event stuff
		wxCommandEvent PTZL_Event(wxEVT_COMMAND_BUTTON_CLICKED, PTZL_B->GetId());
		PTZL_Event.SetEventObject(this);
		this->AddPendingEvent(PTZL_Event);
		if(PTZL_bmp == NULL){
			std::cout << "Layout\n";
			wxSize size(PTZLImage.width,PTZLImage.height);
			PTZL_B->SetMinSize(size);
			PTZL_bmp = new wxBitmap();
			Layout();
			Fit();
		}
    }
    //else
    	//std::cout << "!\n";
}

void LauncherImpl::PTZLDrawPic( wxCommandEvent& event )
{		
		delete PTZL_bmp;
		PTZL_bmp = new wxBitmap(*PTZL_im);
		wxClientDC dc( PTZL_B );
		dc.DrawBitmap( *PTZL_bmp, 0, 0, false ); 
		PTZL_GET_NEW_IMAGE = true;
}
//PTZR
void LauncherImpl::startStop_PTZR( wxCommandEvent& event )
{
	if(PTZR_CB->IsChecked())
	{
		consoleOut(wxT("Opening Right Pan-Tilt-Zoom\n"));
		RightDock_FGS->Show(PTZR_SBS,true);
		Layout();
		Fit();
		panPTZR_S->Enable(true);
		tiltPTZR_S->Enable(true);
		zoomPTZR_S->Enable(true);
		PTZR_B->Enable(true);
		myNode->subscribe("PTZR_image", PTZRImage, &LauncherImpl::incomingPTZRImageConn,this);
		myNode->subscribe("PTZR_state", PTZR_state, &LauncherImpl::incomingPTZRState,this);
		myNode->advertise<std_msgs::PTZActuatorCmd>("PTZR_cmd");
		//*PTZR_bmp = NULL;
	}
	else
	{
		consoleOut(wxT("Closing Right Pan-Tilt-Zoom\n"));
		RightDock_FGS->Hide(PTZR_SBS,true);
		myNode->unsubscribe("PTZR_image");
		panPTZR_S->Enable(false);
		tiltPTZR_S->Enable(false);
		zoomPTZR_S->Enable(false);
		PTZR_B->Enable(false);
		wxSize size(0,0);
		PTZR_B->SetMinSize(size);
		Layout();
		Fit();
		PTZR_bmp = NULL;
	}
}

void LauncherImpl::incomingPTZRState()
{
	//std::cout << "receiving position R\n";
	if(PTZR_state.zoom.pos_valid)
		zoomPTZR_S->SetValue(round(PTZR_state.zoom.pos));
	if(PTZR_state.tilt.pos_valid)
		tiltPTZR_S->SetValue(round(PTZR_state.tilt.pos));
	if(PTZR_state.pan.pos_valid)
		panPTZR_S->SetValue(round(PTZR_state.pan.pos));
	//std::cout << "getting pos " << PTZR_state.pan.pos << " " << PTZR_state.tilt.pos << " " << PTZR_state.zoom.pos << endl;
}

void LauncherImpl::incomingPTZRImageConn()
{
    if(PTZR_GET_NEW_IMAGE)
    {
    	PTZR_GET_NEW_IMAGE = false;
    	const uint32_t count = PTZRImage.get_data_size();
    	delete PTZRImageData;
		PTZRImageData = new uint8_t[count];
		memcpy(PTZRImageData, PTZRImage.data, sizeof(uint8_t) * count);
		wxMemoryInputStream mis(PTZRImageData,PTZRImage.get_data_size());
		delete PTZR_im;
		PTZR_im = new wxImage(mis,wxBITMAP_TYPE_JPEG,-1);
    	
    	//Event stuff
		wxCommandEvent PTZR_Event(wxEVT_COMMAND_BUTTON_CLICKED, PTZR_B->GetId());
		PTZR_Event.SetEventObject(this);
		this->AddPendingEvent(PTZR_Event);
		if(PTZR_bmp == NULL){
			std::cout << "Layout\n";
			wxSize size(PTZRImage.width,PTZRImage.height);
			PTZR_B->SetMinSize(size);
			PTZR_bmp = new wxBitmap();
			Layout();
			Fit();
		}
    }
    //else
    	//std::cout << "!\n";
}

void LauncherImpl::PTZRDrawPic( wxCommandEvent& event )
{		
		delete PTZR_bmp;
		PTZR_bmp = new wxBitmap(*PTZR_im);
		wxClientDC dc( PTZR_B );
		dc.DrawBitmap( *PTZR_bmp, 0, 0, false ); 
		PTZR_GET_NEW_IMAGE = true;
}
//Left Wrist
void LauncherImpl::startStop_WristL( wxCommandEvent& event )
{
	if(WristL_CB->IsChecked())
	{
		consoleOut(wxT("Opening Left Wrist\n"));
		LeftDock_FGS->Show(WristL_SBS,true);
		Layout();
		Fit();
		myNode->subscribe("WristL_image", WristLImage, &LauncherImpl::incomingWristLImageConn,this);
		//*WristL_bmp = NULL;
	}
	else
	{
		consoleOut(wxT("Closing Left Wrist\n"));
		LeftDock_FGS->Hide(WristL_SBS,true);
		myNode->unsubscribe("WristL_image");
		wxSize size(0,0);
		WristL_B->SetMinSize(size);
		Layout();
		Fit();
		WristL_bmp = NULL;
	}
}

void LauncherImpl::incomingWristLImageConn()
{
    if(WristL_GET_NEW_IMAGE)
    {
    	WristL_GET_NEW_IMAGE = false;
    	const uint32_t count = WristLImage.get_data_size();
    	delete WristLImageData;
		WristLImageData = new uint8_t[count];
		memcpy(WristLImageData, WristLImage.data, sizeof(uint8_t) * count);
		wxMemoryInputStream mis(WristLImageData,WristLImage.get_data_size());
		delete WristL_im;
		WristL_im = new wxImage(mis,wxBITMAP_TYPE_JPEG,-1);
    	
    	//Event stuff
		wxCommandEvent WristL_Event(wxEVT_COMMAND_BUTTON_CLICKED, WristL_B->GetId());
		WristL_Event.SetEventObject(this);
		this->AddPendingEvent(WristL_Event);
		if(WristL_bmp == NULL){
			std::cout << "Layout\n";
			wxSize size(WristLImage.width,WristLImage.height);
			WristL_B->SetMinSize(size);
			WristL_bmp = new wxBitmap();
			Layout();
			Fit();
		}
    }
    //else
    	//std::cout << "!\n";
}

void LauncherImpl::WristLDrawPic( wxCommandEvent& event )
{		
		delete WristL_bmp;
		WristL_bmp = new wxBitmap(*WristL_im);
		wxClientDC dc( WristL_B );
		dc.DrawBitmap( *WristL_bmp, 0, 0, false ); 
		WristL_GET_NEW_IMAGE = true;
}

//Right Wrist
void LauncherImpl::startStop_WristR( wxCommandEvent& event )
{
	if(WristR_CB->IsChecked())
	{
		consoleOut(wxT("Opening Right Wrist\n"));
		RightDock_FGS->Show(WristR_SBS,true);
		Layout();
		Fit();
		myNode->subscribe("WristR_image", WristRImage, &LauncherImpl::incomingWristRImageConn,this);
		//*WristR_bmp = NULL;
	}
	else
	{
		consoleOut(wxT("Closing Right Wrist\n"));
		RightDock_FGS->Hide(WristR_SBS,true);
		myNode->unsubscribe("WristR_image");
		wxSize size(0,0);
		WristR_B->SetMinSize(size);
		Layout();
		Fit();
		WristR_bmp = NULL;
	}
}

void LauncherImpl::incomingWristRImageConn()
{
    if(WristR_GET_NEW_IMAGE)
    {
    	WristR_GET_NEW_IMAGE = false;
    	const uint32_t count = WristRImage.get_data_size();
    	delete WristRImageData;
		WristRImageData = new uint8_t[count];
		memcpy(WristRImageData, WristRImage.data, sizeof(uint8_t) * count);
		wxMemoryInputStream mis(WristRImageData,WristRImage.get_data_size());
		delete WristR_im;
		WristR_im = new wxImage(mis,wxBITMAP_TYPE_JPEG,-1);
    	
    	//Event stuff
		wxCommandEvent WristR_Event(wxEVT_COMMAND_BUTTON_CLICKED, WristR_B->GetId());
		WristR_Event.SetEventObject(this);
		this->AddPendingEvent(WristR_Event);
		if(WristR_bmp == NULL){
			std::cout << "Layout\n";
			wxSize size(WristRImage.width,WristRImage.height);
			WristR_B->SetMinSize(size);
			WristR_bmp = new wxBitmap();
			Layout();
			Fit();
		}
    }
    //else
    	//std::cout << "!\n";
}

void LauncherImpl::WristRDrawPic( wxCommandEvent& event )
{		
		delete WristR_bmp;
		WristR_bmp = new wxBitmap(*WristR_im);
		wxClientDC dc( WristR_B );
		dc.DrawBitmap( *WristR_bmp, 0, 0, false ); 
		WristR_GET_NEW_IMAGE = true;
}

void LauncherImpl::PTZL_ptzChanged(wxScrollEvent& event)
{
	ptz_cmd.pan.valid = 1;
	ptz_cmd.pan.cmd = panPTZL_S->GetValue();
	ptz_cmd.tilt.valid = 1;
	ptz_cmd.tilt.cmd = tiltPTZL_S->GetValue();
	ptz_cmd.zoom.valid = 1;
	ptz_cmd.zoom.cmd = zoomPTZL_S->GetValue();
	myNode->publish("PTZL_cmd",ptz_cmd);
}

void LauncherImpl::PTZR_ptzChanged(wxScrollEvent& event)
{
	ptz_cmd.pan.valid = 1;
	ptz_cmd.pan.cmd = panPTZR_S->GetValue();
	ptz_cmd.tilt.valid = 1;
	ptz_cmd.tilt.cmd = tiltPTZR_S->GetValue();
	ptz_cmd.zoom.valid = 1;
	ptz_cmd.zoom.cmd = zoomPTZR_S->GetValue();
	//std::cout << "sending pos " << ptz_cmd.pan.cmd << " " << ptz_cmd.tilt.cmd << " " << ptz_cmd.zoom.cmd << endl;
	myNode->publish("PTZR_cmd",ptz_cmd);
}

void LauncherImpl::EmergencyStop( wxCommandEvent& event )
{
	// TODO: Implement EmergencyStop
}
