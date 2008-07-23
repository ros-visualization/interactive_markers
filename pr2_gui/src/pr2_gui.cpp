///////////////////////////////////////////////////////////////////////////
// C++ code generated with wxFormBuilder (version Apr 21 2008)
// http://www.wxformbuilder.org/
//
// PLEASE DO "NOT" EDIT THIS FILE!
///////////////////////////////////////////////////////////////////////////

#include "pr2_gui.h"

///////////////////////////////////////////////////////////////////////////

launcher::launcher( wxWindow* parent, wxWindowID id, const wxString& title, const wxPoint& pos, const wxSize& size, long style, const wxString& name ) : wxFrame( parent, id, title, pos, size, style, name )
{
	this->SetSizeHints( wxSize( 452,273 ), wxDefaultSize );
	
	Window_FGS = new wxFlexGridSizer( 1, 3, 0, 0 );
	Window_FGS->AddGrowableCol( 0 );
	Window_FGS->AddGrowableCol( 2 );
	Window_FGS->AddGrowableRow( 0 );
	Window_FGS->SetFlexibleDirection( wxBOTH );
	Window_FGS->SetNonFlexibleGrowMode( wxFLEX_GROWMODE_SPECIFIED );
	
	LeftDock_FGS = new wxFlexGridSizer( 3, 1, 0, 0 );
	LeftDock_FGS->AddGrowableCol( 0 );
	LeftDock_FGS->AddGrowableRow( 1 );
	LeftDock_FGS->AddGrowableRow( 2 );
	LeftDock_FGS->SetFlexibleDirection( wxBOTH );
	LeftDock_FGS->SetNonFlexibleGrowMode( wxFLEX_GROWMODE_SPECIFIED );
	
	Visualization_SBS = new wxStaticBoxSizer( new wxStaticBox( this, wxID_ANY, wxT("Visualization") ), wxHORIZONTAL );
	
	wxFlexGridSizer* Visualization_FGS;
	Visualization_FGS = new wxFlexGridSizer( 1, 3, 0, 0 );
	Visualization_FGS->AddGrowableCol( 0 );
	Visualization_FGS->AddGrowableCol( 1 );
	Visualization_FGS->AddGrowableCol( 2 );
	Visualization_FGS->AddGrowableRow( 0 );
	Visualization_FGS->SetFlexibleDirection( wxBOTH );
	Visualization_FGS->SetNonFlexibleGrowMode( wxFLEX_GROWMODE_SPECIFIED );
	
	wxStaticBoxSizer* Models_SBS;
	Models_SBS = new wxStaticBoxSizer( new wxStaticBox( this, wxID_ANY, wxT("Models") ), wxVERTICAL );
	
	fgSizer12 = new wxFlexGridSizer( 6, 1, 0, 0 );
	fgSizer12->AddGrowableCol( 0 );
	fgSizer12->AddGrowableRow( 0 );
	fgSizer12->AddGrowableRow( 1 );
	fgSizer12->AddGrowableRow( 2 );
	fgSizer12->AddGrowableRow( 3 );
	fgSizer12->AddGrowableRow( 4 );
	fgSizer12->AddGrowableRow( 5 );
	fgSizer12->SetFlexibleDirection( wxBOTH );
	fgSizer12->SetNonFlexibleGrowMode( wxFLEX_GROWMODE_SPECIFIED );
	
	HeadLaser_CB = new wxCheckBox( this, wxID_ANY, wxT("Head Laser"), wxDefaultPosition, wxDefaultSize, 0 );
	
	HeadLaser_CB->Enable( false );
	
	fgSizer12->Add( HeadLaser_CB, 0, 0, 5 );
	
	FloorLaser_CB = new wxCheckBox( this, wxID_ANY, wxT("Floor Laser"), wxDefaultPosition, wxDefaultSize, 0 );
	
	FloorLaser_CB->Enable( false );
	
	fgSizer12->Add( FloorLaser_CB, 0, 0, 5 );
	
	Stereo_CB = new wxCheckBox( this, wxID_ANY, wxT("Stereo"), wxDefaultPosition, wxDefaultSize, 0 );
	
	Stereo_CB->Enable( false );
	
	fgSizer12->Add( Stereo_CB, 0, 0, 5 );
	
	Model_CB = new wxCheckBox( this, wxID_ANY, wxT("Model"), wxDefaultPosition, wxDefaultSize, 0 );
	
	Model_CB->Enable( false );
	
	fgSizer12->Add( Model_CB, 0, 0, 5 );
	
	UCS_CB = new wxCheckBox( this, wxID_ANY, wxT("UCS"), wxDefaultPosition, wxDefaultSize, 0 );
	
	UCS_CB->Enable( false );
	
	fgSizer12->Add( UCS_CB, 0, 0, 5 );
	
	Grid_CB = new wxCheckBox( this, wxID_ANY, wxT("Grid"), wxDefaultPosition, wxDefaultSize, 0 );
	Grid_CB->SetValue(true);
	
	Grid_CB->Enable( false );
	
	fgSizer12->Add( Grid_CB, 0, 0, 5 );
	
	Objects_CB = new wxCheckBox( this, wxID_ANY, wxT("Objects"), wxDefaultPosition, wxDefaultSize, 0 );
	Objects_CB->SetValue(true);
	
	Objects_CB->Enable( false );
	
	fgSizer12->Add( Objects_CB, 0, 0, 5 );
	
	Models_SBS->Add( fgSizer12, 1, wxEXPAND, 5 );
	
	Visualization_FGS->Add( Models_SBS, 1, wxEXPAND|wxALL, 5 );
	
	wxString Views_RBChoices[] = { wxT("Mouse"), wxT("FPS"), wxT("Front Left"), wxT("Front Right"), wxT("Rear Left"), wxT("Rear Right"), wxT("Top"), wxT("Bottom"), wxT("Front"), wxT("Rear"), wxT("Left"), wxT("Right") };
	int Views_RBNChoices = sizeof( Views_RBChoices ) / sizeof( wxString );
	Views_RB = new wxRadioBox( this, wxID_ANY, wxT("Views"), wxDefaultPosition, wxDefaultSize, Views_RBNChoices, Views_RBChoices, 2, wxRA_SPECIFY_COLS );
	Views_RB->SetSelection( 0 );
	Views_RB->Enable( false );
	
	Visualization_FGS->Add( Views_RB, 1, wxEXPAND|wxALL, 5 );
	
	wxString HeadLaser_RBChoices[] = { wxT("Wipe"), wxT("Replace"), wxT("At Once") };
	int HeadLaser_RBNChoices = sizeof( HeadLaser_RBChoices ) / sizeof( wxString );
	HeadLaser_RB = new wxRadioBox( this, wxID_ANY, wxT("Head Laser"), wxDefaultPosition, wxDefaultSize, HeadLaser_RBNChoices, HeadLaser_RBChoices, 1, wxRA_SPECIFY_COLS );
	HeadLaser_RB->SetSelection( 0 );
	Visualization_FGS->Add( HeadLaser_RB, 1, wxALL|wxEXPAND, 5 );
	
	Visualization_SBS->Add( Visualization_FGS, 1, wxEXPAND, 5 );
	
	LeftDock_FGS->Add( Visualization_SBS, 1, wxALL|wxEXPAND, 5 );
	
	PTZL_SBS = new wxStaticBoxSizer( new wxStaticBox( this, wxID_ANY, wxT("Left PTZ") ), wxVERTICAL );
	
	wxFlexGridSizer* fgSizer10;
	fgSizer10 = new wxFlexGridSizer( 2, 1, 0, 0 );
	fgSizer10->AddGrowableCol( 0 );
	fgSizer10->AddGrowableRow( 1 );
	fgSizer10->SetFlexibleDirection( wxBOTH );
	fgSizer10->SetNonFlexibleGrowMode( wxFLEX_GROWMODE_SPECIFIED );
	
	panPTZL_S = new wxSlider( this, wxID_ANY, 0, -169, 169, wxDefaultPosition, wxDefaultSize, wxSL_HORIZONTAL );
	panPTZL_S->Enable( false );
	panPTZL_S->SetToolTip( wxT("Pan") );
	
	fgSizer10->Add( panPTZL_S, 1, wxALL|wxEXPAND|wxALIGN_CENTER_HORIZONTAL, 5 );
	
	wxFlexGridSizer* fgSizer9;
	fgSizer9 = new wxFlexGridSizer( 1, 3, 0, 0 );
	fgSizer9->AddGrowableCol( 1 );
	fgSizer9->AddGrowableRow( 0 );
	fgSizer9->SetFlexibleDirection( wxBOTH );
	fgSizer9->SetNonFlexibleGrowMode( wxFLEX_GROWMODE_SPECIFIED );
	
	zoomPTZL_S = new wxSlider( this, wxID_ANY, 0, 0, 10000, wxDefaultPosition, wxDefaultSize, wxSL_INVERSE|wxSL_VERTICAL );
	zoomPTZL_S->Enable( false );
	zoomPTZL_S->SetToolTip( wxT("Zoom") );
	
	fgSizer9->Add( zoomPTZL_S, 1, wxALL|wxEXPAND, 5 );
	
	PTZL_B = new wxPanel( this, wxID_ANY, wxDefaultPosition, wxDefaultSize, wxTAB_TRAVERSAL );
	fgSizer9->Add( PTZL_B, 0, wxALL|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 5 );
	
	tiltPTZL_S = new wxSlider( this, wxID_ANY, 40, -10, 90, wxDefaultPosition, wxDefaultSize, wxSL_INVERSE|wxSL_VERTICAL );
	tiltPTZL_S->Enable( false );
	tiltPTZL_S->SetToolTip( wxT("Tilt") );
	
	fgSizer9->Add( tiltPTZL_S, 1, wxALL|wxEXPAND|wxALIGN_RIGHT, 5 );
	
	fgSizer10->Add( fgSizer9, 1, wxEXPAND|wxALIGN_BOTTOM, 5 );
	
	PTZL_SBS->Add( fgSizer10, 1, wxEXPAND, 5 );
	
	LeftDock_FGS->Add( PTZL_SBS, 1, wxEXPAND|wxALL, 5 );
	
	WristL_SBS = new wxStaticBoxSizer( new wxStaticBox( this, wxID_ANY, wxT("Left Wrist") ), wxVERTICAL );
	
	wxFlexGridSizer* fgSizer27;
	fgSizer27 = new wxFlexGridSizer( 1, 1, 0, 0 );
	fgSizer27->AddGrowableCol( 0 );
	fgSizer27->AddGrowableRow( 0 );
	fgSizer27->SetFlexibleDirection( wxBOTH );
	fgSizer27->SetNonFlexibleGrowMode( wxFLEX_GROWMODE_SPECIFIED );
	
	WristL_B = new wxPanel( this, wxID_ANY, wxDefaultPosition, wxDefaultSize, wxTAB_TRAVERSAL );
	fgSizer27->Add( WristL_B, 0, wxALL|wxALIGN_CENTER_VERTICAL|wxALIGN_CENTER_HORIZONTAL, 5 );
	
	WristL_SBS->Add( fgSizer27, 1, wxEXPAND, 5 );
	
	LeftDock_FGS->Add( WristL_SBS, 1, wxEXPAND|wxALL, 5 );
	
	Window_FGS->Add( LeftDock_FGS, 1, wxEXPAND, 5 );
	
	wxFlexGridSizer* Launcher_FGS;
	Launcher_FGS = new wxFlexGridSizer( 2, 1, 0, 0 );
	Launcher_FGS->AddGrowableCol( 0 );
	Launcher_FGS->AddGrowableRow( 1 );
	Launcher_FGS->SetFlexibleDirection( wxBOTH );
	Launcher_FGS->SetNonFlexibleGrowMode( wxFLEX_GROWMODE_SPECIFIED );
	
	wxBoxSizer* bSizer2;
	bSizer2 = new wxBoxSizer( wxHORIZONTAL );
	
	wxStaticBoxSizer* sbSizer1;
	sbSizer1 = new wxStaticBoxSizer( new wxStaticBox( this, wxID_ANY, wxT("Controls") ), wxVERTICAL );
	
	wxFlexGridSizer* fgSizer3;
	fgSizer3 = new wxFlexGridSizer( 3, 2, 0, 0 );
	fgSizer3->AddGrowableCol( 0 );
	fgSizer3->AddGrowableCol( 1 );
	fgSizer3->AddGrowableRow( 0 );
	fgSizer3->AddGrowableRow( 1 );
	fgSizer3->AddGrowableRow( 2 );
	fgSizer3->SetFlexibleDirection( wxVERTICAL );
	fgSizer3->SetNonFlexibleGrowMode( wxFLEX_GROWMODE_SPECIFIED );
	
	Visualization_CB = new wxCheckBox( this, wxID_ANY, wxT("Visualization"), wxDefaultPosition, wxDefaultSize, 0 );
	
	fgSizer3->Add( Visualization_CB, 0, wxALL, 5 );
	
	Topdown_CB = new wxCheckBox( this, wxID_ANY, wxT("Topdown"), wxDefaultPosition, wxDefaultSize, 0 );
	
	fgSizer3->Add( Topdown_CB, 0, wxALL, 5 );
	
	PTZL_CB = new wxCheckBox( this, wxID_ANY, wxT("Left PTZ"), wxDefaultPosition, wxDefaultSize, 0 );
	
	fgSizer3->Add( PTZL_CB, 0, wxALL, 5 );
	
	PTZR_CB = new wxCheckBox( this, wxID_ANY, wxT("Right PTZ"), wxDefaultPosition, wxDefaultSize, 0 );
	
	fgSizer3->Add( PTZR_CB, 0, wxALL, 5 );
	
	WristL_CB = new wxCheckBox( this, wxID_ANY, wxT("Left Wrist"), wxDefaultPosition, wxDefaultSize, 0 );
	
	fgSizer3->Add( WristL_CB, 0, wxALL, 5 );
	
	WristR_CB = new wxCheckBox( this, wxID_ANY, wxT("Right Wrist"), wxDefaultPosition, wxDefaultSize, 0 );
	
	fgSizer3->Add( WristR_CB, 0, wxALL, 5 );
	
	sbSizer1->Add( fgSizer3, 1, wxEXPAND|wxALIGN_CENTER_HORIZONTAL, 5 );
	
	bSizer2->Add( sbSizer1, 1, wxEXPAND|wxALL, 5 );
	
	EmStop_B = new wxButton( this, wxID_ANY, wxT("Emergency Stop"), wxDefaultPosition, wxDefaultSize, 0 );
	EmStop_B->SetFont( wxFont( wxNORMAL_FONT->GetPointSize(), 70, 90, 92, false, wxEmptyString ) );
	EmStop_B->SetForegroundColour( wxColour( 255, 255, 255 ) );
	EmStop_B->SetBackgroundColour( wxColour( 255, 0, 0 ) );
	
	bSizer2->Add( EmStop_B, 1, wxALL|wxEXPAND|wxALIGN_RIGHT, 5 );
	
	Launcher_FGS->Add( bSizer2, 1, wxEXPAND, 5 );
	
	wxBoxSizer* bSizer1;
	bSizer1 = new wxBoxSizer( wxHORIZONTAL );
	
	Console_TC = new wxTextCtrl( this, wxID_ANY, wxEmptyString, wxDefaultPosition, wxDefaultSize, wxTE_MULTILINE|wxTE_READONLY|wxTE_WORDWRAP );
	bSizer1->Add( Console_TC, 1, wxALL|wxEXPAND|wxALIGN_BOTTOM, 5 );
	
	Ros_TC = new wxTextCtrl( this, wxID_ANY, wxEmptyString, wxDefaultPosition, wxDefaultSize, wxTE_MULTILINE|wxTE_READONLY|wxTE_WORDWRAP );
	bSizer1->Add( Ros_TC, 1, wxALL|wxEXPAND|wxALIGN_RIGHT|wxALIGN_BOTTOM, 5 );
	
	Launcher_FGS->Add( bSizer1, 1, wxEXPAND, 5 );
	
	Window_FGS->Add( Launcher_FGS, 1, wxEXPAND|wxALIGN_CENTER_HORIZONTAL, 5 );
	
	RightDock_FGS = new wxFlexGridSizer( 3, 1, 0, 0 );
	RightDock_FGS->AddGrowableCol( 0 );
	RightDock_FGS->AddGrowableRow( 1 );
	RightDock_FGS->AddGrowableRow( 2 );
	RightDock_FGS->SetFlexibleDirection( wxBOTH );
	RightDock_FGS->SetNonFlexibleGrowMode( wxFLEX_GROWMODE_SPECIFIED );
	
	Topdown_SBS = new wxStaticBoxSizer( new wxStaticBox( this, wxID_ANY, wxT("Topdown") ), wxHORIZONTAL );
	
	PLACEHOLDER_B = new wxButton( this, wxID_ANY, wxT("MyButton"), wxDefaultPosition, wxDefaultSize, 0 );
	PLACEHOLDER_B->Enable( false );
	
	Topdown_SBS->Add( PLACEHOLDER_B, 0, wxALL, 5 );
	
	RightDock_FGS->Add( Topdown_SBS, 1, wxEXPAND|wxALL, 5 );
	
	PTZR_SBS = new wxStaticBoxSizer( new wxStaticBox( this, wxID_ANY, wxT("Right PTZ") ), wxVERTICAL );
	
	wxFlexGridSizer* fgSizer101;
	fgSizer101 = new wxFlexGridSizer( 2, 1, 0, 0 );
	fgSizer101->AddGrowableCol( 0 );
	fgSizer101->AddGrowableRow( 1 );
	fgSizer101->SetFlexibleDirection( wxBOTH );
	fgSizer101->SetNonFlexibleGrowMode( wxFLEX_GROWMODE_SPECIFIED );
	
	panPTZR_S = new wxSlider( this, wxID_ANY, 0, -169, 169, wxDefaultPosition, wxDefaultSize, wxSL_HORIZONTAL );
	panPTZR_S->Enable( false );
	panPTZR_S->SetToolTip( wxT("Pan") );
	
	fgSizer101->Add( panPTZR_S, 1, wxALL|wxEXPAND|wxALIGN_CENTER_HORIZONTAL, 5 );
	
	wxFlexGridSizer* fgSizer91;
	fgSizer91 = new wxFlexGridSizer( 1, 3, 0, 0 );
	fgSizer91->AddGrowableCol( 1 );
	fgSizer91->AddGrowableRow( 0 );
	fgSizer91->SetFlexibleDirection( wxBOTH );
	fgSizer91->SetNonFlexibleGrowMode( wxFLEX_GROWMODE_SPECIFIED );
	
	zoomPTZR_S = new wxSlider( this, wxID_ANY, 0, 0, 10000, wxDefaultPosition, wxDefaultSize, wxSL_INVERSE|wxSL_VERTICAL );
	zoomPTZR_S->Enable( false );
	zoomPTZR_S->SetToolTip( wxT("Zoom") );
	
	fgSizer91->Add( zoomPTZR_S, 1, wxALL|wxEXPAND, 5 );
	
	PTZR_B = new wxPanel( this, wxID_ANY, wxDefaultPosition, wxDefaultSize, wxTAB_TRAVERSAL );
	fgSizer91->Add( PTZR_B, 0, wxALL|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 5 );
	
	tiltPTZR_S = new wxSlider( this, wxID_ANY, 40, -10, 90, wxDefaultPosition, wxDefaultSize, wxSL_INVERSE|wxSL_VERTICAL );
	tiltPTZR_S->Enable( false );
	tiltPTZR_S->SetToolTip( wxT("Tilt") );
	
	fgSizer91->Add( tiltPTZR_S, 1, wxALL|wxEXPAND|wxALIGN_RIGHT, 5 );
	
	fgSizer101->Add( fgSizer91, 1, wxEXPAND|wxALIGN_BOTTOM, 5 );
	
	PTZR_SBS->Add( fgSizer101, 1, wxEXPAND, 5 );
	
	RightDock_FGS->Add( PTZR_SBS, 1, wxEXPAND|wxALL, 5 );
	
	WristR_SBS = new wxStaticBoxSizer( new wxStaticBox( this, wxID_ANY, wxT("Right Wrist") ), wxVERTICAL );
	
	wxFlexGridSizer* fgSizer271;
	fgSizer271 = new wxFlexGridSizer( 1, 1, 0, 0 );
	fgSizer271->AddGrowableCol( 0 );
	fgSizer271->AddGrowableRow( 0 );
	fgSizer271->SetFlexibleDirection( wxBOTH );
	fgSizer271->SetNonFlexibleGrowMode( wxFLEX_GROWMODE_SPECIFIED );
	
	WristR_B = new wxPanel( this, wxID_ANY, wxDefaultPosition, wxDefaultSize, wxTAB_TRAVERSAL );
	fgSizer271->Add( WristR_B, 0, wxALL|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 5 );
	
	WristR_SBS->Add( fgSizer271, 1, wxEXPAND, 5 );
	
	RightDock_FGS->Add( WristR_SBS, 1, wxEXPAND|wxALL, 5 );
	
	Window_FGS->Add( RightDock_FGS, 1, wxEXPAND, 5 );
	
	this->SetSizer( Window_FGS );
	this->Layout();
	
	// Connect Events
	HeadLaser_CB->Connect( wxEVT_COMMAND_CHECKBOX_CLICKED, wxCommandEventHandler( launcher::startStopHeadPtCld ), NULL, this );
	FloorLaser_CB->Connect( wxEVT_COMMAND_CHECKBOX_CLICKED, wxCommandEventHandler( launcher::startStopFloorPtCld ), NULL, this );
	Stereo_CB->Connect( wxEVT_COMMAND_CHECKBOX_CLICKED, wxCommandEventHandler( launcher::startStopStereoPtCld ), NULL, this );
	Model_CB->Connect( wxEVT_COMMAND_CHECKBOX_CLICKED, wxCommandEventHandler( launcher::startStopModel ), NULL, this );
	UCS_CB->Connect( wxEVT_COMMAND_CHECKBOX_CLICKED, wxCommandEventHandler( launcher::startStopUCS ), NULL, this );
	Grid_CB->Connect( wxEVT_COMMAND_CHECKBOX_CLICKED, wxCommandEventHandler( launcher::startStopGrid ), NULL, this );
	Objects_CB->Connect( wxEVT_COMMAND_CHECKBOX_CLICKED, wxCommandEventHandler( launcher::startStopObjects ), NULL, this );
	Views_RB->Connect( wxEVT_COMMAND_RADIOBOX_SELECTED, wxCommandEventHandler( launcher::viewChanged ), NULL, this );
	HeadLaser_RB->Connect( wxEVT_COMMAND_RADIOBOX_SELECTED, wxCommandEventHandler( launcher::HeadLaserChanged ), NULL, this );
	panPTZL_S->Connect( wxEVT_SCROLL_CHANGED, wxScrollEventHandler( launcher::PTZL_ptzChanged ), NULL, this );
	zoomPTZL_S->Connect( wxEVT_SCROLL_CHANGED, wxScrollEventHandler( launcher::PTZL_ptzChanged ), NULL, this );
	PTZL_B->Connect( wxEVT_RIGHT_UP, wxMouseEventHandler( launcher::PTZL_click ), NULL, this );
	tiltPTZL_S->Connect( wxEVT_SCROLL_CHANGED, wxScrollEventHandler( launcher::PTZL_ptzChanged ), NULL, this );
	Visualization_CB->Connect( wxEVT_COMMAND_CHECKBOX_CLICKED, wxCommandEventHandler( launcher::startStop_Visualization ), NULL, this );
	Topdown_CB->Connect( wxEVT_COMMAND_CHECKBOX_CLICKED, wxCommandEventHandler( launcher::startStopTopdown ), NULL, this );
	PTZL_CB->Connect( wxEVT_COMMAND_CHECKBOX_CLICKED, wxCommandEventHandler( launcher::startStop_PTZL ), NULL, this );
	PTZR_CB->Connect( wxEVT_COMMAND_CHECKBOX_CLICKED, wxCommandEventHandler( launcher::startStop_PTZR ), NULL, this );
	WristL_CB->Connect( wxEVT_COMMAND_CHECKBOX_CLICKED, wxCommandEventHandler( launcher::startStop_WristL ), NULL, this );
	WristR_CB->Connect( wxEVT_COMMAND_CHECKBOX_CLICKED, wxCommandEventHandler( launcher::startStop_WristR ), NULL, this );
	EmStop_B->Connect( wxEVT_COMMAND_BUTTON_CLICKED, wxCommandEventHandler( launcher::EmergencyStop ), NULL, this );
	panPTZR_S->Connect( wxEVT_SCROLL_CHANGED, wxScrollEventHandler( launcher::PTZR_ptzChanged ), NULL, this );
	zoomPTZR_S->Connect( wxEVT_SCROLL_CHANGED, wxScrollEventHandler( launcher::PTZR_ptzChanged ), NULL, this );
	PTZR_B->Connect( wxEVT_RIGHT_UP, wxMouseEventHandler( launcher::PTZR_click ), NULL, this );
	tiltPTZR_S->Connect( wxEVT_SCROLL_CHANGED, wxScrollEventHandler( launcher::PTZR_ptzChanged ), NULL, this );
}

launcher::~launcher()
{
	// Disconnect Events
	HeadLaser_CB->Disconnect( wxEVT_COMMAND_CHECKBOX_CLICKED, wxCommandEventHandler( launcher::startStopHeadPtCld ), NULL, this );
	FloorLaser_CB->Disconnect( wxEVT_COMMAND_CHECKBOX_CLICKED, wxCommandEventHandler( launcher::startStopFloorPtCld ), NULL, this );
	Stereo_CB->Disconnect( wxEVT_COMMAND_CHECKBOX_CLICKED, wxCommandEventHandler( launcher::startStopStereoPtCld ), NULL, this );
	Model_CB->Disconnect( wxEVT_COMMAND_CHECKBOX_CLICKED, wxCommandEventHandler( launcher::startStopModel ), NULL, this );
	UCS_CB->Disconnect( wxEVT_COMMAND_CHECKBOX_CLICKED, wxCommandEventHandler( launcher::startStopUCS ), NULL, this );
	Grid_CB->Disconnect( wxEVT_COMMAND_CHECKBOX_CLICKED, wxCommandEventHandler( launcher::startStopGrid ), NULL, this );
	Objects_CB->Disconnect( wxEVT_COMMAND_CHECKBOX_CLICKED, wxCommandEventHandler( launcher::startStopObjects ), NULL, this );
	Views_RB->Disconnect( wxEVT_COMMAND_RADIOBOX_SELECTED, wxCommandEventHandler( launcher::viewChanged ), NULL, this );
	HeadLaser_RB->Disconnect( wxEVT_COMMAND_RADIOBOX_SELECTED, wxCommandEventHandler( launcher::HeadLaserChanged ), NULL, this );
	panPTZL_S->Disconnect( wxEVT_SCROLL_CHANGED, wxScrollEventHandler( launcher::PTZL_ptzChanged ), NULL, this );
	zoomPTZL_S->Disconnect( wxEVT_SCROLL_CHANGED, wxScrollEventHandler( launcher::PTZL_ptzChanged ), NULL, this );
	PTZL_B->Disconnect( wxEVT_RIGHT_UP, wxMouseEventHandler( launcher::PTZL_click ), NULL, this );
	tiltPTZL_S->Disconnect( wxEVT_SCROLL_CHANGED, wxScrollEventHandler( launcher::PTZL_ptzChanged ), NULL, this );
	Visualization_CB->Disconnect( wxEVT_COMMAND_CHECKBOX_CLICKED, wxCommandEventHandler( launcher::startStop_Visualization ), NULL, this );
	Topdown_CB->Disconnect( wxEVT_COMMAND_CHECKBOX_CLICKED, wxCommandEventHandler( launcher::startStopTopdown ), NULL, this );
	PTZL_CB->Disconnect( wxEVT_COMMAND_CHECKBOX_CLICKED, wxCommandEventHandler( launcher::startStop_PTZL ), NULL, this );
	PTZR_CB->Disconnect( wxEVT_COMMAND_CHECKBOX_CLICKED, wxCommandEventHandler( launcher::startStop_PTZR ), NULL, this );
	WristL_CB->Disconnect( wxEVT_COMMAND_CHECKBOX_CLICKED, wxCommandEventHandler( launcher::startStop_WristL ), NULL, this );
	WristR_CB->Disconnect( wxEVT_COMMAND_CHECKBOX_CLICKED, wxCommandEventHandler( launcher::startStop_WristR ), NULL, this );
	EmStop_B->Disconnect( wxEVT_COMMAND_BUTTON_CLICKED, wxCommandEventHandler( launcher::EmergencyStop ), NULL, this );
	panPTZR_S->Disconnect( wxEVT_SCROLL_CHANGED, wxScrollEventHandler( launcher::PTZR_ptzChanged ), NULL, this );
	zoomPTZR_S->Disconnect( wxEVT_SCROLL_CHANGED, wxScrollEventHandler( launcher::PTZR_ptzChanged ), NULL, this );
	PTZR_B->Disconnect( wxEVT_RIGHT_UP, wxMouseEventHandler( launcher::PTZR_click ), NULL, this );
	tiltPTZR_S->Disconnect( wxEVT_SCROLL_CHANGED, wxScrollEventHandler( launcher::PTZR_ptzChanged ), NULL, this );
}
