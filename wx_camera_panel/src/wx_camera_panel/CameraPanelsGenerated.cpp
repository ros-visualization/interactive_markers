///////////////////////////////////////////////////////////////////////////
// C++ code generated with wxFormBuilder (version Apr 21 2008)
// http://www.wxformbuilder.org/
//
// PLEASE DO "NOT" EDIT THIS FILE!
///////////////////////////////////////////////////////////////////////////

#include "CameraPanelsGenerated.h"

///////////////////////////////////////////////////////////////////////////

CameraPanelBase::CameraPanelBase( wxWindow* parent, wxWindowID id, const wxPoint& pos, const wxSize& size, long style ) : wxPanel( parent, id, pos, size, style )
{
	wxBoxSizer* bSizer3;
	bSizer3 = new wxBoxSizer( wxVERTICAL );
	
	wxBoxSizer* bSizer24;
	bSizer24 = new wxBoxSizer( wxHORIZONTAL );
	
	enable_ = new wxCheckBox( this, wxID_ANY, wxT("Enable"), wxDefaultPosition, wxDefaultSize, 0 );
	
	bSizer24->Add( enable_, 0, wxALL|wxALIGN_CENTER_VERTICAL, 5 );
	
	wxBoxSizer* bSizer39;
	bSizer39 = new wxBoxSizer( wxVERTICAL );
	
	setup_ = new wxButton( this, wxID_ANY, wxT("Setup"), wxDefaultPosition, wxDefaultSize, 0 );
	bSizer39->Add( setup_, 0, wxALL|wxALIGN_RIGHT, 5 );
	
	bSizer24->Add( bSizer39, 1, wxEXPAND, 0 );
	
	bSizer3->Add( bSizer24, 0, wxEXPAND, 0 );
	
	image_panel_ = new wxPanel( this, wxID_ANY, wxDefaultPosition, wxDefaultSize, wxTAB_TRAVERSAL );
	bSizer3->Add( image_panel_, 1, wxEXPAND | wxALL, 0 );
	
	this->SetSizer( bSizer3 );
	this->Layout();
	
	// Connect Events
	enable_->Connect( wxEVT_COMMAND_CHECKBOX_CLICKED, wxCommandEventHandler( CameraPanelBase::onEnable ), NULL, this );
	setup_->Connect( wxEVT_COMMAND_BUTTON_CLICKED, wxCommandEventHandler( CameraPanelBase::onSetup ), NULL, this );
	image_panel_->Connect( wxEVT_LEFT_DOWN, wxMouseEventHandler( CameraPanelBase::onLeftMouseDown ), NULL, this );
	image_panel_->Connect( wxEVT_LEFT_UP, wxMouseEventHandler( CameraPanelBase::onLeftMouseUp ), NULL, this );
	image_panel_->Connect( wxEVT_MIDDLE_DOWN, wxMouseEventHandler( CameraPanelBase::onMiddleMouseDown ), NULL, this );
	image_panel_->Connect( wxEVT_MIDDLE_UP, wxMouseEventHandler( CameraPanelBase::onMiddleMouseUp ), NULL, this );
	image_panel_->Connect( wxEVT_MOTION, wxMouseEventHandler( CameraPanelBase::onMouseMotion ), NULL, this );
	image_panel_->Connect( wxEVT_MOUSEWHEEL, wxMouseEventHandler( CameraPanelBase::onMouseWheel ), NULL, this );
	image_panel_->Connect( wxEVT_PAINT, wxPaintEventHandler( CameraPanelBase::onImagePaint ), NULL, this );
	image_panel_->Connect( wxEVT_RIGHT_DOWN, wxMouseEventHandler( CameraPanelBase::onRightMouseDown ), NULL, this );
	image_panel_->Connect( wxEVT_RIGHT_UP, wxMouseEventHandler( CameraPanelBase::onRightMouseUp ), NULL, this );
	image_panel_->Connect( wxEVT_SIZE, wxSizeEventHandler( CameraPanelBase::onImageSize ), NULL, this );
}

CameraPanelBase::~CameraPanelBase()
{
	// Disconnect Events
	enable_->Disconnect( wxEVT_COMMAND_CHECKBOX_CLICKED, wxCommandEventHandler( CameraPanelBase::onEnable ), NULL, this );
	setup_->Disconnect( wxEVT_COMMAND_BUTTON_CLICKED, wxCommandEventHandler( CameraPanelBase::onSetup ), NULL, this );
	image_panel_->Disconnect( wxEVT_LEFT_DOWN, wxMouseEventHandler( CameraPanelBase::onLeftMouseDown ), NULL, this );
	image_panel_->Disconnect( wxEVT_LEFT_UP, wxMouseEventHandler( CameraPanelBase::onLeftMouseUp ), NULL, this );
	image_panel_->Disconnect( wxEVT_MIDDLE_DOWN, wxMouseEventHandler( CameraPanelBase::onMiddleMouseDown ), NULL, this );
	image_panel_->Disconnect( wxEVT_MIDDLE_UP, wxMouseEventHandler( CameraPanelBase::onMiddleMouseUp ), NULL, this );
	image_panel_->Disconnect( wxEVT_MOTION, wxMouseEventHandler( CameraPanelBase::onMouseMotion ), NULL, this );
	image_panel_->Disconnect( wxEVT_MOUSEWHEEL, wxMouseEventHandler( CameraPanelBase::onMouseWheel ), NULL, this );
	image_panel_->Disconnect( wxEVT_PAINT, wxPaintEventHandler( CameraPanelBase::onImagePaint ), NULL, this );
	image_panel_->Disconnect( wxEVT_RIGHT_DOWN, wxMouseEventHandler( CameraPanelBase::onRightMouseDown ), NULL, this );
	image_panel_->Disconnect( wxEVT_RIGHT_UP, wxMouseEventHandler( CameraPanelBase::onRightMouseUp ), NULL, this );
	image_panel_->Disconnect( wxEVT_SIZE, wxSizeEventHandler( CameraPanelBase::onImageSize ), NULL, this );
}

CameraSetupDialogBase::CameraSetupDialogBase( wxWindow* parent, wxWindowID id, const wxString& title, const wxPoint& pos, const wxSize& size, long style ) : wxDialog( parent, id, title, pos, size, style )
{
	this->SetSizeHints( wxDefaultSize, wxDefaultSize );
	
	wxBoxSizer* bSizer21;
	bSizer21 = new wxBoxSizer( wxVERTICAL );
	
	wxStaticBoxSizer* sbSizer4;
	sbSizer4 = new wxStaticBoxSizer( new wxStaticBox( this, wxID_ANY, wxT("Camera") ), wxVERTICAL );
	
	wxBoxSizer* bSizer14;
	bSizer14 = new wxBoxSizer( wxHORIZONTAL );
	
	m_staticText711 = new wxStaticText( this, wxID_ANY, wxT("Name"), wxDefaultPosition, wxDefaultSize, 0 );
	m_staticText711->Wrap( -1 );
	bSizer14->Add( m_staticText711, 1, wxALL|wxALIGN_CENTER_VERTICAL, 5 );
	
	wxBoxSizer* bSizer36;
	bSizer36 = new wxBoxSizer( wxHORIZONTAL );
	
	camera_name_ = new wxTextCtrl( this, wxID_ANY, wxEmptyString, wxDefaultPosition, wxDefaultSize, 0 );
	bSizer36->Add( camera_name_, 1, wxALL|wxALIGN_CENTER_VERTICAL, 5 );
	
	bSizer14->Add( bSizer36, 1, wxEXPAND, 5 );
	
	sbSizer4->Add( bSizer14, 0, wxEXPAND, 5 );
	
	bSizer21->Add( sbSizer4, 0, wxEXPAND, 5 );
	
	wxStaticBoxSizer* sbSizer5;
	sbSizer5 = new wxStaticBoxSizer( new wxStaticBox( this, wxID_ANY, wxT("PTZ") ), wxVERTICAL );
	
	enable_ptz_ = new wxCheckBox( this, wxID_ANY, wxT("Enable"), wxDefaultPosition, wxDefaultSize, 0 );
	enable_ptz_->SetValue(true);
	
	sbSizer5->Add( enable_ptz_, 0, wxALL, 5 );
	
	wxBoxSizer* bSizer12;
	bSizer12 = new wxBoxSizer( wxHORIZONTAL );
	
	m_staticText4 = new wxStaticText( this, wxID_ANY, wxT("Pan Limits:"), wxDefaultPosition, wxDefaultSize, 0 );
	m_staticText4->Wrap( -1 );
	bSizer12->Add( m_staticText4, 1, wxALL|wxALIGN_CENTER_VERTICAL, 5 );
	
	m_staticText5 = new wxStaticText( this, wxID_ANY, wxT("Min:"), wxDefaultPosition, wxDefaultSize, 0 );
	m_staticText5->Wrap( -1 );
	bSizer12->Add( m_staticText5, 0, wxALL|wxALIGN_CENTER_VERTICAL, 5 );
	
	pan_min_ = new wxSpinCtrl( this, wxID_ANY, wxEmptyString, wxDefaultPosition, wxDefaultSize, wxSP_ARROW_KEYS, -9999999, 9999999, -169 );
	bSizer12->Add( pan_min_, 0, wxALL|wxALIGN_CENTER_VERTICAL, 5 );
	
	m_staticText6 = new wxStaticText( this, wxID_ANY, wxT("Max:"), wxDefaultPosition, wxDefaultSize, 0 );
	m_staticText6->Wrap( -1 );
	bSizer12->Add( m_staticText6, 0, wxALL|wxALIGN_CENTER_VERTICAL, 5 );
	
	pan_max_ = new wxSpinCtrl( this, wxID_ANY, wxEmptyString, wxDefaultPosition, wxDefaultSize, wxSP_ARROW_KEYS, -9999999, 9999999, 169 );
	bSizer12->Add( pan_max_, 0, wxALL|wxALIGN_CENTER_VERTICAL, 5 );
	
	sbSizer5->Add( bSizer12, 1, wxEXPAND, 5 );
	
	wxBoxSizer* bSizer1211;
	bSizer1211 = new wxBoxSizer( wxHORIZONTAL );
	
	m_staticText411 = new wxStaticText( this, wxID_ANY, wxT("Tilt Limits:"), wxDefaultPosition, wxDefaultSize, 0 );
	m_staticText411->Wrap( -1 );
	bSizer1211->Add( m_staticText411, 1, wxALL|wxALIGN_CENTER_VERTICAL, 5 );
	
	m_staticText511 = new wxStaticText( this, wxID_ANY, wxT("Min:"), wxDefaultPosition, wxDefaultSize, 0 );
	m_staticText511->Wrap( -1 );
	bSizer1211->Add( m_staticText511, 0, wxALL|wxALIGN_CENTER_VERTICAL, 5 );
	
	tilt_min_ = new wxSpinCtrl( this, wxID_ANY, wxEmptyString, wxDefaultPosition, wxDefaultSize, wxSP_ARROW_KEYS, -9999999, 9999999, -10 );
	bSizer1211->Add( tilt_min_, 0, wxALL|wxALIGN_CENTER_VERTICAL, 5 );
	
	m_staticText611 = new wxStaticText( this, wxID_ANY, wxT("Max:"), wxDefaultPosition, wxDefaultSize, 0 );
	m_staticText611->Wrap( -1 );
	bSizer1211->Add( m_staticText611, 0, wxALL|wxALIGN_CENTER_VERTICAL, 5 );
	
	tilt_max_ = new wxSpinCtrl( this, wxID_ANY, wxEmptyString, wxDefaultPosition, wxDefaultSize, wxSP_ARROW_KEYS, -9999999, 9999999, 90 );
	bSizer1211->Add( tilt_max_, 0, wxALL|wxALIGN_CENTER_VERTICAL, 5 );
	
	sbSizer5->Add( bSizer1211, 1, wxEXPAND, 5 );
	
	wxBoxSizer* bSizer121;
	bSizer121 = new wxBoxSizer( wxHORIZONTAL );
	
	m_staticText41 = new wxStaticText( this, wxID_ANY, wxT("Zoom Limits:"), wxDefaultPosition, wxDefaultSize, 0 );
	m_staticText41->Wrap( -1 );
	bSizer121->Add( m_staticText41, 1, wxALL|wxALIGN_CENTER_VERTICAL, 5 );
	
	m_staticText51 = new wxStaticText( this, wxID_ANY, wxT("Min:"), wxDefaultPosition, wxDefaultSize, 0 );
	m_staticText51->Wrap( -1 );
	bSizer121->Add( m_staticText51, 0, wxALL|wxALIGN_CENTER_VERTICAL, 5 );
	
	zoom_min_ = new wxSpinCtrl( this, wxID_ANY, wxEmptyString, wxDefaultPosition, wxDefaultSize, wxSP_ARROW_KEYS, -9999999, 9999999, 1 );
	bSizer121->Add( zoom_min_, 0, wxALL|wxALIGN_CENTER_VERTICAL, 5 );
	
	m_staticText61 = new wxStaticText( this, wxID_ANY, wxT("Max:"), wxDefaultPosition, wxDefaultSize, 0 );
	m_staticText61->Wrap( -1 );
	bSizer121->Add( m_staticText61, 0, wxALL|wxALIGN_CENTER_VERTICAL, 5 );
	
	zoom_max_ = new wxSpinCtrl( this, wxID_ANY, wxEmptyString, wxDefaultPosition, wxDefaultSize, wxSP_ARROW_KEYS, -9999999, 9999999, 9999 );
	bSizer121->Add( zoom_max_, 0, wxALL|wxALIGN_CENTER_VERTICAL, 5 );
	
	sbSizer5->Add( bSizer121, 1, wxEXPAND, 5 );
	
	bSizer21->Add( sbSizer5, 1, wxEXPAND, 5 );
	
	wxBoxSizer* bSizer23;
	bSizer23 = new wxBoxSizer( wxHORIZONTAL );
	
	ok_ = new wxButton( this, wxID_ANY, wxT("Ok"), wxDefaultPosition, wxDefaultSize, 0 );
	bSizer23->Add( ok_, 0, wxALL, 5 );
	
	cancel_ = new wxButton( this, wxID_ANY, wxT("Cancel"), wxDefaultPosition, wxDefaultSize, 0 );
	bSizer23->Add( cancel_, 0, wxALL, 5 );
	
	bSizer21->Add( bSizer23, 0, wxALIGN_RIGHT, 5 );
	
	this->SetSizer( bSizer21 );
	this->Layout();
	
	// Connect Events
	ok_->Connect( wxEVT_COMMAND_BUTTON_CLICKED, wxCommandEventHandler( CameraSetupDialogBase::onOk ), NULL, this );
	cancel_->Connect( wxEVT_COMMAND_BUTTON_CLICKED, wxCommandEventHandler( CameraSetupDialogBase::onCancel ), NULL, this );
}

CameraSetupDialogBase::~CameraSetupDialogBase()
{
	// Disconnect Events
	ok_->Disconnect( wxEVT_COMMAND_BUTTON_CLICKED, wxCommandEventHandler( CameraSetupDialogBase::onOk ), NULL, this );
	cancel_->Disconnect( wxEVT_COMMAND_BUTTON_CLICKED, wxCommandEventHandler( CameraSetupDialogBase::onCancel ), NULL, this );
}
