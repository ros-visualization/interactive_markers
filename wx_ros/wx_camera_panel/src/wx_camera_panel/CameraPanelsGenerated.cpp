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
	
	m_Enable = new wxCheckBox( this, wxID_ANY, wxT("Enable"), wxDefaultPosition, wxDefaultSize, 0 );
	
	bSizer24->Add( m_Enable, 0, wxALL|wxALIGN_CENTER_VERTICAL, 5 );
	
	wxBoxSizer* bSizer39;
	bSizer39 = new wxBoxSizer( wxVERTICAL );
	
	m_Setup = new wxButton( this, wxID_ANY, wxT("Setup"), wxDefaultPosition, wxDefaultSize, 0 );
	bSizer39->Add( m_Setup, 0, wxALL|wxALIGN_RIGHT, 5 );
	
	bSizer24->Add( bSizer39, 1, wxEXPAND, 5 );
	
	bSizer3->Add( bSizer24, 0, wxEXPAND, 5 );
	
	wxStaticBoxSizer* sbSizer5;
	sbSizer5 = new wxStaticBoxSizer( new wxStaticBox( this, wxID_ANY, wxEmptyString ), wxVERTICAL );
	
	m_ImagePanel = new wxPanel( this, wxID_ANY, wxDefaultPosition, wxDefaultSize, wxTAB_TRAVERSAL );
	sbSizer5->Add( m_ImagePanel, 1, wxEXPAND | wxALL, 5 );
	
	bSizer3->Add( sbSizer5, 1, wxEXPAND, 5 );
	
	this->SetSizer( bSizer3 );
	this->Layout();
	
	// Connect Events
	m_Enable->Connect( wxEVT_COMMAND_CHECKBOX_CLICKED, wxCommandEventHandler( CameraPanelBase::OnEnable ), NULL, this );
	m_Setup->Connect( wxEVT_COMMAND_BUTTON_CLICKED, wxCommandEventHandler( CameraPanelBase::OnSetup ), NULL, this );
	m_ImagePanel->Connect( wxEVT_LEFT_DOWN, wxMouseEventHandler( CameraPanelBase::OnLeftMouseDown ), NULL, this );
	m_ImagePanel->Connect( wxEVT_LEFT_UP, wxMouseEventHandler( CameraPanelBase::OnLeftMouseUp ), NULL, this );
	m_ImagePanel->Connect( wxEVT_MIDDLE_DOWN, wxMouseEventHandler( CameraPanelBase::OnMiddleMouseDown ), NULL, this );
	m_ImagePanel->Connect( wxEVT_MIDDLE_UP, wxMouseEventHandler( CameraPanelBase::OnMiddleMouseUp ), NULL, this );
	m_ImagePanel->Connect( wxEVT_MOTION, wxMouseEventHandler( CameraPanelBase::OnMouseMotion ), NULL, this );
	m_ImagePanel->Connect( wxEVT_MOUSEWHEEL, wxMouseEventHandler( CameraPanelBase::OnMouseWheel ), NULL, this );
	m_ImagePanel->Connect( wxEVT_PAINT, wxPaintEventHandler( CameraPanelBase::OnImagePaint ), NULL, this );
	m_ImagePanel->Connect( wxEVT_RIGHT_DOWN, wxMouseEventHandler( CameraPanelBase::OnRightMouseDown ), NULL, this );
	m_ImagePanel->Connect( wxEVT_RIGHT_UP, wxMouseEventHandler( CameraPanelBase::OnRightMouseUp ), NULL, this );
	m_ImagePanel->Connect( wxEVT_SIZE, wxSizeEventHandler( CameraPanelBase::OnImageSize ), NULL, this );
}

CameraPanelBase::~CameraPanelBase()
{
	// Disconnect Events
	m_Enable->Disconnect( wxEVT_COMMAND_CHECKBOX_CLICKED, wxCommandEventHandler( CameraPanelBase::OnEnable ), NULL, this );
	m_Setup->Disconnect( wxEVT_COMMAND_BUTTON_CLICKED, wxCommandEventHandler( CameraPanelBase::OnSetup ), NULL, this );
	m_ImagePanel->Disconnect( wxEVT_LEFT_DOWN, wxMouseEventHandler( CameraPanelBase::OnLeftMouseDown ), NULL, this );
	m_ImagePanel->Disconnect( wxEVT_LEFT_UP, wxMouseEventHandler( CameraPanelBase::OnLeftMouseUp ), NULL, this );
	m_ImagePanel->Disconnect( wxEVT_MIDDLE_DOWN, wxMouseEventHandler( CameraPanelBase::OnMiddleMouseDown ), NULL, this );
	m_ImagePanel->Disconnect( wxEVT_MIDDLE_UP, wxMouseEventHandler( CameraPanelBase::OnMiddleMouseUp ), NULL, this );
	m_ImagePanel->Disconnect( wxEVT_MOTION, wxMouseEventHandler( CameraPanelBase::OnMouseMotion ), NULL, this );
	m_ImagePanel->Disconnect( wxEVT_MOUSEWHEEL, wxMouseEventHandler( CameraPanelBase::OnMouseWheel ), NULL, this );
	m_ImagePanel->Disconnect( wxEVT_PAINT, wxPaintEventHandler( CameraPanelBase::OnImagePaint ), NULL, this );
	m_ImagePanel->Disconnect( wxEVT_RIGHT_DOWN, wxMouseEventHandler( CameraPanelBase::OnRightMouseDown ), NULL, this );
	m_ImagePanel->Disconnect( wxEVT_RIGHT_UP, wxMouseEventHandler( CameraPanelBase::OnRightMouseUp ), NULL, this );
	m_ImagePanel->Disconnect( wxEVT_SIZE, wxSizeEventHandler( CameraPanelBase::OnImageSize ), NULL, this );
}

CameraSetupDialogBase::CameraSetupDialogBase( wxWindow* parent, wxWindowID id, const wxString& title, const wxPoint& pos, const wxSize& size, long style ) : wxDialog( parent, id, title, pos, size, style )
{
	this->SetSizeHints( wxDefaultSize, wxDefaultSize );
	
	wxBoxSizer* bSizer21;
	bSizer21 = new wxBoxSizer( wxVERTICAL );
	
	wxStaticBoxSizer* sbSizer4;
	sbSizer4 = new wxStaticBoxSizer( new wxStaticBox( this, wxID_ANY, wxT("Image") ), wxVERTICAL );
	
	wxBoxSizer* bSizer14;
	bSizer14 = new wxBoxSizer( wxHORIZONTAL );
	
	m_staticText711 = new wxStaticText( this, wxID_ANY, wxT("Subscription"), wxDefaultPosition, wxDefaultSize, 0 );
	m_staticText711->Wrap( -1 );
	bSizer14->Add( m_staticText711, 1, wxALL|wxALIGN_CENTER_VERTICAL, 5 );
	
	wxBoxSizer* bSizer36;
	bSizer36 = new wxBoxSizer( wxHORIZONTAL );
	
	m_ImageSubscriptionText = new wxTextCtrl( this, wxID_ANY, wxEmptyString, wxDefaultPosition, wxDefaultSize, wxTE_PROCESS_ENTER );
	bSizer36->Add( m_ImageSubscriptionText, 1, wxALL|wxALIGN_CENTER_VERTICAL, 5 );
	
	m_ImageSubscriptionBrowse = new wxButton( this, wxID_ANY, wxT(" ... "), wxDefaultPosition, wxDefaultSize, wxBU_EXACTFIT );
	bSizer36->Add( m_ImageSubscriptionBrowse, 0, wxALL|wxALIGN_CENTER_VERTICAL, 5 );
	
	bSizer14->Add( bSizer36, 1, wxEXPAND, 5 );
	
	sbSizer4->Add( bSizer14, 0, wxEXPAND, 5 );
	
	bSizer21->Add( sbSizer4, 0, wxEXPAND, 5 );
	
	wxStaticBoxSizer* sbSizer5;
	sbSizer5 = new wxStaticBoxSizer( new wxStaticBox( this, wxID_ANY, wxT("PTZ") ), wxVERTICAL );
	
	m_EnablePTZCheck = new wxCheckBox( this, wxID_ANY, wxT("Enable"), wxDefaultPosition, wxDefaultSize, 0 );
	m_EnablePTZCheck->SetValue(true);
	
	sbSizer5->Add( m_EnablePTZCheck, 0, wxALL, 5 );
	
	wxBoxSizer* bSizer12;
	bSizer12 = new wxBoxSizer( wxHORIZONTAL );
	
	m_staticText4 = new wxStaticText( this, wxID_ANY, wxT("Pan Limits:"), wxDefaultPosition, wxDefaultSize, 0 );
	m_staticText4->Wrap( -1 );
	bSizer12->Add( m_staticText4, 1, wxALL|wxALIGN_CENTER_VERTICAL, 5 );
	
	m_staticText5 = new wxStaticText( this, wxID_ANY, wxT("Min:"), wxDefaultPosition, wxDefaultSize, 0 );
	m_staticText5->Wrap( -1 );
	bSizer12->Add( m_staticText5, 0, wxALL|wxALIGN_CENTER_VERTICAL, 5 );
	
	m_PanMinSpin = new wxSpinCtrl( this, wxID_ANY, wxEmptyString, wxDefaultPosition, wxDefaultSize, wxSP_ARROW_KEYS, -9999999, 9999999, -169 );
	bSizer12->Add( m_PanMinSpin, 0, wxALL|wxALIGN_CENTER_VERTICAL, 5 );
	
	m_staticText6 = new wxStaticText( this, wxID_ANY, wxT("Max:"), wxDefaultPosition, wxDefaultSize, 0 );
	m_staticText6->Wrap( -1 );
	bSizer12->Add( m_staticText6, 0, wxALL|wxALIGN_CENTER_VERTICAL, 5 );
	
	m_PanMaxSpin = new wxSpinCtrl( this, wxID_ANY, wxEmptyString, wxDefaultPosition, wxDefaultSize, wxSP_ARROW_KEYS, -9999999, 9999999, 169 );
	bSizer12->Add( m_PanMaxSpin, 0, wxALL|wxALIGN_CENTER_VERTICAL, 5 );
	
	sbSizer5->Add( bSizer12, 1, wxEXPAND, 5 );
	
	wxBoxSizer* bSizer1211;
	bSizer1211 = new wxBoxSizer( wxHORIZONTAL );
	
	m_staticText411 = new wxStaticText( this, wxID_ANY, wxT("Tilt Limits:"), wxDefaultPosition, wxDefaultSize, 0 );
	m_staticText411->Wrap( -1 );
	bSizer1211->Add( m_staticText411, 1, wxALL|wxALIGN_CENTER_VERTICAL, 5 );
	
	m_staticText511 = new wxStaticText( this, wxID_ANY, wxT("Min:"), wxDefaultPosition, wxDefaultSize, 0 );
	m_staticText511->Wrap( -1 );
	bSizer1211->Add( m_staticText511, 0, wxALL|wxALIGN_CENTER_VERTICAL, 5 );
	
	m_TiltMinSpin = new wxSpinCtrl( this, wxID_ANY, wxEmptyString, wxDefaultPosition, wxDefaultSize, wxSP_ARROW_KEYS, -9999999, 9999999, -10 );
	bSizer1211->Add( m_TiltMinSpin, 0, wxALL|wxALIGN_CENTER_VERTICAL, 5 );
	
	m_staticText611 = new wxStaticText( this, wxID_ANY, wxT("Max:"), wxDefaultPosition, wxDefaultSize, 0 );
	m_staticText611->Wrap( -1 );
	bSizer1211->Add( m_staticText611, 0, wxALL|wxALIGN_CENTER_VERTICAL, 5 );
	
	m_TiltMaxSpin = new wxSpinCtrl( this, wxID_ANY, wxEmptyString, wxDefaultPosition, wxDefaultSize, wxSP_ARROW_KEYS, -9999999, 9999999, 90 );
	bSizer1211->Add( m_TiltMaxSpin, 0, wxALL|wxALIGN_CENTER_VERTICAL, 5 );
	
	sbSizer5->Add( bSizer1211, 1, wxEXPAND, 5 );
	
	wxBoxSizer* bSizer121;
	bSizer121 = new wxBoxSizer( wxHORIZONTAL );
	
	m_staticText41 = new wxStaticText( this, wxID_ANY, wxT("Zoom Limits:"), wxDefaultPosition, wxDefaultSize, 0 );
	m_staticText41->Wrap( -1 );
	bSizer121->Add( m_staticText41, 1, wxALL|wxALIGN_CENTER_VERTICAL, 5 );
	
	m_staticText51 = new wxStaticText( this, wxID_ANY, wxT("Min:"), wxDefaultPosition, wxDefaultSize, 0 );
	m_staticText51->Wrap( -1 );
	bSizer121->Add( m_staticText51, 0, wxALL|wxALIGN_CENTER_VERTICAL, 5 );
	
	m_ZoomMinSpin = new wxSpinCtrl( this, wxID_ANY, wxEmptyString, wxDefaultPosition, wxDefaultSize, wxSP_ARROW_KEYS, -9999999, 9999999, 1 );
	bSizer121->Add( m_ZoomMinSpin, 0, wxALL|wxALIGN_CENTER_VERTICAL, 5 );
	
	m_staticText61 = new wxStaticText( this, wxID_ANY, wxT("Max:"), wxDefaultPosition, wxDefaultSize, 0 );
	m_staticText61->Wrap( -1 );
	bSizer121->Add( m_staticText61, 0, wxALL|wxALIGN_CENTER_VERTICAL, 5 );
	
	m_ZoomMaxSpin = new wxSpinCtrl( this, wxID_ANY, wxEmptyString, wxDefaultPosition, wxDefaultSize, wxSP_ARROW_KEYS, -9999999, 9999999, 9999 );
	bSizer121->Add( m_ZoomMaxSpin, 0, wxALL|wxALIGN_CENTER_VERTICAL, 5 );
	
	sbSizer5->Add( bSizer121, 1, wxEXPAND, 5 );
	
	wxBoxSizer* bSizer141;
	bSizer141 = new wxBoxSizer( wxHORIZONTAL );
	
	m_staticText71 = new wxStaticText( this, wxID_ANY, wxT("State Subscription"), wxDefaultPosition, wxDefaultSize, 0 );
	m_staticText71->Wrap( -1 );
	bSizer141->Add( m_staticText71, 1, wxALL|wxALIGN_CENTER_VERTICAL, 5 );
	
	wxBoxSizer* bSizer37;
	bSizer37 = new wxBoxSizer( wxHORIZONTAL );
	
	m_PTZStateSubscriptionText = new wxTextCtrl( this, wxID_ANY, wxEmptyString, wxDefaultPosition, wxDefaultSize, wxTE_PROCESS_ENTER );
	bSizer37->Add( m_PTZStateSubscriptionText, 1, wxALL|wxALIGN_CENTER_VERTICAL, 5 );
	
	m_PTZStateSubscriptionBrowse = new wxButton( this, wxID_ANY, wxT(" ... "), wxDefaultPosition, wxDefaultSize, wxBU_EXACTFIT );
	bSizer37->Add( m_PTZStateSubscriptionBrowse, 0, wxALL|wxALIGN_CENTER_VERTICAL, 5 );
	
	bSizer141->Add( bSizer37, 1, wxEXPAND, 5 );
	
	sbSizer5->Add( bSizer141, 1, wxEXPAND, 5 );
	
	wxBoxSizer* bSizer1411;
	bSizer1411 = new wxBoxSizer( wxHORIZONTAL );
	
	m_staticText712 = new wxStaticText( this, wxID_ANY, wxT("Control Command"), wxDefaultPosition, wxDefaultSize, 0 );
	m_staticText712->Wrap( -1 );
	bSizer1411->Add( m_staticText712, 1, wxALL|wxALIGN_CENTER_VERTICAL, 5 );
	
	wxBoxSizer* bSizer38;
	bSizer38 = new wxBoxSizer( wxVERTICAL );
	
	m_PTZControlCommandText = new wxTextCtrl( this, wxID_ANY, wxEmptyString, wxDefaultPosition, wxDefaultSize, wxTE_PROCESS_ENTER );
	bSizer38->Add( m_PTZControlCommandText, 0, wxALL|wxEXPAND, 5 );
	
	bSizer1411->Add( bSizer38, 1, wxEXPAND|wxALIGN_CENTER_VERTICAL, 5 );
	
	sbSizer5->Add( bSizer1411, 1, wxEXPAND, 5 );
	
	bSizer21->Add( sbSizer5, 1, wxEXPAND, 5 );
	
	wxBoxSizer* bSizer23;
	bSizer23 = new wxBoxSizer( wxHORIZONTAL );
	
	m_Ok = new wxButton( this, wxID_ANY, wxT("Ok"), wxDefaultPosition, wxDefaultSize, 0 );
	bSizer23->Add( m_Ok, 0, wxALL, 5 );
	
	m_Cancel = new wxButton( this, wxID_ANY, wxT("Cancel"), wxDefaultPosition, wxDefaultSize, 0 );
	bSizer23->Add( m_Cancel, 0, wxALL, 5 );
	
	bSizer21->Add( bSizer23, 0, wxALIGN_RIGHT, 5 );
	
	this->SetSizer( bSizer21 );
	this->Layout();
	
	// Connect Events
	m_ImageSubscriptionBrowse->Connect( wxEVT_COMMAND_BUTTON_CLICKED, wxCommandEventHandler( CameraSetupDialogBase::OnImageSubscriptionBrowse ), NULL, this );
	m_EnablePTZCheck->Connect( wxEVT_COMMAND_CHECKBOX_CLICKED, wxCommandEventHandler( CameraSetupDialogBase::OnPTZEnableChecked ), NULL, this );
	m_PTZStateSubscriptionBrowse->Connect( wxEVT_COMMAND_BUTTON_CLICKED, wxCommandEventHandler( CameraSetupDialogBase::OnPTZStateSubscriptionBrowse ), NULL, this );
	m_Ok->Connect( wxEVT_COMMAND_BUTTON_CLICKED, wxCommandEventHandler( CameraSetupDialogBase::OnOk ), NULL, this );
	m_Cancel->Connect( wxEVT_COMMAND_BUTTON_CLICKED, wxCommandEventHandler( CameraSetupDialogBase::OnCancel ), NULL, this );
}

CameraSetupDialogBase::~CameraSetupDialogBase()
{
	// Disconnect Events
	m_ImageSubscriptionBrowse->Disconnect( wxEVT_COMMAND_BUTTON_CLICKED, wxCommandEventHandler( CameraSetupDialogBase::OnImageSubscriptionBrowse ), NULL, this );
	m_EnablePTZCheck->Disconnect( wxEVT_COMMAND_CHECKBOX_CLICKED, wxCommandEventHandler( CameraSetupDialogBase::OnPTZEnableChecked ), NULL, this );
	m_PTZStateSubscriptionBrowse->Disconnect( wxEVT_COMMAND_BUTTON_CLICKED, wxCommandEventHandler( CameraSetupDialogBase::OnPTZStateSubscriptionBrowse ), NULL, this );
	m_Ok->Disconnect( wxEVT_COMMAND_BUTTON_CLICKED, wxCommandEventHandler( CameraSetupDialogBase::OnOk ), NULL, this );
	m_Cancel->Disconnect( wxEVT_COMMAND_BUTTON_CLICKED, wxCommandEventHandler( CameraSetupDialogBase::OnCancel ), NULL, this );
}
