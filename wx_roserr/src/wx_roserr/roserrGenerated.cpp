///////////////////////////////////////////////////////////////////////////
// C++ code generated with wxFormBuilder (version Apr 21 2008)
// http://www.wxformbuilder.org/
//
// PLEASE DO "NOT" EDIT THIS FILE!
///////////////////////////////////////////////////////////////////////////

#include "roserrGenerated.h"

///////////////////////////////////////////////////////////////////////////

RoserrPanelBase::RoserrPanelBase( wxWindow* parent, wxWindowID id, const wxPoint& pos, const wxSize& size, long style ) : wxPanel( parent, id, pos, size, style )
{
	this->SetMinSize( wxSize( 273,138 ) );
	
	wxBoxSizer* bSizer2;
	bSizer2 = new wxBoxSizer( wxVERTICAL );
	
	wxBoxSizer* bSizer3;
	bSizer3 = new wxBoxSizer( wxHORIZONTAL );
	
	m_EnableCB = new wxCheckBox( this, wxID_ANY, wxT("Enable"), wxDefaultPosition, wxDefaultSize, 0 );
	
	bSizer3->Add( m_EnableCB, 0, wxALL|wxALIGN_CENTER_VERTICAL, 5 );
	
	wxBoxSizer* bSizer4;
	bSizer4 = new wxBoxSizer( wxVERTICAL );
	
	wxBoxSizer* bSizer5;
	bSizer5 = new wxBoxSizer( wxHORIZONTAL );
	
	wxBoxSizer* bSizer6;
	bSizer6 = new wxBoxSizer( wxVERTICAL );
	
	m_FontFP = new wxFontPickerCtrl( this, wxID_ANY, wxFont( 10, 70, 90, 90, false, wxT("sans") ), wxDefaultPosition, wxDefaultSize, wxFNTP_DEFAULT_STYLE );
	m_FontFP->SetMaxPointSize( 100 ); 
	bSizer6->Add( m_FontFP, 0, wxALL|wxALIGN_RIGHT, 5 );
	
	bSizer5->Add( bSizer6, 1, 0, 5 );
	
	wxBoxSizer* bSizer7;
	bSizer7 = new wxBoxSizer( wxVERTICAL );
	
	m_Setup = new wxButton( this, wxID_ANY, wxT("Setup"), wxDefaultPosition, wxDefaultSize, 0 );
	bSizer7->Add( m_Setup, 0, wxALL|wxALIGN_RIGHT, 5 );
	
	bSizer5->Add( bSizer7, 0, 0, 5 );
	
	bSizer4->Add( bSizer5, 1, wxEXPAND|wxALIGN_RIGHT, 5 );
	
	bSizer3->Add( bSizer4, 1, wxEXPAND, 5 );
	
	bSizer2->Add( bSizer3, 0, wxEXPAND, 5 );
	
	m_roserrTC = new wxTextCtrl( this, wxID_ANY, wxEmptyString, wxDefaultPosition, wxDefaultSize, wxTE_LEFT|wxTE_MULTILINE|wxTE_READONLY );
	bSizer2->Add( m_roserrTC, 1, wxALL|wxEXPAND, 5 );
	
	this->SetSizer( bSizer2 );
	this->Layout();
	
	// Connect Events
	m_EnableCB->Connect( wxEVT_COMMAND_CHECKBOX_CLICKED, wxCommandEventHandler( RoserrPanelBase::OnEnable ), NULL, this );
	m_FontFP->Connect( wxEVT_COMMAND_FONTPICKER_CHANGED, wxFontPickerEventHandler( RoserrPanelBase::OnFontChange ), NULL, this );
	m_Setup->Connect( wxEVT_COMMAND_BUTTON_CLICKED, wxCommandEventHandler( RoserrPanelBase::OnSetup ), NULL, this );
}

RoserrPanelBase::~RoserrPanelBase()
{
	// Disconnect Events
	m_EnableCB->Disconnect( wxEVT_COMMAND_CHECKBOX_CLICKED, wxCommandEventHandler( RoserrPanelBase::OnEnable ), NULL, this );
	m_FontFP->Disconnect( wxEVT_COMMAND_FONTPICKER_CHANGED, wxFontPickerEventHandler( RoserrPanelBase::OnFontChange ), NULL, this );
	m_Setup->Disconnect( wxEVT_COMMAND_BUTTON_CLICKED, wxCommandEventHandler( RoserrPanelBase::OnSetup ), NULL, this );
}

RoserrSetupDialogBase::RoserrSetupDialogBase( wxWindow* parent, wxWindowID id, const wxString& title, const wxPoint& pos, const wxSize& size, long style ) : wxDialog( parent, id, title, pos, size, style )
{
	this->SetSizeHints( wxSize( 219,150 ), wxDefaultSize );
	
	wxBoxSizer* bSizer8;
	bSizer8 = new wxBoxSizer( wxVERTICAL );
	
	wxStaticBoxSizer* TCOptionsSBS;
	TCOptionsSBS = new wxStaticBoxSizer( new wxStaticBox( this, wxID_ANY, wxT("Textbox Options") ), wxVERTICAL );
	
	m_MonochromeCB = new wxCheckBox( this, wxID_ANY, wxT("Monochrome"), wxDefaultPosition, wxDefaultSize, 0 );
	
	TCOptionsSBS->Add( m_MonochromeCB, 0, wxALL, 5 );
	
	bSizer8->Add( TCOptionsSBS, 1, wxEXPAND, 5 );
	
	wxStaticBoxSizer* sbSizer2;
	sbSizer2 = new wxStaticBoxSizer( new wxStaticBox( this, wxID_ANY, wxT("Ros Topic") ), wxVERTICAL );
	
	wxBoxSizer* bSizer10;
	bSizer10 = new wxBoxSizer( wxHORIZONTAL );
	
	m_staticText1 = new wxStaticText( this, wxID_ANY, wxT("Subscription"), wxDefaultPosition, wxDefaultSize, 0 );
	m_staticText1->Wrap( -1 );
	bSizer10->Add( m_staticText1, 1, wxALL|wxALIGN_CENTER_VERTICAL, 5 );
	
	wxBoxSizer* bSizer91;
	bSizer91 = new wxBoxSizer( wxHORIZONTAL );
	
	m_rostopicL = new wxTextCtrl( this, wxID_ANY, wxEmptyString, wxDefaultPosition, wxDefaultSize, wxTE_PROCESS_ENTER );
	bSizer91->Add( m_rostopicL, 1, wxALL|wxALIGN_CENTER_VERTICAL|wxEXPAND, 5 );
	
	m_roserrB = new wxButton( this, wxID_ANY, wxT(" ... "), wxDefaultPosition, wxDefaultSize, wxBU_EXACTFIT );
	bSizer91->Add( m_roserrB, 0, wxALL|wxALIGN_CENTER_VERTICAL, 5 );
	
	bSizer10->Add( bSizer91, 1, wxEXPAND, 5 );
	
	sbSizer2->Add( bSizer10, 0, wxEXPAND, 5 );
	
	bSizer8->Add( sbSizer2, 0, wxEXPAND, 5 );
	
	wxBoxSizer* bSizer9;
	bSizer9 = new wxBoxSizer( wxHORIZONTAL );
	
	m_sdbSizer1 = new wxStdDialogButtonSizer();
	m_sdbSizer1OK = new wxButton( this, wxID_OK );
	m_sdbSizer1->AddButton( m_sdbSizer1OK );
	m_sdbSizer1Cancel = new wxButton( this, wxID_CANCEL );
	m_sdbSizer1->AddButton( m_sdbSizer1Cancel );
	m_sdbSizer1->Realize();
	bSizer9->Add( m_sdbSizer1, 1, wxEXPAND|wxALL|wxALIGN_BOTTOM, 5 );
	
	bSizer8->Add( bSizer9, 0, wxEXPAND, 5 );
	
	this->SetSizer( bSizer8 );
	this->Layout();
	
	// Connect Events
	m_roserrB->Connect( wxEVT_COMMAND_BUTTON_CLICKED, wxCommandEventHandler( RoserrSetupDialogBase::OnClick ), NULL, this );
	m_sdbSizer1Cancel->Connect( wxEVT_COMMAND_BUTTON_CLICKED, wxCommandEventHandler( RoserrSetupDialogBase::OnCancel ), NULL, this );
	m_sdbSizer1OK->Connect( wxEVT_COMMAND_BUTTON_CLICKED, wxCommandEventHandler( RoserrSetupDialogBase::OnOk ), NULL, this );
}

RoserrSetupDialogBase::~RoserrSetupDialogBase()
{
	// Disconnect Events
	m_roserrB->Disconnect( wxEVT_COMMAND_BUTTON_CLICKED, wxCommandEventHandler( RoserrSetupDialogBase::OnClick ), NULL, this );
	m_sdbSizer1Cancel->Disconnect( wxEVT_COMMAND_BUTTON_CLICKED, wxCommandEventHandler( RoserrSetupDialogBase::OnCancel ), NULL, this );
	m_sdbSizer1OK->Disconnect( wxEVT_COMMAND_BUTTON_CLICKED, wxCommandEventHandler( RoserrSetupDialogBase::OnOk ), NULL, this );
}
