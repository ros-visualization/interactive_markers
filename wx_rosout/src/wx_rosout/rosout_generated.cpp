///////////////////////////////////////////////////////////////////////////
// C++ code generated with wxFormBuilder (version Apr 21 2008)
// http://www.wxformbuilder.org/
//
// PLEASE DO "NOT" EDIT THIS FILE!
///////////////////////////////////////////////////////////////////////////

#include "rosout_generated.h"

///////////////////////////////////////////////////////////////////////////

RosoutPanelBase::RosoutPanelBase( wxWindow* parent, wxWindowID id, const wxPoint& pos, const wxSize& size, long style ) : wxPanel( parent, id, pos, size, style )
{
	this->SetMinSize( wxSize( 273,138 ) );
	
	wxBoxSizer* bSizer2;
	bSizer2 = new wxBoxSizer( wxVERTICAL );
	
	wxBoxSizer* bSizer3;
	bSizer3 = new wxBoxSizer( wxHORIZONTAL );
	
	wxBoxSizer* bSizer8;
	bSizer8 = new wxBoxSizer( wxHORIZONTAL );
	
	enable_checkbox_ = new wxCheckBox( this, wxID_ANY, wxT("Enable"), wxDefaultPosition, wxDefaultSize, 0 );
	
	bSizer8->Add( enable_checkbox_, 0, wxALL|wxALIGN_CENTER_VERTICAL, 5 );
	
	pause_toggle_ = new wxToggleButton( this, wxID_ANY, wxT("Pause"), wxDefaultPosition, wxDefaultSize, 0 );
	bSizer8->Add( pause_toggle_, 0, wxALL, 5 );
	
	clear_button_ = new wxButton( this, wxID_ANY, wxT("Clear"), wxDefaultPosition, wxDefaultSize, 0 );
	bSizer8->Add( clear_button_, 0, wxALL, 5 );
	
	bSizer3->Add( bSizer8, 1, wxEXPAND, 5 );
	
	wxBoxSizer* bSizer4;
	bSizer4 = new wxBoxSizer( wxVERTICAL );
	
	setup_button_ = new wxButton( this, wxID_ANY, wxT("Setup"), wxDefaultPosition, wxDefaultSize, 0 );
	bSizer4->Add( setup_button_, 0, wxALL|wxALIGN_RIGHT, 5 );
	
	bSizer3->Add( bSizer4, 1, wxEXPAND, 5 );
	
	bSizer2->Add( bSizer3, 0, wxEXPAND, 5 );
	
	book_ = new wxChoicebook( this, wxID_ANY, wxDefaultPosition, wxDefaultSize, wxCHB_DEFAULT );
	bSizer2->Add( book_, 1, wxEXPAND | wxALL, 5 );
	
	this->SetSizer( bSizer2 );
	this->Layout();
	
	// Connect Events
	enable_checkbox_->Connect( wxEVT_COMMAND_CHECKBOX_CLICKED, wxCommandEventHandler( RosoutPanelBase::onEnable ), NULL, this );
	pause_toggle_->Connect( wxEVT_COMMAND_TOGGLEBUTTON_CLICKED, wxCommandEventHandler( RosoutPanelBase::onPauseToggled ), NULL, this );
	clear_button_->Connect( wxEVT_COMMAND_BUTTON_CLICKED, wxCommandEventHandler( RosoutPanelBase::onClear ), NULL, this );
	setup_button_->Connect( wxEVT_COMMAND_BUTTON_CLICKED, wxCommandEventHandler( RosoutPanelBase::onSetup ), NULL, this );
}

RosoutPanelBase::~RosoutPanelBase()
{
	// Disconnect Events
	enable_checkbox_->Disconnect( wxEVT_COMMAND_CHECKBOX_CLICKED, wxCommandEventHandler( RosoutPanelBase::onEnable ), NULL, this );
	pause_toggle_->Disconnect( wxEVT_COMMAND_TOGGLEBUTTON_CLICKED, wxCommandEventHandler( RosoutPanelBase::onPauseToggled ), NULL, this );
	clear_button_->Disconnect( wxEVT_COMMAND_BUTTON_CLICKED, wxCommandEventHandler( RosoutPanelBase::onClear ), NULL, this );
	setup_button_->Disconnect( wxEVT_COMMAND_BUTTON_CLICKED, wxCommandEventHandler( RosoutPanelBase::onSetup ), NULL, this );
}

RosoutSetupDialogBase::RosoutSetupDialogBase( wxWindow* parent, wxWindowID id, const wxString& title, const wxPoint& pos, const wxSize& size, long style ) : wxDialog( parent, id, title, pos, size, style )
{
	this->SetSizeHints( wxSize( -1,-1 ), wxDefaultSize );
	
	wxBoxSizer* bSizer8;
	bSizer8 = new wxBoxSizer( wxVERTICAL );
	
	wxStaticBoxSizer* sbSizer2;
	sbSizer2 = new wxStaticBoxSizer( new wxStaticBox( this, wxID_ANY, wxT("Rosout Topic") ), wxVERTICAL );
	
	wxBoxSizer* bSizer91;
	bSizer91 = new wxBoxSizer( wxHORIZONTAL );
	
	topic_ = new wxTextCtrl( this, wxID_ANY, wxEmptyString, wxDefaultPosition, wxDefaultSize, wxTE_PROCESS_ENTER );
	bSizer91->Add( topic_, 1, wxALL|wxALIGN_CENTER_VERTICAL|wxEXPAND, 5 );
	
	topic_browse_button_ = new wxButton( this, wxID_ANY, wxT(" ... "), wxDefaultPosition, wxDefaultSize, wxBU_EXACTFIT );
	bSizer91->Add( topic_browse_button_, 0, wxALL|wxALIGN_CENTER_VERTICAL, 5 );
	
	sbSizer2->Add( bSizer91, 1, wxEXPAND, 5 );
	
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
	topic_browse_button_->Connect( wxEVT_COMMAND_BUTTON_CLICKED, wxCommandEventHandler( RosoutSetupDialogBase::onTopicBrowse ), NULL, this );
	m_sdbSizer1Cancel->Connect( wxEVT_COMMAND_BUTTON_CLICKED, wxCommandEventHandler( RosoutSetupDialogBase::onCancel ), NULL, this );
	m_sdbSizer1OK->Connect( wxEVT_COMMAND_BUTTON_CLICKED, wxCommandEventHandler( RosoutSetupDialogBase::onOk ), NULL, this );
}

RosoutSetupDialogBase::~RosoutSetupDialogBase()
{
	// Disconnect Events
	topic_browse_button_->Disconnect( wxEVT_COMMAND_BUTTON_CLICKED, wxCommandEventHandler( RosoutSetupDialogBase::onTopicBrowse ), NULL, this );
	m_sdbSizer1Cancel->Disconnect( wxEVT_COMMAND_BUTTON_CLICKED, wxCommandEventHandler( RosoutSetupDialogBase::onCancel ), NULL, this );
	m_sdbSizer1OK->Disconnect( wxEVT_COMMAND_BUTTON_CLICKED, wxCommandEventHandler( RosoutSetupDialogBase::onOk ), NULL, this );
}
