///////////////////////////////////////////////////////////////////////////
// C++ code generated with wxFormBuilder (version Apr 21 2008)
// http://www.wxformbuilder.org/
//
// PLEASE DO "NOT" EDIT THIS FILE!
///////////////////////////////////////////////////////////////////////////

#include "grid_options_panel.h"

///////////////////////////////////////////////////////////////////////////

GridOptionsPanelGenerated::GridOptionsPanelGenerated( wxWindow* parent, wxWindowID id, const wxPoint& pos, const wxSize& size, long style ) : wxPanel( parent, id, pos, size, style )
{
	wxBoxSizer* bSizer6;
	bSizer6 = new wxBoxSizer( wxVERTICAL );
	
	wxBoxSizer* bSizer7;
	bSizer7 = new wxBoxSizer( wxHORIZONTAL );
	
	m_staticText3 = new wxStaticText( this, wxID_ANY, wxT("Cell Count"), wxDefaultPosition, wxDefaultSize, 0 );
	m_staticText3->Wrap( -1 );
	bSizer7->Add( m_staticText3, 1, wxALL|wxALIGN_CENTER_VERTICAL, 5 );
	
	m_CellCount = new wxTextCtrl( this, wxID_ANY, wxEmptyString, wxDefaultPosition, wxDefaultSize, wxTE_PROCESS_ENTER );
	bSizer7->Add( m_CellCount, 0, wxALL, 5 );
	
	bSizer6->Add( bSizer7, 0, wxEXPAND, 5 );
	
	wxBoxSizer* bSizer71;
	bSizer71 = new wxBoxSizer( wxHORIZONTAL );
	
	m_staticText31 = new wxStaticText( this, wxID_ANY, wxT("Cell Size"), wxDefaultPosition, wxDefaultSize, 0 );
	m_staticText31->Wrap( -1 );
	bSizer71->Add( m_staticText31, 1, wxALL|wxALIGN_CENTER_VERTICAL, 5 );
	
	m_CellSize = new wxTextCtrl( this, wxID_ANY, wxEmptyString, wxDefaultPosition, wxDefaultSize, wxTE_PROCESS_ENTER );
	bSizer71->Add( m_CellSize, 0, wxALL, 5 );
	
	bSizer6->Add( bSizer71, 0, wxEXPAND, 5 );
	
	wxBoxSizer* bSizer72;
	bSizer72 = new wxBoxSizer( wxHORIZONTAL );
	
	m_staticText32 = new wxStaticText( this, wxID_ANY, wxT("Color"), wxDefaultPosition, wxDefaultSize, 0 );
	m_staticText32->Wrap( -1 );
	bSizer72->Add( m_staticText32, 1, wxALL|wxALIGN_CENTER_VERTICAL, 5 );
	
	m_R = new wxTextCtrl( this, wxID_ANY, wxEmptyString, wxDefaultPosition, wxDefaultSize, wxTE_PROCESS_ENTER );
	m_R->SetMinSize( wxSize( 40,-1 ) );
	
	bSizer72->Add( m_R, 0, wxALL, 5 );
	
	m_G = new wxTextCtrl( this, wxID_ANY, wxEmptyString, wxDefaultPosition, wxDefaultSize, wxTE_PROCESS_ENTER );
	m_G->SetMinSize( wxSize( 40,-1 ) );
	
	bSizer72->Add( m_G, 0, wxALL, 5 );
	
	m_B = new wxTextCtrl( this, wxID_ANY, wxEmptyString, wxDefaultPosition, wxDefaultSize, wxTE_PROCESS_ENTER );
	m_B->SetMinSize( wxSize( 40,-1 ) );
	
	bSizer72->Add( m_B, 0, wxALL, 5 );
	
	bSizer6->Add( bSizer72, 0, wxEXPAND, 5 );
	
	this->SetSizer( bSizer6 );
	this->Layout();
	
	// Connect Events
	m_CellCount->Connect( wxEVT_KILL_FOCUS, wxFocusEventHandler( GridOptionsPanelGenerated::OnKillFocus ), NULL, this );
	m_CellCount->Connect( wxEVT_COMMAND_TEXT_ENTER, wxCommandEventHandler( GridOptionsPanelGenerated::OnTextEnter ), NULL, this );
	m_CellSize->Connect( wxEVT_KILL_FOCUS, wxFocusEventHandler( GridOptionsPanelGenerated::OnKillFocus ), NULL, this );
	m_CellSize->Connect( wxEVT_COMMAND_TEXT_ENTER, wxCommandEventHandler( GridOptionsPanelGenerated::OnTextEnter ), NULL, this );
	m_R->Connect( wxEVT_KILL_FOCUS, wxFocusEventHandler( GridOptionsPanelGenerated::OnKillFocus ), NULL, this );
	m_R->Connect( wxEVT_COMMAND_TEXT_ENTER, wxCommandEventHandler( GridOptionsPanelGenerated::OnTextEnter ), NULL, this );
	m_G->Connect( wxEVT_KILL_FOCUS, wxFocusEventHandler( GridOptionsPanelGenerated::OnKillFocus ), NULL, this );
	m_G->Connect( wxEVT_COMMAND_TEXT_ENTER, wxCommandEventHandler( GridOptionsPanelGenerated::OnTextEnter ), NULL, this );
	m_B->Connect( wxEVT_KILL_FOCUS, wxFocusEventHandler( GridOptionsPanelGenerated::OnKillFocus ), NULL, this );
	m_B->Connect( wxEVT_COMMAND_TEXT_ENTER, wxCommandEventHandler( GridOptionsPanelGenerated::OnTextEnter ), NULL, this );
}

GridOptionsPanelGenerated::~GridOptionsPanelGenerated()
{
	// Disconnect Events
	m_CellCount->Disconnect( wxEVT_KILL_FOCUS, wxFocusEventHandler( GridOptionsPanelGenerated::OnKillFocus ), NULL, this );
	m_CellCount->Disconnect( wxEVT_COMMAND_TEXT_ENTER, wxCommandEventHandler( GridOptionsPanelGenerated::OnTextEnter ), NULL, this );
	m_CellSize->Disconnect( wxEVT_KILL_FOCUS, wxFocusEventHandler( GridOptionsPanelGenerated::OnKillFocus ), NULL, this );
	m_CellSize->Disconnect( wxEVT_COMMAND_TEXT_ENTER, wxCommandEventHandler( GridOptionsPanelGenerated::OnTextEnter ), NULL, this );
	m_R->Disconnect( wxEVT_KILL_FOCUS, wxFocusEventHandler( GridOptionsPanelGenerated::OnKillFocus ), NULL, this );
	m_R->Disconnect( wxEVT_COMMAND_TEXT_ENTER, wxCommandEventHandler( GridOptionsPanelGenerated::OnTextEnter ), NULL, this );
	m_G->Disconnect( wxEVT_KILL_FOCUS, wxFocusEventHandler( GridOptionsPanelGenerated::OnKillFocus ), NULL, this );
	m_G->Disconnect( wxEVT_COMMAND_TEXT_ENTER, wxCommandEventHandler( GridOptionsPanelGenerated::OnTextEnter ), NULL, this );
	m_B->Disconnect( wxEVT_KILL_FOCUS, wxFocusEventHandler( GridOptionsPanelGenerated::OnKillFocus ), NULL, this );
	m_B->Disconnect( wxEVT_COMMAND_TEXT_ENTER, wxCommandEventHandler( GridOptionsPanelGenerated::OnTextEnter ), NULL, this );
}
