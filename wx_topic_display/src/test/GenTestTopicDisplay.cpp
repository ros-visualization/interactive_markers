///////////////////////////////////////////////////////////////////////////
// C++ code generated with wxFormBuilder (version Apr 21 2008)
// http://www.wxformbuilder.org/
//
// PLEASE DO "NOT" EDIT THIS FILE!
///////////////////////////////////////////////////////////////////////////

#include "GenTestTopicDisplay.h"

///////////////////////////////////////////////////////////////////////////

GenTestTopicDisplay::GenTestTopicDisplay( wxWindow* parent, wxWindowID id, const wxString& title, const wxPoint& pos, const wxSize& size, long style ) : wxFrame( parent, id, title, pos, size, style )
{
	this->SetSizeHints( wxDefaultSize, wxDefaultSize );
	
	wxStaticBoxSizer* sbSizer1;
	sbSizer1 = new wxStaticBoxSizer( new wxStaticBox( this, wxID_ANY, wxEmptyString ), wxHORIZONTAL );
	
	topicPanel = new wxPanel( this, wxID_ANY, wxDefaultPosition, wxDefaultSize, wxTAB_TRAVERSAL );
	sbSizer1->Add( topicPanel, 1, wxALL|wxEXPAND, 5 );
	
	topicPanel2 = new wxPanel( this, wxID_ANY, wxDefaultPosition, wxDefaultSize, wxTAB_TRAVERSAL );
	sbSizer1->Add( topicPanel2, 1, wxEXPAND | wxALL, 5 );
	
	wxBoxSizer* bSizer1;
	bSizer1 = new wxBoxSizer( wxVERTICAL );
	
	m_button1 = new wxButton( this, wxID_ANY, wxT("SelectionButton"), wxDefaultPosition, wxDefaultSize, 0 );
	bSizer1->Add( m_button1, 0, wxALL, 5 );
	
	m_Browse = new wxButton( this, wxID_ANY, wxT("Browse Dialog"), wxDefaultPosition, wxDefaultSize, 0 );
	bSizer1->Add( m_Browse, 0, wxALL, 5 );
	
	sbSizer1->Add( bSizer1, 0, wxEXPAND, 5 );
	
	this->SetSizer( sbSizer1 );
	this->Layout();
	
	// Connect Events
	this->Connect( wxEVT_CLOSE_WINDOW, wxCloseEventHandler( GenTestTopicDisplay::onClose ) );
	m_button1->Connect( wxEVT_COMMAND_BUTTON_CLICKED, wxCommandEventHandler( GenTestTopicDisplay::printSelections ), NULL, this );
	m_Browse->Connect( wxEVT_COMMAND_BUTTON_CLICKED, wxCommandEventHandler( GenTestTopicDisplay::browse ), NULL, this );
}

GenTestTopicDisplay::~GenTestTopicDisplay()
{
	// Disconnect Events
	this->Disconnect( wxEVT_CLOSE_WINDOW, wxCloseEventHandler( GenTestTopicDisplay::onClose ) );
	m_button1->Disconnect( wxEVT_COMMAND_BUTTON_CLICKED, wxCommandEventHandler( GenTestTopicDisplay::printSelections ), NULL, this );
	m_Browse->Disconnect( wxEVT_COMMAND_BUTTON_CLICKED, wxCommandEventHandler( GenTestTopicDisplay::browse ), NULL, this );
}
