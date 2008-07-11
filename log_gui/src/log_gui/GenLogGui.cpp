///////////////////////////////////////////////////////////////////////////
// C++ code generated with wxFormBuilder (version Apr 21 2008)
// http://www.wxformbuilder.org/
//
// PLEASE DO "NOT" EDIT THIS FILE!
///////////////////////////////////////////////////////////////////////////

#include "GenLogGui.h"

///////////////////////////////////////////////////////////////////////////

GenLogGui::GenLogGui( wxWindow* parent, wxWindowID id, const wxString& title, const wxPoint& pos, const wxSize& size, long style ) : wxFrame( parent, id, title, pos, size, style )
{
	this->SetSizeHints( wxDefaultSize, wxDefaultSize );
	
	m_menubar1 = new wxMenuBar( 0 );
	m_menu1 = new wxMenu();
	m_menubar1->Append( m_menu1, wxT("File") );
	
	this->SetMenuBar( m_menubar1 );
	
	statusBar = this->CreateStatusBar( 1, wxST_SIZEGRIP, wxID_ANY );
	wxBoxSizer* bSizer2;
	bSizer2 = new wxBoxSizer( wxVERTICAL );
	
	wxBoxSizer* bSizer3;
	bSizer3 = new wxBoxSizer( wxHORIZONTAL );
	
	logDir = new wxTextCtrl( this, wxID_ANY, wxEmptyString, wxDefaultPosition, wxDefaultSize, 0 );
	bSizer3->Add( logDir, 1, wxALL, 5 );
	
	dirButton = new wxButton( this, wxID_ANY, wxT(".."), wxDefaultPosition, wxSize( 25,25 ), 0 );
	dirButton->SetFont( wxFont( wxNORMAL_FONT->GetPointSize(), 70, 90, 92, false, wxEmptyString ) );
	
	bSizer3->Add( dirButton, 0, wxALL, 5 );
	
	bSizer2->Add( bSizer3, 0, wxEXPAND, 5 );
	
	wxBoxSizer* bSizer1;
	bSizer1 = new wxBoxSizer( wxHORIZONTAL );
	
	topicPanel = new wxPanel( this, wxID_ANY, wxDefaultPosition, wxDefaultSize, wxTAB_TRAVERSAL );
	bSizer1->Add( topicPanel, 1, wxALL|wxEXPAND, 5 );
	
	wxBoxSizer* bSizer4;
	bSizer4 = new wxBoxSizer( wxVERTICAL );
	
	startLogButton = new wxButton( this, wxID_ANY, wxT("start logging"), wxDefaultPosition, wxDefaultSize, 0 );
	bSizer4->Add( startLogButton, 0, wxALL, 5 );
	
	stopLogButton = new wxButton( this, wxID_ANY, wxT("stop logging"), wxDefaultPosition, wxDefaultSize, 0 );
	stopLogButton->Enable( false );
	
	bSizer4->Add( stopLogButton, 0, wxALL, 5 );
	
	bSizer1->Add( bSizer4, 0, wxEXPAND, 5 );
	
	bSizer2->Add( bSizer1, 1, wxEXPAND, 5 );
	
	this->SetSizer( bSizer2 );
	this->Layout();
	
	// Connect Events
	this->Connect( wxEVT_CLOSE_WINDOW, wxCloseEventHandler( GenLogGui::onClose ) );
	dirButton->Connect( wxEVT_COMMAND_BUTTON_CLICKED, wxCommandEventHandler( GenLogGui::openDir ), NULL, this );
	startLogButton->Connect( wxEVT_COMMAND_BUTTON_CLICKED, wxCommandEventHandler( GenLogGui::startLogging ), NULL, this );
	stopLogButton->Connect( wxEVT_COMMAND_BUTTON_CLICKED, wxCommandEventHandler( GenLogGui::stopLogging ), NULL, this );
}

GenLogGui::~GenLogGui()
{
	// Disconnect Events
	this->Disconnect( wxEVT_CLOSE_WINDOW, wxCloseEventHandler( GenLogGui::onClose ) );
	dirButton->Disconnect( wxEVT_COMMAND_BUTTON_CLICKED, wxCommandEventHandler( GenLogGui::openDir ), NULL, this );
	startLogButton->Disconnect( wxEVT_COMMAND_BUTTON_CLICKED, wxCommandEventHandler( GenLogGui::startLogging ), NULL, this );
	stopLogButton->Disconnect( wxEVT_COMMAND_BUTTON_CLICKED, wxCommandEventHandler( GenLogGui::stopLogging ), NULL, this );
}
