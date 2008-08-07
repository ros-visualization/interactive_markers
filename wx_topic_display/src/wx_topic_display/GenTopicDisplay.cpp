///////////////////////////////////////////////////////////////////////////
// C++ code generated with wxFormBuilder (version Apr 21 2008)
// http://www.wxformbuilder.org/
//
// PLEASE DO "NOT" EDIT THIS FILE!
///////////////////////////////////////////////////////////////////////////

#include "GenTopicDisplay.h"

///////////////////////////////////////////////////////////////////////////

GenTopicDisplay::GenTopicDisplay( wxWindow* parent, wxWindowID id, const wxPoint& pos, const wxSize& size, long style ) : wxPanel( parent, id, pos, size, style )
{
	wxBoxSizer* bSizer1;
	bSizer1 = new wxBoxSizer( wxVERTICAL );
	
	topicTree = new wxTreeCtrl( this, wxID_ANY, wxDefaultPosition, wxDefaultSize, wxTR_DEFAULT_STYLE|wxTR_HIDE_ROOT|wxTR_MULTIPLE );
	bSizer1->Add( topicTree, 1, wxALL|wxEXPAND, 5 );
	
	this->SetSizer( bSizer1 );
	this->Layout();
	
	// Connect Events
	topicTree->Connect( wxEVT_COMMAND_TREE_SEL_CHANGED, wxTreeEventHandler( GenTopicDisplay::checkIsTopic ), NULL, this );
	topicTree->Connect( wxEVT_COMMAND_TREE_SEL_CHANGING, wxTreeEventHandler( GenTopicDisplay::checkIsTopic ), NULL, this );
}

GenTopicDisplay::~GenTopicDisplay()
{
	// Disconnect Events
	topicTree->Disconnect( wxEVT_COMMAND_TREE_SEL_CHANGED, wxTreeEventHandler( GenTopicDisplay::checkIsTopic ), NULL, this );
	topicTree->Disconnect( wxEVT_COMMAND_TREE_SEL_CHANGING, wxTreeEventHandler( GenTopicDisplay::checkIsTopic ), NULL, this );
}

GenTopicDisplayDialog::GenTopicDisplayDialog( wxWindow* parent, wxWindowID id, const wxString& title, const wxPoint& pos, const wxSize& size, long style ) : wxDialog( parent, id, title, pos, size, style )
{
	this->SetSizeHints( wxDefaultSize, wxDefaultSize );
	
	wxBoxSizer* bSizer3;
	bSizer3 = new wxBoxSizer( wxVERTICAL );
	
	m_TreePanel = new wxPanel( this, wxID_ANY, wxDefaultPosition, wxDefaultSize, wxTAB_TRAVERSAL );
	bSizer3->Add( m_TreePanel, 1, wxEXPAND | wxALL, 5 );
	
	wxBoxSizer* bSizer4;
	bSizer4 = new wxBoxSizer( wxHORIZONTAL );
	
	m_OK = new wxButton( this, wxID_ANY, wxT("OK"), wxDefaultPosition, wxDefaultSize, 0 );
	bSizer4->Add( m_OK, 0, wxALL, 5 );
	
	m_Cancel = new wxButton( this, wxID_ANY, wxT("Cancel"), wxDefaultPosition, wxDefaultSize, 0 );
	bSizer4->Add( m_Cancel, 0, wxALL, 5 );
	
	bSizer3->Add( bSizer4, 0, wxALIGN_RIGHT, 5 );
	
	this->SetSizer( bSizer3 );
	this->Layout();
	
	// Connect Events
	m_OK->Connect( wxEVT_COMMAND_BUTTON_CLICKED, wxCommandEventHandler( GenTopicDisplayDialog::onOK ), NULL, this );
	m_Cancel->Connect( wxEVT_COMMAND_BUTTON_CLICKED, wxCommandEventHandler( GenTopicDisplayDialog::onCancel ), NULL, this );
}

GenTopicDisplayDialog::~GenTopicDisplayDialog()
{
	// Disconnect Events
	m_OK->Disconnect( wxEVT_COMMAND_BUTTON_CLICKED, wxCommandEventHandler( GenTopicDisplayDialog::onOK ), NULL, this );
	m_Cancel->Disconnect( wxEVT_COMMAND_BUTTON_CLICKED, wxCommandEventHandler( GenTopicDisplayDialog::onCancel ), NULL, this );
}
