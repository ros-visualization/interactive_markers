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
	wxStaticBoxSizer* sbSizer1;
	sbSizer1 = new wxStaticBoxSizer( new wxStaticBox( this, wxID_ANY, wxEmptyString ), wxVERTICAL );
	
	topicTree = new wxTreeCtrl( this, wxID_ANY, wxDefaultPosition, wxDefaultSize, wxTR_DEFAULT_STYLE|wxTR_HIDE_ROOT|wxTR_MULTIPLE );
	sbSizer1->Add( topicTree, 1, wxALL|wxEXPAND, 5 );
	
	this->SetSizer( sbSizer1 );
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
