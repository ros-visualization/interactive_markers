///////////////////////////////////////////////////////////////////////////
// C++ code generated with wxFormBuilder (version Apr 21 2008)
// http://www.wxformbuilder.org/
//
// PLEASE DO "NOT" EDIT THIS FILE!
///////////////////////////////////////////////////////////////////////////

#include "visualization_panel_generated.h"

///////////////////////////////////////////////////////////////////////////

VisualizationPanelGenerated::VisualizationPanelGenerated( wxWindow* parent, wxWindowID id, const wxPoint& pos, const wxSize& size, long style ) : wxPanel( parent, id, pos, size, style )
{
	wxBoxSizer* bSizer23;
	bSizer23 = new wxBoxSizer( wxVERTICAL );
	
	m_splitter1 = new wxSplitterWindow( this, wxID_ANY, wxDefaultPosition, wxDefaultSize, wxSP_3D );
	m_splitter1->Connect( wxEVT_IDLE, wxIdleEventHandler( VisualizationPanelGenerated::m_splitter1OnIdle ), NULL, this );
	m_panel3 = new wxPanel( m_splitter1, wxID_ANY, wxDefaultPosition, wxDefaultSize, wxTAB_TRAVERSAL );
	m_panel3->SetMinSize( wxSize( 200,-1 ) );
	
	wxBoxSizer* bSizer25;
	bSizer25 = new wxBoxSizer( wxVERTICAL );
	
	bSizer25->SetMinSize( wxSize( 200,200 ) ); 
	m_splitter2 = new wxSplitterWindow( m_panel3, wxID_ANY, wxDefaultPosition, wxDefaultSize, wxSP_3D );
	m_splitter2->Connect( wxEVT_IDLE, wxIdleEventHandler( VisualizationPanelGenerated::m_splitter2OnIdle ), NULL, this );
	m_panel5 = new wxPanel( m_splitter2, wxID_ANY, wxDefaultPosition, wxDefaultSize, wxTAB_TRAVERSAL );
	m_panel5->SetMinSize( wxSize( 200,100 ) );
	
	wxBoxSizer* bSizer8;
	bSizer8 = new wxBoxSizer( wxVERTICAL );
	
	m_staticText1 = new wxStaticText( m_panel5, wxID_ANY, wxT("Displays"), wxDefaultPosition, wxDefaultSize, 0 );
	m_staticText1->Wrap( -1 );
	bSizer8->Add( m_staticText1, 0, wxALL, 5 );
	
	wxArrayString m_DisplaysChoices;
	m_Displays = new wxCheckListBox( m_panel5, wxID_ANY, wxDefaultPosition, wxDefaultSize, m_DisplaysChoices, 0 );
	m_Displays->SetMinSize( wxSize( 150,-1 ) );
	
	bSizer8->Add( m_Displays, 1, wxALL|wxEXPAND, 5 );
	
	m_panel5->SetSizer( bSizer8 );
	m_panel5->Layout();
	bSizer8->Fit( m_panel5 );
	m_panel6 = new wxPanel( m_splitter2, wxID_ANY, wxDefaultPosition, wxDefaultSize, wxTAB_TRAVERSAL );
	m_panel6->SetMinSize( wxSize( 200,100 ) );
	
	wxBoxSizer* bSizer9;
	bSizer9 = new wxBoxSizer( wxVERTICAL );
	
	m_staticText2 = new wxStaticText( m_panel6, wxID_ANY, wxT("Display Properties"), wxDefaultPosition, wxDefaultSize, 0 );
	m_staticText2->Wrap( -1 );
	bSizer9->Add( m_staticText2, 0, wxALL, 5 );
	
	m_PropertiesPanel = new wxPanel( m_panel6, wxID_ANY, wxDefaultPosition, wxDefaultSize, wxTAB_TRAVERSAL );
	m_PropertiesPanelSizer = new wxBoxSizer( wxVERTICAL );
	
	m_PropertiesPanel->SetSizer( m_PropertiesPanelSizer );
	m_PropertiesPanel->Layout();
	m_PropertiesPanelSizer->Fit( m_PropertiesPanel );
	bSizer9->Add( m_PropertiesPanel, 1, wxEXPAND | wxALL, 5 );
	
	m_panel6->SetSizer( bSizer9 );
	m_panel6->Layout();
	bSizer9->Fit( m_panel6 );
	m_splitter2->SplitHorizontally( m_panel5, m_panel6, 0 );
	bSizer25->Add( m_splitter2, 1, wxEXPAND, 5 );
	
	m_panel3->SetSizer( bSizer25 );
	m_panel3->Layout();
	bSizer25->Fit( m_panel3 );
	m_3DPanel = new wxPanel( m_splitter1, wxID_ANY, wxDefaultPosition, wxDefaultSize, wxTAB_TRAVERSAL );
	m_3DSizer = new wxBoxSizer( wxVERTICAL );
	
	m_Views = new wxToolBar( m_3DPanel, wxID_ANY, wxDefaultPosition, wxDefaultSize, wxTB_HORIZONTAL|wxTB_NOICONS|wxTB_TEXT ); 
	m_Views->Realize();
	
	m_3DSizer->Add( m_Views, 0, wxEXPAND, 0 );
	
	m_3DPanel->SetSizer( m_3DSizer );
	m_3DPanel->Layout();
	m_3DSizer->Fit( m_3DPanel );
	m_splitter1->SplitVertically( m_panel3, m_3DPanel, 200 );
	bSizer23->Add( m_splitter1, 1, wxEXPAND, 5 );
	
	this->SetSizer( bSizer23 );
	this->Layout();
}

VisualizationPanelGenerated::~VisualizationPanelGenerated()
{
}
