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
	
	wxArrayString displays_Choices;
	displays_ = new wxCheckListBox( m_panel5, wxID_ANY, wxDefaultPosition, wxDefaultSize, displays_Choices, 0 );
	displays_->SetMinSize( wxSize( 150,-1 ) );
	
	bSizer8->Add( displays_, 1, wxALL|wxEXPAND, 5 );
	
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
	
	properties_panel_ = new wxPanel( m_panel6, wxID_ANY, wxDefaultPosition, wxDefaultSize, wxTAB_TRAVERSAL );
	properties_panel_sizer_ = new wxBoxSizer( wxVERTICAL );
	
	properties_panel_->SetSizer( properties_panel_sizer_ );
	properties_panel_->Layout();
	properties_panel_sizer_->Fit( properties_panel_ );
	bSizer9->Add( properties_panel_, 1, wxEXPAND | wxALL, 5 );
	
	m_panel6->SetSizer( bSizer9 );
	m_panel6->Layout();
	bSizer9->Fit( m_panel6 );
	m_splitter2->SplitHorizontally( m_panel5, m_panel6, 0 );
	bSizer25->Add( m_splitter2, 1, wxEXPAND, 5 );
	
	m_panel3->SetSizer( bSizer25 );
	m_panel3->Layout();
	bSizer25->Fit( m_panel3 );
	render_panel_ = new wxPanel( m_splitter1, wxID_ANY, wxDefaultPosition, wxDefaultSize, wxTAB_TRAVERSAL );
	render_sizer_ = new wxBoxSizer( wxVERTICAL );
	
	views_ = new wxToolBar( render_panel_, wxID_ANY, wxDefaultPosition, wxDefaultSize, wxTB_HORIZONTAL|wxTB_NOICONS|wxTB_TEXT ); 
	views_->Realize();
	
	render_sizer_->Add( views_, 0, wxEXPAND, 0 );
	
	render_panel_->SetSizer( render_sizer_ );
	render_panel_->Layout();
	render_sizer_->Fit( render_panel_ );
	m_splitter1->SplitVertically( m_panel3, render_panel_, 200 );
	bSizer23->Add( m_splitter1, 1, wxEXPAND, 5 );
	
	this->SetSizer( bSizer23 );
	this->Layout();
}

VisualizationPanelGenerated::~VisualizationPanelGenerated()
{
}
