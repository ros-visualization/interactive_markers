///////////////////////////////////////////////////////////////////////////
// C++ code generated with wxFormBuilder (version Apr 21 2008)
// http://www.wxformbuilder.org/
//
// PLEASE DO "NOT" EDIT THIS FILE!
///////////////////////////////////////////////////////////////////////////

#ifndef __visualization_panel_generated__
#define __visualization_panel_generated__

#include <wx/string.h>
#include <wx/stattext.h>
#include <wx/gdicmn.h>
#include <wx/font.h>
#include <wx/colour.h>
#include <wx/settings.h>
#include <wx/checklst.h>
#include <wx/sizer.h>
#include <wx/panel.h>
#include <wx/splitter.h>
#include <wx/toolbar.h>

///////////////////////////////////////////////////////////////////////////


///////////////////////////////////////////////////////////////////////////////
/// Class VisualizationPanelGenerated
///////////////////////////////////////////////////////////////////////////////
class VisualizationPanelGenerated : public wxPanel 
{
	private:
	
	protected:
		wxSplitterWindow* m_splitter1;
		wxPanel* m_panel3;
		wxSplitterWindow* m_splitter2;
		wxPanel* m_panel5;
		wxStaticText* m_staticText1;
		wxCheckListBox* m_Displays;
		wxPanel* m_panel6;
		wxStaticText* m_staticText2;
		wxPanel* m_PropertiesPanel;
		wxBoxSizer* m_PropertiesPanelSizer;
		wxPanel* m_3DPanel;
		wxBoxSizer* m_3DSizer;
		wxToolBar* m_Views;
	
	public:
		VisualizationPanelGenerated( wxWindow* parent, wxWindowID id = wxID_ANY, const wxPoint& pos = wxDefaultPosition, const wxSize& size = wxSize( 765,578 ), long style = wxTAB_TRAVERSAL );
		~VisualizationPanelGenerated();
		void m_splitter1OnIdle( wxIdleEvent& )
		{
		m_splitter1->SetSashPosition( 200 );
		m_splitter1->Disconnect( wxEVT_IDLE, wxIdleEventHandler( VisualizationPanelGenerated::m_splitter1OnIdle ), NULL, this );
		}
		
		void m_splitter2OnIdle( wxIdleEvent& )
		{
		m_splitter2->SetSashPosition( 0 );
		m_splitter2->Disconnect( wxEVT_IDLE, wxIdleEventHandler( VisualizationPanelGenerated::m_splitter2OnIdle ), NULL, this );
		}
		
	
};

#endif //__visualization_panel_generated__
