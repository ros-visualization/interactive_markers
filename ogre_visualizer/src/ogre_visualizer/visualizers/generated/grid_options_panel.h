///////////////////////////////////////////////////////////////////////////
// C++ code generated with wxFormBuilder (version Apr 21 2008)
// http://www.wxformbuilder.org/
//
// PLEASE DO "NOT" EDIT THIS FILE!
///////////////////////////////////////////////////////////////////////////

#ifndef __grid_options_panel__
#define __grid_options_panel__

#include <wx/string.h>
#include <wx/stattext.h>
#include <wx/gdicmn.h>
#include <wx/font.h>
#include <wx/colour.h>
#include <wx/settings.h>
#include <wx/textctrl.h>
#include <wx/sizer.h>
#include <wx/panel.h>

///////////////////////////////////////////////////////////////////////////


///////////////////////////////////////////////////////////////////////////////
/// Class GridOptionsPanelGenerated
///////////////////////////////////////////////////////////////////////////////
class GridOptionsPanelGenerated : public wxPanel 
{
	private:
	
	protected:
		wxStaticText* m_staticText3;
		wxStaticText* m_staticText31;
		wxStaticText* m_staticText32;
		
		// Virtual event handlers, overide them in your derived class
		virtual void OnKillFocus( wxFocusEvent& event ){ event.Skip(); }
		virtual void OnTextEnter( wxCommandEvent& event ){ event.Skip(); }
		
	
	public:
		wxTextCtrl* m_CellCount;
		wxTextCtrl* m_CellSize;
		wxTextCtrl* m_R;
		wxTextCtrl* m_G;
		wxTextCtrl* m_B;
		GridOptionsPanelGenerated( wxWindow* parent, wxWindowID id = wxID_ANY, const wxPoint& pos = wxDefaultPosition, const wxSize& size = wxSize( 199,132 ), long style = wxTAB_TRAVERSAL );
		~GridOptionsPanelGenerated();
	
};

#endif //__grid_options_panel__
