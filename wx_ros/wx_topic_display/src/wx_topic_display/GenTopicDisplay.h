///////////////////////////////////////////////////////////////////////////
// C++ code generated with wxFormBuilder (version Apr 16 2008)
// http://www.wxformbuilder.org/
//
// PLEASE DO "NOT" EDIT THIS FILE!
///////////////////////////////////////////////////////////////////////////

#ifndef __GenTopicDisplay__
#define __GenTopicDisplay__

#include <wx/treectrl.h>
#include <wx/gdicmn.h>
#include <wx/font.h>
#include <wx/colour.h>
#include <wx/settings.h>
#include <wx/string.h>
#include <wx/sizer.h>
#include <wx/statbox.h>
#include <wx/panel.h>

///////////////////////////////////////////////////////////////////////////


///////////////////////////////////////////////////////////////////////////////
/// Class GenTopicDisplay
///////////////////////////////////////////////////////////////////////////////
class GenTopicDisplay : public wxPanel 
{
	private:
	
	protected:
		wxTreeCtrl* topicTree;
		
		// Virtual event handlers, overide them in your derived class
		virtual void checkIsTopic( wxTreeEvent& event ){ event.Skip(); }
		
	
	public:
		GenTopicDisplay( wxWindow* parent, wxWindowID id = wxID_ANY, const wxPoint& pos = wxDefaultPosition, const wxSize& size = wxSize( 500,300 ), long style = wxTAB_TRAVERSAL );
		~GenTopicDisplay();
	
};

#endif //__GenTopicDisplay__
