///////////////////////////////////////////////////////////////////////////
// C++ code generated with wxFormBuilder (version Apr 16 2008)
// http://www.wxformbuilder.org/
//
// PLEASE DO "NOT" EDIT THIS FILE!
///////////////////////////////////////////////////////////////////////////

#ifndef __GenTestTopicDisplay__
#define __GenTestTopicDisplay__

#include <wx/string.h>
#include <wx/frame.h>
#include <wx/gdicmn.h>
#include <wx/font.h>
#include <wx/colour.h>
#include <wx/settings.h>

///////////////////////////////////////////////////////////////////////////


///////////////////////////////////////////////////////////////////////////////
/// Class GenTestTopicDisplay
///////////////////////////////////////////////////////////////////////////////
class GenTestTopicDisplay : public wxFrame 
{
	private:
	
	protected:
		
		// Virtual event handlers, overide them in your derived class
		virtual void onClose( wxCloseEvent& event ){ event.Skip(); }
		
	
	public:
		GenTestTopicDisplay( wxWindow* parent, wxWindowID id = wxID_ANY, const wxString& title = wxEmptyString, const wxPoint& pos = wxDefaultPosition, const wxSize& size = wxSize( 500,300 ), long style = wxDEFAULT_FRAME_STYLE|wxTAB_TRAVERSAL );
		~GenTestTopicDisplay();
	
};

#endif //__GenTestTopicDisplay__
