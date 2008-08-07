///////////////////////////////////////////////////////////////////////////
// C++ code generated with wxFormBuilder (version Apr 21 2008)
// http://www.wxformbuilder.org/
//
// PLEASE DO "NOT" EDIT THIS FILE!
///////////////////////////////////////////////////////////////////////////

#ifndef __GenTestTopicDisplay__
#define __GenTestTopicDisplay__

#include <wx/panel.h>
#include <wx/gdicmn.h>
#include <wx/font.h>
#include <wx/colour.h>
#include <wx/settings.h>
#include <wx/string.h>
#include <wx/button.h>
#include <wx/sizer.h>
#include <wx/statbox.h>
#include <wx/frame.h>

///////////////////////////////////////////////////////////////////////////


///////////////////////////////////////////////////////////////////////////////
/// Class GenTestTopicDisplay
///////////////////////////////////////////////////////////////////////////////
class GenTestTopicDisplay : public wxFrame 
{
	private:
	
	protected:
		wxPanel* topicPanel;
		wxPanel* topicPanel2;
		wxButton* m_button1;
		wxButton* m_Browse;
		
		// Virtual event handlers, overide them in your derived class
		virtual void onClose( wxCloseEvent& event ){ event.Skip(); }
		virtual void printSelections( wxCommandEvent& event ){ event.Skip(); }
		virtual void browse( wxCommandEvent& event ){ event.Skip(); }
		
	
	public:
		GenTestTopicDisplay( wxWindow* parent, wxWindowID id = wxID_ANY, const wxString& title = wxEmptyString, const wxPoint& pos = wxDefaultPosition, const wxSize& size = wxSize( 500,300 ), long style = wxDEFAULT_FRAME_STYLE|wxTAB_TRAVERSAL );
		~GenTestTopicDisplay();
	
};

#endif //__GenTestTopicDisplay__
