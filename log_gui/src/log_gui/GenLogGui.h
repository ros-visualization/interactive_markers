///////////////////////////////////////////////////////////////////////////
// C++ code generated with wxFormBuilder (version Apr 21 2008)
// http://www.wxformbuilder.org/
//
// PLEASE DO "NOT" EDIT THIS FILE!
///////////////////////////////////////////////////////////////////////////

#ifndef __GenLogGui__
#define __GenLogGui__

#include <wx/string.h>
#include <wx/menu.h>
#include <wx/gdicmn.h>
#include <wx/font.h>
#include <wx/colour.h>
#include <wx/settings.h>
#include <wx/statusbr.h>
#include <wx/textctrl.h>
#include <wx/button.h>
#include <wx/sizer.h>
#include <wx/panel.h>
#include <wx/frame.h>

///////////////////////////////////////////////////////////////////////////


///////////////////////////////////////////////////////////////////////////////
/// Class GenLogGui
///////////////////////////////////////////////////////////////////////////////
class GenLogGui : public wxFrame 
{
	private:
	
	protected:
		wxMenuBar* m_menubar1;
		wxMenu* m_menu1;
		wxStatusBar* statusBar;
		wxTextCtrl* logDir;
		wxButton* dirButton;
		wxPanel* topicPanel;
		wxButton* startLogButton;
		wxButton* stopLogButton;
		
		// Virtual event handlers, overide them in your derived class
		virtual void onClose( wxCloseEvent& event ){ event.Skip(); }
		virtual void openDir( wxCommandEvent& event ){ event.Skip(); }
		virtual void startLogging( wxCommandEvent& event ){ event.Skip(); }
		virtual void stopLogging( wxCommandEvent& event ){ event.Skip(); }
		
	
	public:
		GenLogGui( wxWindow* parent, wxWindowID id = wxID_ANY, const wxString& title = wxT("ROS Logging"), const wxPoint& pos = wxDefaultPosition, const wxSize& size = wxSize( 621,347 ), long style = wxDEFAULT_FRAME_STYLE|wxTAB_TRAVERSAL );
		~GenLogGui();
	
};

#endif //__GenLogGui__
