///////////////////////////////////////////////////////////////////////////
// C++ code generated with wxFormBuilder (version Apr 21 2008)
// http://www.wxformbuilder.org/
//
// PLEASE DO "NOT" EDIT THIS FILE!
///////////////////////////////////////////////////////////////////////////

#ifndef __rosout_generated__
#define __rosout_generated__

#include <wx/string.h>
#include <wx/checkbox.h>
#include <wx/gdicmn.h>
#include <wx/font.h>
#include <wx/colour.h>
#include <wx/settings.h>
#include <wx/tglbtn.h>
#include <wx/button.h>
#include <wx/sizer.h>
#include <wx/choicebk.h>
#include <wx/panel.h>
#include <wx/textctrl.h>
#include <wx/statbox.h>
#include <wx/dialog.h>

///////////////////////////////////////////////////////////////////////////


///////////////////////////////////////////////////////////////////////////////
/// Class RosoutPanelBase
///////////////////////////////////////////////////////////////////////////////
class RosoutPanelBase : public wxPanel 
{
	private:
	
	protected:
		wxCheckBox* enable_checkbox_;
		wxToggleButton* pause_toggle_;
		wxButton* clear_button_;
		wxButton* setup_button_;
		wxChoicebook* book_;
		
		// Virtual event handlers, overide them in your derived class
		virtual void onEnable( wxCommandEvent& event ){ event.Skip(); }
		virtual void onPauseToggled( wxCommandEvent& event ){ event.Skip(); }
		virtual void onClear( wxCommandEvent& event ){ event.Skip(); }
		virtual void onSetup( wxCommandEvent& event ){ event.Skip(); }
		
	
	public:
		RosoutPanelBase( wxWindow* parent, wxWindowID id = wxID_ANY, const wxPoint& pos = wxDefaultPosition, const wxSize& size = wxSize( 538,309 ), long style = wxTAB_TRAVERSAL );
		~RosoutPanelBase();
	
};

///////////////////////////////////////////////////////////////////////////////
/// Class RosoutSetupDialogBase
///////////////////////////////////////////////////////////////////////////////
class RosoutSetupDialogBase : public wxDialog 
{
	private:
	
	protected:
		wxTextCtrl* topic_;
		wxButton* topic_browse_button_;
		wxStdDialogButtonSizer* m_sdbSizer1;
		wxButton* m_sdbSizer1OK;
		wxButton* m_sdbSizer1Cancel;
		
		// Virtual event handlers, overide them in your derived class
		virtual void onTopicBrowse( wxCommandEvent& event ){ event.Skip(); }
		virtual void onCancel( wxCommandEvent& event ){ event.Skip(); }
		virtual void onOk( wxCommandEvent& event ){ event.Skip(); }
		
	
	public:
		RosoutSetupDialogBase( wxWindow* parent, wxWindowID id = wxID_ANY, const wxString& title = wxT("Rosout Panel Setup"), const wxPoint& pos = wxDefaultPosition, const wxSize& size = wxSize( 331,135 ), long style = wxDEFAULT_DIALOG_STYLE );
		~RosoutSetupDialogBase();
	
};

#endif //__rosout_generated__
