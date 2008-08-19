///////////////////////////////////////////////////////////////////////////
// C++ code generated with wxFormBuilder (version Apr 21 2008)
// http://www.wxformbuilder.org/
//
// PLEASE DO "NOT" EDIT THIS FILE!
///////////////////////////////////////////////////////////////////////////

#ifndef __roserrGenerated__
#define __roserrGenerated__

#include <wx/string.h>
#include <wx/checkbox.h>
#include <wx/gdicmn.h>
#include <wx/font.h>
#include <wx/colour.h>
#include <wx/settings.h>
#include <wx/fontpicker.h>
#include <wx/sizer.h>
#include <wx/button.h>
#include <wx/textctrl.h>
#include <wx/panel.h>
#include <wx/statbox.h>
#include <wx/stattext.h>
#include <wx/dialog.h>

///////////////////////////////////////////////////////////////////////////


///////////////////////////////////////////////////////////////////////////////
/// Class RoserrPanelBase
///////////////////////////////////////////////////////////////////////////////
class RoserrPanelBase : public wxPanel 
{
	private:
	
	protected:
		wxCheckBox* m_EnableCB;
		wxFontPickerCtrl* m_FontFP;
		wxButton* m_Setup;
		wxTextCtrl* m_roserrTC;
		
		// Virtual event handlers, overide them in your derived class
		virtual void OnEnable( wxCommandEvent& event ){ event.Skip(); }
		virtual void OnFontChange( wxFontPickerEvent& event ){ event.Skip(); }
		virtual void OnSetup( wxCommandEvent& event ){ event.Skip(); }
		
	
	public:
		RoserrPanelBase( wxWindow* parent, wxWindowID id = wxID_ANY, const wxPoint& pos = wxDefaultPosition, const wxSize& size = wxSize( 273,138 ), long style = wxTAB_TRAVERSAL );
		~RoserrPanelBase();
	
};

///////////////////////////////////////////////////////////////////////////////
/// Class RoserrSetupDialogBase
///////////////////////////////////////////////////////////////////////////////
class RoserrSetupDialogBase : public wxDialog 
{
	private:
	
	protected:
		wxCheckBox* m_MonochromeCB;
		wxStaticText* m_staticText1;
		wxTextCtrl* m_rostopicL;
		wxButton* m_roserrB;
		wxStdDialogButtonSizer* m_sdbSizer1;
		wxButton* m_sdbSizer1OK;
		wxButton* m_sdbSizer1Cancel;
		
		// Virtual event handlers, overide them in your derived class
		virtual void OnClick( wxCommandEvent& event ){ event.Skip(); }
		virtual void OnCancel( wxCommandEvent& event ){ event.Skip(); }
		virtual void OnOk( wxCommandEvent& event ){ event.Skip(); }
		
	
	public:
		RoserrSetupDialogBase( wxWindow* parent, wxWindowID id = wxID_ANY, const wxString& title = wxT("Roserr Setup"), const wxPoint& pos = wxDefaultPosition, const wxSize& size = wxSize( 333,207 ), long style = wxDEFAULT_DIALOG_STYLE );
		~RoserrSetupDialogBase();
	
};

#endif //__roserrGenerated__
