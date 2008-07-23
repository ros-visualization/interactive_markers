///////////////////////////////////////////////////////////////////////////
// C++ code generated with wxFormBuilder (version Apr 21 2008)
// http://www.wxformbuilder.org/
//
// PLEASE DO "NOT" EDIT THIS FILE!
///////////////////////////////////////////////////////////////////////////

#ifndef __CameraPanelsGenerated__
#define __CameraPanelsGenerated__

#include <wx/string.h>
#include <wx/checkbox.h>
#include <wx/gdicmn.h>
#include <wx/font.h>
#include <wx/colour.h>
#include <wx/settings.h>
#include <wx/button.h>
#include <wx/sizer.h>
#include <wx/panel.h>
#include <wx/statbox.h>
#include <wx/stattext.h>
#include <wx/textctrl.h>
#include <wx/dialog.h>

///////////////////////////////////////////////////////////////////////////


///////////////////////////////////////////////////////////////////////////////
/// Class CameraPanelBase
///////////////////////////////////////////////////////////////////////////////
class CameraPanelBase : public wxPanel 
{
	private:
	
	protected:
		wxCheckBox* m_Enable;
		wxButton* m_Setup;
		wxPanel* m_ImagePanel;
		
		// Virtual event handlers, overide them in your derived class
		virtual void OnEnable( wxCommandEvent& event ){ event.Skip(); }
		virtual void OnSetup( wxCommandEvent& event ){ event.Skip(); }
		virtual void OnLeftMouseDown( wxMouseEvent& event ){ event.Skip(); }
		virtual void OnLeftMouseUp( wxMouseEvent& event ){ event.Skip(); }
		virtual void OnMiddleMouseDown( wxMouseEvent& event ){ event.Skip(); }
		virtual void OnMiddleMouseUp( wxMouseEvent& event ){ event.Skip(); }
		virtual void OnMouseMotion( wxMouseEvent& event ){ event.Skip(); }
		virtual void OnMouseWheel( wxMouseEvent& event ){ event.Skip(); }
		virtual void OnImagePaint( wxPaintEvent& event ){ event.Skip(); }
		virtual void OnRightMouseDown( wxMouseEvent& event ){ event.Skip(); }
		virtual void OnRightMouseUp( wxMouseEvent& event ){ event.Skip(); }
		virtual void OnImageSize( wxSizeEvent& event ){ event.Skip(); }
		
	
	public:
		CameraPanelBase( wxWindow* parent, wxWindowID id = wxID_ANY, const wxPoint& pos = wxDefaultPosition, const wxSize& size = wxSize( 487,374 ), long style = wxTAB_TRAVERSAL );
		~CameraPanelBase();
	
};

///////////////////////////////////////////////////////////////////////////////
/// Class CameraSetupDialogBase
///////////////////////////////////////////////////////////////////////////////
class CameraSetupDialogBase : public wxDialog 
{
	private:
	
	protected:
		wxStaticText* m_staticText711;
		wxTextCtrl* m_ImageSubscriptionText;
		wxButton* m_ImageSubscriptionBrowse;
		wxStaticText* m_staticText71;
		wxTextCtrl* m_PTZStateSubscriptionText;
		wxButton* m_PTZStateSubscriptionBrowse;
		wxStaticText* m_staticText712;
		wxTextCtrl* m_PTZControlCommandText;
		wxButton* m_Ok;
		wxButton* m_Cancel;
		
		// Virtual event handlers, overide them in your derived class
		virtual void OnImageSubscriptionBrowse( wxCommandEvent& event ){ event.Skip(); }
		virtual void OnPTZStateSubscriptionBrowse( wxCommandEvent& event ){ event.Skip(); }
		virtual void OnOk( wxCommandEvent& event ){ event.Skip(); }
		virtual void OnCancel( wxCommandEvent& event ){ event.Skip(); }
		
	
	public:
		CameraSetupDialogBase( wxWindow* parent, wxWindowID id = wxID_ANY, const wxString& title = wxT("Camera Setup"), const wxPoint& pos = wxDefaultPosition, const wxSize& size = wxSize( 472,231 ), long style = wxDEFAULT_DIALOG_STYLE );
		~CameraSetupDialogBase();
	
};

#endif //__CameraPanelsGenerated__
