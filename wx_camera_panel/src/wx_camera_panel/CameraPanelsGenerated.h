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
#include <wx/stattext.h>
#include <wx/textctrl.h>
#include <wx/statbox.h>
#include <wx/spinctrl.h>
#include <wx/dialog.h>

///////////////////////////////////////////////////////////////////////////


///////////////////////////////////////////////////////////////////////////////
/// Class CameraPanelBase
///////////////////////////////////////////////////////////////////////////////
class CameraPanelBase : public wxPanel 
{
	private:
	
	protected:
		wxCheckBox* enable_;
		wxButton* setup_;
		wxPanel* image_panel_;
		
		// Virtual event handlers, overide them in your derived class
		virtual void onEnable( wxCommandEvent& event ){ event.Skip(); }
		virtual void onSetup( wxCommandEvent& event ){ event.Skip(); }
		virtual void onLeftMouseDown( wxMouseEvent& event ){ event.Skip(); }
		virtual void onLeftMouseUp( wxMouseEvent& event ){ event.Skip(); }
		virtual void onMiddleMouseDown( wxMouseEvent& event ){ event.Skip(); }
		virtual void onMiddleMouseUp( wxMouseEvent& event ){ event.Skip(); }
		virtual void onMouseMotion( wxMouseEvent& event ){ event.Skip(); }
		virtual void onMouseWheel( wxMouseEvent& event ){ event.Skip(); }
		virtual void onImagePaint( wxPaintEvent& event ){ event.Skip(); }
		virtual void onRightMouseDown( wxMouseEvent& event ){ event.Skip(); }
		virtual void onRightMouseUp( wxMouseEvent& event ){ event.Skip(); }
		virtual void onImageSize( wxSizeEvent& event ){ event.Skip(); }
		
	
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
		wxTextCtrl* camera_name_;
		wxCheckBox* enable_ptz_;
		wxStaticText* m_staticText4;
		wxStaticText* m_staticText5;
		wxSpinCtrl* pan_min_;
		wxStaticText* m_staticText6;
		wxSpinCtrl* pan_max_;
		wxStaticText* m_staticText411;
		wxStaticText* m_staticText511;
		wxSpinCtrl* tilt_min_;
		wxStaticText* m_staticText611;
		wxSpinCtrl* tilt_max_;
		wxStaticText* m_staticText41;
		wxStaticText* m_staticText51;
		wxSpinCtrl* zoom_min_;
		wxStaticText* m_staticText61;
		wxSpinCtrl* zoom_max_;
		wxButton* ok_;
		wxButton* cancel_;
		
		// Virtual event handlers, overide them in your derived class
		virtual void onOk( wxCommandEvent& event ){ event.Skip(); }
		virtual void onCancel( wxCommandEvent& event ){ event.Skip(); }
		
	
	public:
		CameraSetupDialogBase( wxWindow* parent, wxWindowID id = wxID_ANY, const wxString& title = wxT("Camera Setup"), const wxPoint& pos = wxDefaultPosition, const wxSize& size = wxSize( 472,333 ), long style = wxDEFAULT_DIALOG_STYLE );
		~CameraSetupDialogBase();
	
};

#endif //__CameraPanelsGenerated__
