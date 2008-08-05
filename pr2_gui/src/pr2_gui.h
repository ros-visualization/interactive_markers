///////////////////////////////////////////////////////////////////////////
// C++ code generated with wxFormBuilder (version Apr 21 2008)
// http://www.wxformbuilder.org/
//
// PLEASE DO "NOT" EDIT THIS FILE!
///////////////////////////////////////////////////////////////////////////

#ifndef __pr2_gui__
#define __pr2_gui__

#include <wx/string.h>
#include <wx/checkbox.h>
#include <wx/gdicmn.h>
#include <wx/font.h>
#include <wx/colour.h>
#include <wx/settings.h>
#include <wx/sizer.h>
#include <wx/statbox.h>
#include <wx/radiobox.h>
#include <wx/slider.h>
#include <wx/panel.h>
#include <wx/button.h>
#include <wx/textctrl.h>
#include <wx/frame.h>

///////////////////////////////////////////////////////////////////////////


///////////////////////////////////////////////////////////////////////////////
/// Class launcher
///////////////////////////////////////////////////////////////////////////////
class launcher : public wxFrame 
{
	private:
	
	protected:
		wxFlexGridSizer* Window_FGS;
		wxFlexGridSizer* LeftDock_FGS;
		wxStaticBoxSizer* Visualization_SBS;
		wxFlexGridSizer* fgSizer12;
		wxCheckBox* HeadLaser_CB;
		wxCheckBox* FloorLaser_CB;
		wxCheckBox* Stereo_CB;
		wxCheckBox* Model_CB;
		wxCheckBox* UCS_CB;
		wxCheckBox* Grid_CB;
		wxCheckBox* Objects_CB;
		wxRadioBox* Views_RB;
		wxRadioBox* HeadLaser_RB;
		wxStaticBoxSizer* PTZL_SBS;
		wxSlider* panPTZL_S;
		wxSlider* zoomPTZL_S;
		wxPanel* PTZL_B;
		wxSlider* tiltPTZL_S;
		wxStaticBoxSizer* WristL_SBS;
		wxPanel* WristL_B;
		wxCheckBox* Visualization_CB;
		wxCheckBox* Topdown_CB;
		wxCheckBox* PTZL_CB;
		wxCheckBox* PTZR_CB;
		wxCheckBox* WristL_CB;
		wxCheckBox* WristR_CB;
		wxButton* EmStop_B;
		wxTextCtrl* Console_TC;
		wxTextCtrl* Ros_TC;
		wxFlexGridSizer* RightDock_FGS;
		wxStaticBoxSizer* Topdown_SBS;
		wxButton* PLACEHOLDER_B;
		wxStaticBoxSizer* PTZR_SBS;
		wxSlider* panPTZR_S;
		wxSlider* zoomPTZR_S;
		wxPanel* PTZR_B;
		wxSlider* tiltPTZR_S;
		wxStaticBoxSizer* WristR_SBS;
		wxPanel* WristR_B;
		
		// Virtual event handlers, overide them in your derived class
		virtual void startStopHeadPtCld( wxCommandEvent& event ){ event.Skip(); }
		virtual void startStopFloorPtCld( wxCommandEvent& event ){ event.Skip(); }
		virtual void startStopStereoPtCld( wxCommandEvent& event ){ event.Skip(); }
		virtual void startStopModel( wxCommandEvent& event ){ event.Skip(); }
		virtual void startStopUCS( wxCommandEvent& event ){ event.Skip(); }
		virtual void startStopGrid( wxCommandEvent& event ){ event.Skip(); }
		virtual void startStopObjects( wxCommandEvent& event ){ event.Skip(); }
		virtual void deleteObjects( wxMouseEvent& event ){ event.Skip(); }
		virtual void viewChanged( wxCommandEvent& event ){ event.Skip(); }
		virtual void HeadLaserChanged( wxCommandEvent& event ){ event.Skip(); }
		virtual void PTZL_ptzChanged( wxScrollEvent& event ){ event.Skip(); }
		virtual void PTZL_click( wxMouseEvent& event ){ event.Skip(); }
		virtual void startStop_Visualization( wxCommandEvent& event ){ event.Skip(); }
		virtual void startStopTopdown( wxCommandEvent& event ){ event.Skip(); }
		virtual void startStop_PTZL( wxCommandEvent& event ){ event.Skip(); }
		virtual void startStop_PTZR( wxCommandEvent& event ){ event.Skip(); }
		virtual void startStop_WristL( wxCommandEvent& event ){ event.Skip(); }
		virtual void startStop_WristR( wxCommandEvent& event ){ event.Skip(); }
		virtual void EmergencyStop( wxCommandEvent& event ){ event.Skip(); }
		virtual void PTZR_ptzChanged( wxScrollEvent& event ){ event.Skip(); }
		virtual void PTZR_click( wxMouseEvent& event ){ event.Skip(); }
		
	
	public:
		launcher( wxWindow* parent, wxWindowID id = wxID_ANY, const wxString& title = wxT("Launcher"), const wxPoint& pos = wxDefaultPosition, const wxSize& size = wxSize( 1474,523 ), long style = wxDEFAULT_FRAME_STYLE|wxTAB_TRAVERSAL, const wxString& name = wxT("launcher") );
		~launcher();
	
};

#endif //__pr2_gui__
