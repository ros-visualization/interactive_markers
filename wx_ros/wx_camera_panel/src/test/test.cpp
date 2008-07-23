#include <wx/wx.h>

#include "../wx_camera_panel/CameraPanel.h"

#include "ros/node.h"

class MyFrame : public wxFrame {
public:
  MyFrame(wxWindow* parent) : wxFrame(parent, -1, _("Camera Panels Test App"),
				      wxDefaultPosition, wxSize(800,600),
				      wxDEFAULT_FRAME_STYLE)                              
  {
    CameraPanel* cameraPanel = new CameraPanel( this );

	cameraPanel->SetSize( this->GetSize() );
	cameraPanel->SetImageSubscription( "/PTZR_image" );
	cameraPanel->SetPTZStateSubscription( "/PTZR_state" );
	cameraPanel->SetPTZControlCommand( "/PTZR_cmd" );
	cameraPanel->SetEnabled( true );
  }
  
  ~MyFrame()
  {
  }
};

// our normal wxApp-derived class, as usual
class MyApp : public wxApp {
public:
  
  bool OnInit()
  {
	ros::init(argc, (char**)argv);

    wxFrame* frame = new MyFrame(NULL);
    SetTopWindow(frame);
    frame->Show();
    return true;                    
  }

  int OnExit()
  {
	  return 0;
  }
};

DECLARE_APP(MyApp);
IMPLEMENT_APP(MyApp);
