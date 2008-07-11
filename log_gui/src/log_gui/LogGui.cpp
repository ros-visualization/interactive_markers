#include <wx/app.h>
#include <wx/dir.h>

#include <wx/dirdlg.h>
#include <wx/msgdlg.h>

#include "ros/node.h"
#include "logging/LogRecorder.h"

#include "wx_topic_display/TopicDisplay.h"

#include "GenLogGui.h"

class LogGui : public GenLogGui
{
  TopicDisplay* topicDisplay;

  ros::node* node;

public:
  LogGui(wxWindow* parent) : GenLogGui(parent), node(NULL)
  {
    node = new ros::node("TestTopicDisplay");

    topicDisplay = new TopicDisplay(topicPanel,node);

    logDir->SetValue(wxGetCwd());
  }

  ~LogGui()
  {
    if (node)
      delete node;
  }

  virtual void onClose( wxCloseEvent& event )
  {
    printf("Shutting down ros...\n");
    ros::fini();
    event.Skip();
  }

  virtual void openDir( wxCommandEvent& event )
  {
    wxString dir = wxDirSelector( wxT("Choose a folder") );

    if (!dir.empty())
      logDir->SetValue(dir);
  }

  virtual void startLogging( wxCommandEvent& event )
  {
    wxDir dir;

    if (!wxDir::Exists(logDir->GetValue()))
    {
      int res = wxMessageBox(wxT("Directory: ") + 
                             logDir->GetValue() + 
                             wxT(" does not exist.  Create it now?"),
                             wxT(""),
                             wxOK | wxCANCEL);



      if (res == wxCANCEL)
      {
        event.Skip();
        return;
      }

      if (wxMkDir(logDir->GetValue().mb_str(wxConvUTF8), 0755) < 0) {
        wxMessageBox(wxT("Could not make directory: ") + logDir->GetValue());
        event.Skip();
        return;
      }

    }

    std::vector<std::string> selections = topicDisplay->getSelectedTopics();

    if (selections.size() == 0)
    {
      wxMessageBox(wxT("No topics selected."));
      event.Skip();
      return;
    }

    wxDateTime time = wxDateTime::Now();
    wxString timeStr = time.Format(wxT("%Y-%m-%d-%H-%M-%S"));

    if (wxMkDir(timeStr.mb_str(wxConvUTF8), 0755) < 0) {
      wxMessageBox(wxT("Could not make directory: ") + timeStr);
      event.Skip();
      return;
    }

    stopLogButton->Enable(true);
    startLogButton->Enable(false);
    statusBar->SetStatusText(wxT("Logging..."));

    ros::Time start = ros::Time::now();

    for (std::vector<std::string>::iterator i = selections.begin(); i != selections.end(); i++)
    {
      printf("vacuuming up [%s]\n", i->c_str());
      for (size_t j = 0; j < i->length(); j++)
      {
        char c = (*)[j]; // sanitize it a bit
        if (c == '\\' || c == '/' || c == '#' || c == '&' || c == ';')
          (*i)[j] = '_';
      }
      if (!bags[i].open_log(std::string(logdir) + std::string("/") + vac_topics[i] + string(".bag"),
                            map_name(vac_topics[i]),
                            start))
        throw std::runtime_error("couldn't open log file\n");
      subscribe(vac_topics[i], bags[i], &Vacuum::dummy_cb);
    }

    //    printf("Logging to: %s\n", (const char*)logDir->GetPath().mb_str(wxConvUTF8));


    {
      printf("%s is selected\n", i->c_str());
    }
    printf("\n");

    event.Skip();
  }

  virtual void stopLogging( wxCommandEvent& event )
  {
    startLogButton->Enable(true);
    stopLogButton->Enable(false);
    statusBar->SetStatusText(wxT(""));
  }

};

class LogApp : public wxApp
{
public:
  LogApp() {  };

  virtual ~LogApp() { };

  virtual bool OnInit()
  {
    char **argv = new char*[wxApp::argc]; //This memory duplicates argv, so is never deallocated
    for(int i = 0; i < wxApp::argc; i++){
      argv[i] = strdup((wxString(wxApp::argv[i]).mb_str(wxConvUTF8)));
    }
    ros::init(wxApp::argc, argv);

    LogGui* t = new LogGui( (wxWindow*)NULL );
    t->Show();
    SetTopWindow( t );
    return true;
  }
};

DECLARE_APP(LogApp);

IMPLEMENT_APP(LogApp);


