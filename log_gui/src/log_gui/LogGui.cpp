/*********************************************************************
* Software License Agreement (BSD License)
* 
*  Copyright (c) 2008, Willow Garage, Inc.
*  All rights reserved.
* 
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
* 
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the Willow Garage nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
* 
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/

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

  std::vector< LogRecorder<>* > bags;

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

  void dummyCb() {}

  virtual void startLogging( wxCommandEvent& event )
  {
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
    wxString dirName = logDir->GetValue() + wxT("/") + time.Format(wxT("%Y-%m-%d-%H-%M-%S"));

    if (wxMkDir(dirName.mb_str(wxConvUTF8), 0755) < 0) {
      wxMessageBox(wxT("Could not make directory: ") + dirName);
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

      std::string sanitizedName = *i;

      for (size_t j = 0; j < sanitizedName.length(); j++)
      {
        char c = sanitizedName[j]; // sanitize it a bit
        if (c == '\\' || c == '/' || c == '#' || c == '&' || c == ';')
          sanitizedName[j] = '_';
      }
      
      LogRecorder<>* bag = new LogRecorder<>;

      sanitizedName = std::string( dirName.mb_str(wxConvUTF8) ) + 
                      std::string("/") +
                      sanitizedName +
                      std::string(".bag");

      if (bag->open_log(sanitizedName,
                        node->map_name(*i),
                        start))
      {
        node->subscribe(*i, *bag, &LogGui::dummyCb, this);
        bags.push_back(bag);
      } else {
        delete bag;
      }
    }

    event.Skip();
  }

  virtual void stopLogging( wxCommandEvent& event )
  {
    for (std::vector<LogRecorder<>*>::iterator i = bags.begin(); i != bags.end(); i++)
    {
      node->unsubscribe(**i);
      delete *i;
    }
    bags.clear();

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


