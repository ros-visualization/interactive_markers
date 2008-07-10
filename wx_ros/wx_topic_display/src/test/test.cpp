
#include <wx/app.h>

#include "ros/node.h"
#include "wx_topic_display/TopicDisplay.h"

#include "GenTestTopicDisplay.h"



class TestTopicDisplay : public GenTestTopicDisplay
{
  TopicDisplay* topicDisplay;

  ros::node* node;

public:
  TestTopicDisplay(wxWindow* parent) : GenTestTopicDisplay(parent), node(NULL)
  {
    int argc = 0;
    ros::init(argc, NULL);

    node = new ros::node("TestTopicDisplay");

    topicDisplay = new TopicDisplay(this,node);
  }

  ~TestTopicDisplay()
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
};

class TestApp : public wxApp
{
public:
  TestApp() {  };

  virtual ~TestApp() { };

  virtual bool OnInit()
  {
    TestTopicDisplay* t = new TestTopicDisplay( (wxWindow*)NULL );
    t->Show();
    SetTopWindow( t );
    return true;
  }
};

DECLARE_APP(TestApp);

IMPLEMENT_APP(TestApp);


