#include "rosout_panel.h"
#include "rosout_setup_dialog.h"

#include <wx/wx.h>
#include <wx/aui/auibook.h>
#include <wx/richtext/richtextctrl.h>

#include <ros/node.h>

#include <sstream>

#include <boost/bind.hpp>

NodePage::~NodePage()
{
  V_LogPage::iterator it = log_pages_.begin();
  V_LogPage::iterator end = log_pages_.end();
  for ( ; it != end; ++it )
  {
    delete *it;
  }
  log_pages_.clear();
}

RosoutPanel::RosoutPanel( wxWindow* parent )
: RosoutPanelBase( parent )
, enabled_( false )
, topic_( "/rosout" )
{
  ros_node_ = ros::node::instance();

  /// @todo This should go away once creation of the ros::node is more well-defined
  if (!ros_node_)
  {
    int argc = 0;
    ros::init( argc, 0 );
    ros_node_ = new ros::node( "RosoutPanel" );
  }
  ROS_ASSERT( ros_node_ );

  createDefaultPages();

  process_timer_ = new wxTimer( this );
  process_timer_->Start( 100 );

  Connect( process_timer_->GetId(), wxEVT_TIMER, wxTimerEventHandler( RosoutPanel::onProcessTimer ), NULL, this );
}

RosoutPanel::~RosoutPanel()
{
  unsubscribe();

  Disconnect( process_timer_->GetId(), wxEVT_TIMER, wxTimerEventHandler( RosoutPanel::onProcessTimer ), NULL, this );

  delete process_timer_;

  clear();
}

void RosoutPanel::clear()
{
  // bug in DeleteAllPages causes segfault later on... fixed in later versions of wxWidgets
  while ( book_->GetPageCount() > 0 )
  {
    book_->DeletePage( 0 );
  }

  V_NodePage::iterator it = node_pages_.begin();
  V_NodePage::iterator end = node_pages_.end();
  for ( ; it != end; ++it )
  {
    delete *it;
  }
  node_pages_.clear();
  node_pages_by_name_.clear();
}

void RosoutPanel::setEnabled( bool enabled )
{
  if ( enabled_ == enabled )
  {
    return;
  }

  enabled_ = enabled;
  if ( enabled )
  {
    subscribe();
  }
  else
  {
    unsubscribe();
  }

  enable_checkbox_->SetValue( enabled );
}

void RosoutPanel::subscribe()
{
  if ( !enabled_ || topic_.empty() )
  {
    return;
  }

  ros_node_->subscribe( topic_, message_, &RosoutPanel::incomingMessage, this, 0 );
}

void RosoutPanel::unsubscribe()
{
  if ( topic_.empty() )
  {
    return;
  }

  ros_node_->unsubscribe( topic_ );
}

void RosoutPanel::forEachLogPage( boost::function<void (NodePage* node_page, LogPage* log_page)> f )
{
  V_NodePage::iterator node_it = node_pages_.begin();
  V_NodePage::iterator node_end = node_pages_.end();
  for ( ; node_it != node_end; ++node_it )
  {
    NodePage* node_page = *node_it;

    V_LogPage::iterator log_it = node_page->log_pages_.begin();
    V_LogPage::iterator log_end = node_page->log_pages_.end();
    for ( ; log_it != log_end; ++log_it )
    {
      LogPage* log_page = *log_it;

      f( node_page, log_page );
    }
  }
}

void RosoutPanel::setTopic( const std::string& topic )
{
  if ( topic == topic_ )
  {
    return;
  }

  unsubscribe();

  topic_ = topic;

  subscribe();
}

NodePage* RosoutPanel::createNodePage( const std::string& name )
{
  M_NameToNodePage::iterator it = node_pages_by_name_.find( name );
  if ( it != node_pages_by_name_.end() )
  {
    return it->second;
  }

  NodePage* page = new NodePage;
  page->name_ = name;
  page->notebook_ = new wxAuiNotebook( book_, wxID_ANY, wxDefaultPosition, wxDefaultSize, wxAUI_NB_SCROLL_BUTTONS|wxAUI_NB_TAB_MOVE|wxAUI_NB_TAB_SPLIT );
  book_->AddPage( page->notebook_, wxString::FromAscii( name.c_str() ) );

  node_pages_.push_back( page );
  node_pages_by_name_.insert( std::make_pair( name, page ) );

  return page;
}

LogPage* RosoutPanel::createLogPage( NodePage* node_page, uint32_t filter, const std::string& name )
{
  LogPage* page = new LogPage;
  page->filter_ = filter;
  page->text_control_ = new wxTextCtrl( node_page->notebook_, wxID_ANY, wxEmptyString, wxDefaultPosition, wxDefaultSize, wxTE_LEFT|wxTE_MULTILINE|wxTE_READONLY|wxTE_RICH );
  page->text_control_->SetBackgroundColour(*wxLIGHT_GREY);

  node_page->log_pages_.push_back( page );

  node_page->notebook_->AddPage( page->text_control_, wxString::FromAscii( name.c_str() ) );

  return page;
}

void RosoutPanel::createDefaultLogPages( NodePage* node_page )
{
  createLogPage( node_page, 0xffffffff, "All Levels" );
  createLogPage( node_page, rostools::Log::FATAL, "Fatal" );
  createLogPage( node_page, rostools::Log::ERROR, "Error" );
  createLogPage( node_page, rostools::Log::WARN, "Warning" );
  createLogPage( node_page, rostools::Log::DEBUG, "Debug" );
  createLogPage( node_page, rostools::Log::INFO, "Info" );
}

void RosoutPanel::createDefaultPages()
{
  NodePage* node_page = createNodePage( "All Nodes" );

  createDefaultLogPages( node_page );
}

void RosoutPanel::onProcessTimer( wxTimerEvent& evt )
{
  processMessages();
}

void RosoutPanel::onPauseToggled( wxCommandEvent& event )
{
  if ( event.IsChecked() )
  {
    process_timer_->Stop();
  }
  else
  {
    process_timer_->Start(100);
  }
}

void RosoutPanel::onClear( wxCommandEvent& event )
{
  clear();

  createDefaultPages();
}

void RosoutPanel::processMessage( const rostools::Log& message )
{
  std::stringstream ss;

  const wxColour* color;
  static const wxColour YELLOW( 255, 255, 0 );

  ss << "[" << message.name << "] ";

  ss << "[";
  switch (message.level)
  {
  case rostools::Log::FATAL:
    ss << "FATAL";
    color = wxRED;
    break;
  case rostools::Log::ERROR:
    ss << "ERROR";
    color = wxRED;
    break;
  case rostools::Log::WARN:
    ss << "WARN";
    color = &YELLOW;
    break;
  case rostools::Log::DEBUG:
    ss << "DEBUG";
    color = wxBLUE;
    break;
  case rostools::Log::INFO:
    ss << "INFO";
    color = wxBLACK;
    break;
  default:
    ss << "UNKNOWN (" << message.level << ")";
  }

  ss << "] " << message.msg << std::endl;

  // Make sure we have a node page for this node
  M_NameToNodePage::iterator it = node_pages_by_name_.find( message.name );
  if ( it == node_pages_by_name_.end() )
  {
    createDefaultLogPages( createNodePage( message.name ) );
  }

  V_NodePage::iterator node_it = node_pages_.begin();
  V_NodePage::iterator node_end = node_pages_.end();
  for ( ; node_it != node_end; ++node_it )
  {
    NodePage* node_page = *node_it;

    if ( node_it != node_pages_.begin() && node_page->name_ != message.name )
    {
      continue;
    }

    V_LogPage::iterator log_it = node_page->log_pages_.begin();
    V_LogPage::iterator log_end = node_page->log_pages_.end();
    for ( ; log_it != log_end; ++log_it )
    {
      LogPage* log_page = *log_it;

      if ( log_page->filter_ & message.level )
      {
        log_page->text_control_->SetDefaultStyle(wxTextAttr(*color));
        log_page->text_control_->AppendText( wxString::FromAscii( ss.str().c_str() ) );
      }
    }
  }
}

void RosoutPanel::processMessages()
{
  queue_mutex_.lock();

  V_Log local_queue;
  local_queue.swap( message_queue_ );

  queue_mutex_.unlock();

  V_Log::iterator it = local_queue.begin();
  V_Log::iterator end = local_queue.end();
  for ( ; it != end; ++it )
  {
    rostools::Log& message = *it;

    processMessage( message );
  }
}

void RosoutPanel::incomingMessage()
{
  queue_mutex_.lock();

  message_queue_.push_back( message_ );

  queue_mutex_.unlock();
}

void RosoutPanel::onEnable( wxCommandEvent& evt )
{
  setEnabled( evt.IsChecked() );
}

void RosoutPanel::onSetup( wxCommandEvent& evt )
{
  RosoutSetupDialog dialog( this, ros_node_, topic_ );

  if ( dialog.ShowModal() == wxOK )
  {
    setTopic( dialog.getTopic() );
  }
}
