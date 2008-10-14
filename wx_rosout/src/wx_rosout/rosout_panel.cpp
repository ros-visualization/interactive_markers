#include "rosout_panel.h"
#include "rosout_setup_dialog.h"
#include "rosout_list_control.h"

#include <wx/wx.h>
#include <wx/aui/auibook.h>
#include <wx/richtext/richtextctrl.h>

#include <ros/node.h>

#include <sstream>
#include <algorithm>

#include <boost/bind.hpp>

namespace wx_rosout
{

RosoutPanel::RosoutPanel( wxWindow* parent )
: RosoutPanelBase( parent )
, enabled_( false )
, topic_( "/rosout" )
, message_id_counter_( 0 )
, max_messages_( 20000 )
{
  ros_node_ = ros::node::instance();

  /// @todo This should go away once creation of the ros::node is more well-defined
  if (!ros_node_)
  {
    int argc = 0;
    ros::init( argc, 0 );
    ros_node_ = new ros::node( "RosoutPanel", ros::node::DONT_HANDLE_SIGINT );
  }
  ROS_ASSERT( ros_node_ );

  process_timer_ = new wxTimer( this );
  process_timer_->Start( 100 );

  Connect( process_timer_->GetId(), wxEVT_TIMER, wxTimerEventHandler( RosoutPanel::onProcessTimer ), NULL, this );

  table_->setMessageFunction( boost::bind( &RosoutPanel::getMessageByIndex, this, _1 ) );
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
  messages_.clear();
  ordered_messages_.clear();
  table_->SetItemCount( 0 );
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

  ros_node_->unsubscribe( topic_, &RosoutPanel::incomingMessage, this );
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

void RosoutPanel::onProcessTimer( wxTimerEvent& evt )
{
  processMessages();
}

void RosoutPanel::onClear( wxCommandEvent& event )
{
  clear();
}

void RosoutPanel::addMessageToTable( const rostools::Log& message, uint32_t id )
{
  ordered_messages_.push_back( id );
  table_->SetItemCount( ordered_messages_.size() );
}

const rostools::Log& RosoutPanel::getMessageByIndex( uint32_t index ) const
{
  ROS_ASSERT( index < ordered_messages_.size() );

  M_IdToMessage::const_iterator it = messages_.find( ordered_messages_[ index ] );
  ROS_ASSERT( it != messages_.end() );

  return it->second;
}

bool RosoutPanel::filter( const std::string& str ) const
{
  return str.find( filter_ ) != std::string::npos;
}

bool RosoutPanel::filter( const V_string& strs ) const
{
  V_string::const_iterator it = strs.begin();
  V_string::const_iterator end = strs.end();
  for ( ; it != end; ++it )
  {
    if ( filter( *it ) )
    {
      return true;
    }
  }

  return false;
}

bool RosoutPanel::filter( uint32_t id ) const
{
  if ( filter_.empty() )
  {
    return true;
  }

  M_IdToMessage::const_iterator it = messages_.find( id );
  ROS_ASSERT( it != messages_.end() );

  const rostools::Log& message = it->second;

  std::stringstream line;
  line << message.line;
  std::stringstream time;
  time << message.header.stamp;

  return filter( message.name )
      || filter( message.msg )
      || filter( message.file )
      || filter( message.function )
      || filter( line.str() )
      || filter( message.topics )
      || filter( time.str() )
      || filter( (const char*)table_->getSeverityText( message ).fn_str() );
}

void RosoutPanel::refilter( const std::string& old_filter )
{
  table_->Freeze();

  long item_count = table_->GetItemCount();
  bool select_last_item = false;
  if ( item_count == 0 || table_->GetItemState( item_count - 1, wxLIST_STATE_FOCUSED ) & wxLIST_STATE_FOCUSED )
  {
    select_last_item = true;
  }

  if ( filter_.substr( 0, old_filter.size() ) == old_filter )
  {
    V_u32 new_list;
    new_list.reserve( ordered_messages_.size() );
    V_u32::iterator it = ordered_messages_.begin();
    V_u32::iterator end = ordered_messages_.end();
    for ( ; it != end; ++it )
    {
      if ( filter( *it ) )
      {
        new_list.push_back( *it );
      }
    }

    new_list.swap( ordered_messages_ );

    table_->SetItemCount( ordered_messages_.size() );
  }
  else
  {
    ordered_messages_.clear();
    M_IdToMessage::iterator it = messages_.begin();
    M_IdToMessage::iterator end = messages_.end();
    for ( ; it != end; ++it )
    {
      uint32_t id = it->first;
      rostools::Log& message = it->second;

      if ( filter( id ) )
      {
        addMessageToTable( message, id );
      }
    }
  }

  if ( select_last_item )
  {
    item_count = table_->GetItemCount();
    table_->SetItemState( item_count - 1, wxLIST_STATE_FOCUSED, wxLIST_STATE_FOCUSED );
    table_->EnsureVisible( item_count - 1 );
  }

  // This for some reason prevents the control from flickering: http://wiki.wxwidgets.org/Flicker-Free_Drawing#No-flickering_for_wxListCtrl_with_wxLC_REPORT_.7C_wxLC_VIRTUAL_style
  wxIdleEvent idle;
  wxTheApp->SendIdleEvents(this, idle);

  table_->Thaw();
}

void RosoutPanel::popMessage()
{
  M_IdToMessage::iterator it = messages_.begin();
  if ( !ordered_messages_.empty() && ordered_messages_.front() == it->first )
  {

    ordered_messages_.erase( ordered_messages_.begin() );
    table_->SetItemCount( ordered_messages_.size() );

#if 0
    int32_t selection = table_->getSelection();
    if ( selection != -1 )
    {
      table_->SetItemState( selection, wxLIST_STATE_SELECTED, 0 );

      if ( selection > 0 )
      {
        table_->SetItemState( selection - 1, wxLIST_STATE_SELECTED, wxLIST_STATE_SELECTED );
        table_->EnsureVisible( selection - 1 );
      }

      // This for some reason prevents the control from flickering: http://wiki.wxwidgets.org/Flicker-Free_Drawing#No-flickering_for_wxListCtrl_with_wxLC_REPORT_.7C_wxLC_VIRTUAL_style
      wxIdleEvent idle;
      wxTheApp->SendIdleEvents(this, idle);
    }
#endif
  }

  messages_.erase( it );
}

void RosoutPanel::processMessage( const rostools::Log& message )
{
  uint32_t id = message_id_counter_++;

  messages_.insert( std::make_pair( id, message ) );

  if ( filter( id ) )
  {
    addMessageToTable( message, id );
  }

  if ( messages_.size() > max_messages_ )
  {
    popMessage();
  }
}

void RosoutPanel::processMessages()
{
  queue_mutex_.lock();

  V_Log local_queue;
  local_queue.swap( message_queue_ );

  queue_mutex_.unlock();

  if ( local_queue.empty() )
  {
    return;
  }

  table_->Freeze();

  long item_count = table_->GetItemCount();
  bool select_last_item = false;
  if ( item_count == 0 || table_->GetItemState( item_count - 1, wxLIST_STATE_FOCUSED ) & wxLIST_STATE_FOCUSED )
  {
    select_last_item = true;
    table_->SetItemState( item_count - 1, wxLIST_STATE_FOCUSED, 0 );
  }

  V_Log::iterator it = local_queue.begin();
  V_Log::iterator end = local_queue.end();
  for ( ; it != end; ++it )
  {
    rostools::Log& message = *it;

    processMessage( message );
  }

  if ( select_last_item )
  {
    item_count = table_->GetItemCount();
    table_->SetItemState( item_count - 1, wxLIST_STATE_FOCUSED, wxLIST_STATE_FOCUSED );
    table_->EnsureVisible( item_count - 1 );
  }

  // This for some reason prevents the control from flickering: http://wiki.wxwidgets.org/Flicker-Free_Drawing#No-flickering_for_wxListCtrl_with_wxLC_REPORT_.7C_wxLC_VIRTUAL_style
  wxIdleEvent idle;
  wxTheApp->SendIdleEvents(this, idle);

  table_->Thaw();
}

void RosoutPanel::incomingMessage()
{
  queue_mutex_.lock();

  message_queue_.push_back( message_ );

  queue_mutex_.unlock();
}

void RosoutPanel::onPause( wxCommandEvent& evt )
{
  if ( evt.IsChecked() )
  {
    process_timer_->Stop();
  }
  else
  {
    process_timer_->Start( 100 );
  }
}

void RosoutPanel::onSetup( wxCommandEvent& evt )
{
  RosoutSetupDialog dialog( this, ros_node_, topic_, max_messages_ );

  if ( dialog.ShowModal() == wxOK )
  {
    setTopic( dialog.getTopic() );
    setBufferSize( dialog.getBufferSize() );
  }
}

void RosoutPanel::setBufferSize( uint32_t size )
{
  max_messages_ = size;
  while ( messages_.size() >= max_messages_ )
  {
    popMessage();
  }
}

void RosoutPanel::onFilterText( wxCommandEvent& event )
{
  std::string old_filter = filter_;
  filter_ = filter_text_->GetValue().fn_str();

  refilter( old_filter );
}

} // namespace wx_rosout
