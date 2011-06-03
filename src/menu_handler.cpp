/*
 * Copyright (c) 2011, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 * 
 * Author: David Gossow
 */

#include "interactive_markers/menu_handler.h"

#include <boost/bind.hpp>
#include <boost/make_shared.hpp>

namespace interactive_markers
{

MenuHandler::MenuHandler() :
    current_handle_(1)
{

}

MenuHandler::EntryHandle MenuHandler::insert( const std::string &title,
    const FeedbackCallback &feedback_cb )
{
  EntryHandle handle = doInsert( title, feedback_cb );
  top_level_handles_.push_back( handle );
  return handle;
}

MenuHandler::EntryHandle MenuHandler::insert( const std::string &title,
    const std::string &command, const std::string &command_type )
{
  EntryHandle handle = doInsert( title, command, command_type );
  top_level_handles_.push_back( handle );
  return handle;
}


MenuHandler::EntryHandle MenuHandler::insert( EntryHandle parent, const std::string &title,
    const FeedbackCallback &feedback_cb )
{
  boost::unordered_map<EntryHandle, EntryContext>::iterator parent_context =
      entry_contexts_.find( parent );

  if ( parent_context == entry_contexts_.end() )
  {
    return 0;
  }

  EntryHandle handle = doInsert( title, feedback_cb );
  parent_context->second.sub_entries.push_back( handle );
  return handle;
}


MenuHandler::EntryHandle MenuHandler::insert( EntryHandle parent, const std::string &title,
    const std::string &command, const std::string &command_type )
{
  boost::unordered_map<EntryHandle, EntryContext>::iterator parent_context =
      entry_contexts_.find( parent );

  if ( parent_context == entry_contexts_.end() )
  {
    return 0;
  }

  EntryHandle handle = doInsert( title, command, command_type );
  parent_context->second.sub_entries.push_back( handle );
  return handle;
}


void MenuHandler::setVisible( EntryHandle handle, bool visible )
{
  boost::unordered_map<EntryHandle, EntryContext>::iterator context =
      entry_contexts_.find( handle );

  if ( context == entry_contexts_.end() )
  {
    return;
  }

  context->second.visible = visible;
}


void MenuHandler::setCheckState( EntryHandle handle, CheckState check_state )
{
  boost::unordered_map<EntryHandle, EntryContext>::iterator context =
      entry_contexts_.find( handle );

  if ( context == entry_contexts_.end() )
  {
    return;
  }

  context->second.check_state = check_state;
}


bool MenuHandler::getCheckState( EntryHandle handle, CheckState &check_state )
{
  boost::unordered_map<EntryHandle, EntryContext>::iterator context =
      entry_contexts_.find( handle );

  if ( context == entry_contexts_.end() )
  {
    check_state = NO_CHECKBOX;
    return false;
  }

  check_state = context->second.check_state;
  return true;
}


bool MenuHandler::apply( InteractiveMarkerServer &server, const std::string &marker_name )
{
  visualization_msgs::InteractiveMarker int_marker;

  if ( !server.get( marker_name, int_marker ) )
  {
    // forget marker name.
    ROS_WARN_STREAM( "Interactive marker '" << marker_name << "' does not exist." );
    managed_markers_.erase( marker_name );
    return false;
  }

  int_marker.menu.clear();

  for ( unsigned t=0; t<top_level_handles_.size(); t++ )
  {
    EntryHandle top_level_handle = top_level_handles_[t];
    boost::unordered_map<EntryHandle, EntryContext>::iterator top_level_context_it =
        entry_contexts_.find( top_level_handle );

    if ( top_level_context_it == entry_contexts_.end() )
    {
      ROS_ERROR( "Internal error: context handle not found! This is a bug in MenuHandler." );
      return false;
    }

    if ( !top_level_context_it->second.visible )
    {
      continue;
    }

    visualization_msgs::Menu menu;
    menu.entry = makeEntry( top_level_context_it->second );

    for ( unsigned s=0; s<top_level_context_it->second.sub_entries.size(); s++ )
    {
      EntryHandle second_level_handle = top_level_context_it->second.sub_entries[s];
      boost::unordered_map<EntryHandle, EntryContext>::iterator second_level_context_it =
          entry_contexts_.find( second_level_handle );

      if ( second_level_context_it == entry_contexts_.end() )
      {
        ROS_ERROR( "Internal error: context handle not found! This is a bug in MenuHandler." );
        return false;
      }

      if ( !second_level_context_it->second.visible )
      {
        continue;
      }

      menu.sub_entries.push_back( makeEntry( second_level_context_it->second ) );
    }

    int_marker.menu.push_back( menu );
  }

  server.insert( int_marker );
  server.setCallback( marker_name, boost::bind( &MenuHandler::processFeedback, this, _1 ), visualization_msgs::InteractiveMarkerFeedback::MENU_SELECT );
  managed_markers_.insert( marker_name );
  return true;
}

bool MenuHandler::reApply( InteractiveMarkerServer &server )
{
  bool success = true;
  std::set<std::string>::iterator it = managed_markers_.begin();
  for ( ; it!=managed_markers_.end(); it++ )
  {
    success = success && apply( server, *it );
  }
  return success;
}


MenuHandler::EntryHandle MenuHandler::doInsert( const std::string &title,
    const FeedbackCallback &feedback_cb )
{
  EntryHandle handle = current_handle_;
  current_handle_++;

  std::ostringstream s;
  s << "menu_handler_cmd_" << handle;

  EntryContext context;
  context.title = title;
  context.command = s.str();
  context.visible = true;
  context.check_state = NO_CHECKBOX;
  context.feedback_cb = feedback_cb;

  entry_contexts_[handle] = context;
  return handle;
}


MenuHandler::EntryHandle MenuHandler::doInsert( const std::string &title,
    const std::string &command="", const std::string &command_type="" )
{
  EntryHandle handle = current_handle_;
  current_handle_++;

  EntryContext context;
  context.title = title;
  context.command = command;
  context.command_type = command_type;
  context.visible = true;
  context.check_state = NO_CHECKBOX;

  entry_contexts_[handle] = context;
  return handle;
}


visualization_msgs::MenuEntry MenuHandler::makeEntry( EntryContext& context )
{
  visualization_msgs::MenuEntry menu_entry;

  switch ( context.check_state )
  {
    case NO_CHECKBOX:
      menu_entry.title = context.title;
      break;
    case CHECKED:
      menu_entry.title = "[x] "+context.title;
      break;
    case UNCHECKED:
      menu_entry.title = "[ ] "+context.title;
      break;
  }

  menu_entry.command = context.command;
  menu_entry.command_type = context.command_type;

  return menu_entry;
}


void MenuHandler::processFeedback( const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback )
{
  EntryHandle handle;
  if ( getHandle(feedback->command, handle) )
  {
    boost::unordered_map<EntryHandle, EntryContext>::iterator context =
        entry_contexts_.find( handle );

    if ( context != entry_contexts_.end() && context->second.feedback_cb )
    {
      context->second.feedback_cb( feedback );
    }
  }
}

bool MenuHandler::getHandle( const std::string &command, EntryHandle &handle )
{
  if ( command.find( "menu_handler_cmd_") != 0 )
  {
    return false;
  }

  std::string id = command.substr( 17 );
  handle = atoi( id.c_str() );

  // check if the handle exists
  if ( entry_contexts_.find( handle ) == entry_contexts_.end() )
  {
    return false;
  }

  return true;
}

}
