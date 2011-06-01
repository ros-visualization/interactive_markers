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

#ifndef INTERACTIVE_MARKER_MENU_HANDLER
#define INTERACTIVE_MARKER_MENU_HANDLER

#include <visualization_msgs/Menu.h>
#include <interactive_markers/interactive_marker_server.h>

#include <boost/function.hpp>
#include <boost/unordered_map.hpp>

namespace interactive_markers
{

// Simple non-intrusive helper class which creates a menu and maps its
// entries to function callbacks
class MenuHandler
{
public:

  typedef visualization_msgs::InteractiveMarkerFeedbackConstPtr FeedbackConstPtr;
  typedef boost::function< void ( const FeedbackConstPtr& ) > FeedbackCallback;

  typedef unsigned int EntryHandle;

  enum CheckState {
    NO_CHECKBOX,
    CHECKED,
    UNCHECKED
  };

  MenuHandler( );

  // Insert top-level entry with feedback function
  EntryHandle insert( const std::string &title, const FeedbackCallback &feedback_cb );

  // Insert top-level entry with custom (client-side) command
  EntryHandle insert( const std::string &title, const std::string &command="",
      const std::string &command_type="" );

  // Insert second-level entry with feedback function
  EntryHandle insert( EntryHandle parent, const std::string &title, const FeedbackCallback &feedback_cb );

  // Insert second-level entry with custom (client-side) command
  EntryHandle insert( EntryHandle parent, const std::string &title, const std::string &command="",
      const std::string &command_type="" );

  // Specify if an entry should be visible or hidden
  void setVisible( EntryHandle handle, bool visible );

  // Specify if an entry is checked or can't be checked at all
  void setChecked( EntryHandle handle, CheckState check_state );

  // Copy current menu state into the marker given by the specified name &
  // divert callback for MENU_SELECT feedback to this manager
  bool apply( InteractiveMarkerServer &server, const std::string &marker_name );

private:

  struct EntryContext
  {
    std::string title;
    std::string command;
    std::string command_type;
    std::vector<EntryHandle> sub_entries;
    bool visible;
    CheckState check_state;
    FeedbackCallback feedback_cb;
  };

  // Call registered callback functions for given feedback command
  void processFeedback( const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback );

  visualization_msgs::MenuEntry makeEntry( EntryContext& context );

  // Insert without adding a top-level entry
  EntryHandle doInsert( const std::string &title, const FeedbackCallback &feedback_cb );
  EntryHandle doInsert( const std::string &title, const std::string &command,
      const std::string &command_type );

  std::vector<EntryHandle> top_level_handles_;

  boost::unordered_map<EntryHandle, EntryContext> entry_contexts_;

  EntryHandle current_handle_;

//  visualisation_msgs::Menu menu_;
};

}

#endif
