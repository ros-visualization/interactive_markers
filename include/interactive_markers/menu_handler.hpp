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

#include <visualization_msgs/MenuEntry.h>
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

  typedef uint32_t EntryHandle;

  typedef visualization_msgs::InteractiveMarkerFeedbackConstPtr FeedbackConstPtr;
  typedef boost::function< void ( const FeedbackConstPtr& ) > FeedbackCallback;

  enum CheckState {
    NO_CHECKBOX,
    CHECKED,
    UNCHECKED
  };

  MenuHandler( );

  /// Insert top-level entry with feedback function
  EntryHandle insert( const std::string &title, const FeedbackCallback &feedback_cb );

  /// Insert top-level entry with custom (client-side) command
  EntryHandle insert( const std::string &title,
                      const uint8_t command_type = visualization_msgs::MenuEntry::FEEDBACK,
                      const std::string &command="" );

  /// Insert second-level entry with feedback function
  EntryHandle insert( EntryHandle parent, const std::string &title,
      const FeedbackCallback &feedback_cb );

  /// Insert second-level entry with custom (client-side) command
  EntryHandle insert( EntryHandle parent, const std::string &title,
                      const uint8_t command_type = visualization_msgs::MenuEntry::FEEDBACK,
                      const std::string &command="" );

  /// Specify if an entry should be visible or hidden
  bool setVisible( EntryHandle handle, bool visible );

  /// Specify if an entry is checked or can't be checked at all
  bool setCheckState( EntryHandle handle, CheckState check_state );

  /// Get the current state of an entry
  /// @return true if the entry exists
  bool getCheckState( EntryHandle handle, CheckState &check_state ) const;

  /// Copy current menu state into the marker given by the specified name &
  /// divert callback for MENU_SELECT feedback to this manager
  bool apply( InteractiveMarkerServer &server, const std::string &marker_name );

  /// Re-apply to all markers that this was applied to previously
  bool reApply( InteractiveMarkerServer &server );

  /// Get the title for the given menu entry
  /// @return true if the entry exists
  bool getTitle( EntryHandle handle, std::string &title ) const;

private:

  struct EntryContext
  {
    std::string title;
    std::string command;
    uint8_t command_type;
    std::vector<EntryHandle> sub_entries;
    bool visible;
    CheckState check_state;
    FeedbackCallback feedback_cb;
  };

  // Call registered callback functions for given feedback command
  void processFeedback( const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback );

  // Create and push MenuEntry objects from handles_in onto
  // entries_out.  Calls itself recursively to add the entire menu
  // tree.
  bool pushMenuEntries( std::vector<EntryHandle>& handles_in,
                        std::vector<visualization_msgs::MenuEntry>& entries_out,
                        EntryHandle parent_handle );

  visualization_msgs::MenuEntry makeEntry( EntryContext& context, EntryHandle handle, EntryHandle parent_handle );

  // Insert without adding a top-level entry
  EntryHandle doInsert( const std::string &title,
                        const uint8_t command_type,
                        const std::string &command,
                        const FeedbackCallback &feedback_cb );

  std::vector<EntryHandle> top_level_handles_;

  boost::unordered_map<EntryHandle, EntryContext> entry_contexts_;

  EntryHandle current_handle_;

  std::set<std::string> managed_markers_;
};

}

#endif
