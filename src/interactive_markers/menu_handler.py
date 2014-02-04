#!/usr/bin/env python

# Copyright (c) 2011, Willow Garage, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#   * Redistributions of source code must retain the above copyright
#     notice, this list of conditions and the following disclaimer.
#   * Redistributions in binary form must reproduce the above copyright
#     notice, this list of conditions and the following disclaimer in the
#     documentation and/or other materials provided with the distribution.
#   * Neither the name of the Willow Garage, Inc. nor the names of its
#     contributors may be used to endorse or promote products derived from
#     this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

# Author: Michael Ferguson

import rospy

from visualization_msgs.msg import InteractiveMarkerFeedback
from visualization_msgs.msg import MenuEntry


class EntryContext:
    def __init__(self):
        self.title = ""
        self.command = ""
        self.command_type = 0
        self.sub_entries = list()
        self.visible = True
        self.check_state = 0
        self.feedback_cb = None


## @brief Simple non-intrusive helper class which creates a menu and maps its
## entries to function callbacks
class MenuHandler:
    NO_CHECKBOX = 0
    CHECKED = 1
    UNCHECKED = 2

    def __init__(self):
        self.top_level_handles_ = list()    # std::vector<EntryHandle>
        self.entry_contexts_ = dict()       # boost::unordered_map<EntryHandle, EntryContext>
        self.current_handle_ = 1
        self.managed_markers_ = set()       # std::set<std::string>

    ## Insert a new menu item
    def insert(self, title, parent=None, command_type=MenuEntry.FEEDBACK, command="", callback=None):
        handle = self.doInsert(title, command_type, command, callback)
        if parent is not None:
            try:
                parent_context = self.entry_contexts_[parent]
                parent_context.sub_entries.append(handle)
            except:
                rospy.logerr("Parent menu entry " + str(parent) + " not found.")
                return None
        else:
            self.top_level_handles_.append(handle)
        return handle

    ## Specify if an entry should be visible or hidden
    def setVisible(self, handle, visible):
        try:
            context = self.entry_contexts_[handle]
            context.visible = visible
            return True
        except:
            return False

    ## Specify if an entry is checked or can't be checked at all
    def setCheckState(self, handle, check_state):
        try:
            context = self.entry_contexts_[handle]
            context.check_state = check_state
            return True
        except:
            return False

    ## Get the current state of an entry
    ## @return CheckState if the entry exists and has checkbox, None otherwise
    def getCheckState(self, handle):
        try:
            context = self.entry_contexts_[handle]
            return context.check_state
        except:
            return None

    ## Copy current menu state into the marker given by the specified name &
    ## divert callback for MENU_SELECT feedback to this manager
    def apply(self, server, marker_name):
        marker = server.get(marker_name)
        if not marker:
            self.managed_markers_.remove(marker_name)
            return False

        marker.menu_entries = list()
        self.pushMenuEntries(self.top_level_handles_, marker.menu_entries, 0)

        server.insert(marker, self.processFeedback, InteractiveMarkerFeedback.MENU_SELECT)
        self.managed_markers_.add(marker_name)
        return True

    ## Re-apply to all markers that this was applied to previously
    def reApply(self, server):
        success = True
        # self.apply() might remove elements from
        # self.managed_markers_. To prevent errors, copy the
        # managed_markers sequence and iterate over the copy
        managed_markers = list(self.managed_markers_)
        for marker in managed_markers:
            success = self.apply(server, marker) and success
        return success

    ## @brief Get the title for the given menu entry
    ## @return The title, None if menu entry does not exist.
    def getTitle(self, handle):
        try:
            return self.entry_contexts_[handle].title
        except:
            return None

    # Call registered callback functions for given feedback command
    def processFeedback(self, feedback):
        try:
            context = self.entry_contexts_[feedback.menu_entry_id]
            context.feedback_cb(feedback)
        except KeyError:
            pass

    # Create and push MenuEntry objects from handles_in onto
    # entries_out. Calls itself recursively to add the entire menu tree.
    def pushMenuEntries(self, handles_in, entries_out, parent_handle):
        for handle in handles_in:
            try:
                context = self.entry_contexts_[handle]
                if not context.visible:
                    continue
                entries_out.append(self.makeEntry(context, handle, parent_handle))
                if not self.pushMenuEntries(context.sub_entries, entries_out, handle):
                    return False
            except:
                rospy.logerr("Internal error: context handle not found! This is a bug in MenuHandler.")
                return False
        return True

    def makeEntry(self, context, handle, parent_handle):
        menu_entry = MenuEntry()
        if context.check_state == self.NO_CHECKBOX:
            menu_entry.title = context.title
        elif context.check_state == self.CHECKED:
            menu_entry.title = "[x] "+context.title
        elif context.check_state == self.UNCHECKED:
            menu_entry.title = "[ ] "+context.title

        menu_entry.command = context.command
        menu_entry.command_type = context.command_type
        menu_entry.id = handle
        menu_entry.parent_id = parent_handle

        return menu_entry

    # Insert without adding a top-level entry
    def doInsert(self, title, command_type, command, feedback_cb):
        handle = self.current_handle_
        self.current_handle_ += 1

        context = EntryContext()
        context.title = title
        context.command = command
        context.command_type = command_type
        context.visible = True
        context.check_state = self.NO_CHECKBOX
        context.feedback_cb = feedback_cb

        self.entry_contexts_[handle] = context
        return handle
