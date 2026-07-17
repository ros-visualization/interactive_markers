# Copyright (c) 2011, Willow Garage, Inc.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#    * Redistributions of source code must retain the above copyright
#      notice, this list of conditions and the following disclaimer.
#
#    * Redistributions in binary form must reproduce the above copyright
#      notice, this list of conditions and the following disclaimer in the
#      documentation and/or other materials provided with the distribution.
#
#    * Neither the name of the copyright holder nor the names of its
#      contributors may be used to endorse or promote products derived from
#      this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

# Author: Michael Ferguson

from collections.abc import Callable
from dataclasses import dataclass
from dataclasses import field
import sys

from visualization_msgs.msg import InteractiveMarkerFeedback
from visualization_msgs.msg import MenuEntry


@dataclass
class EntryContext:
    """Represents a single menu entry."""

    title: str = ''
    command: str = ''
    command_type: int = 0
    sub_entries: list[int] = field(default_factory=list)
    visible: bool = True
    check_state: int = 0
    feedback_cb: Callable | None = None


class MenuHandler:
    """Creates a menu and maps its entries to function callbacks."""

    NO_CHECKBOX = 0
    CHECKED = 1
    UNCHECKED = 2

    def __init__(self):
        self.top_level_handles_: list[int] = []
        self.entry_contexts_: dict[int, EntryContext] = {}
        self.current_handle_ = 1
        self.managed_markers_: set[str] = set()

    def insert(
        self,
        title,
        parent=None,
        command_type=MenuEntry.FEEDBACK,
        command='',
        callback=None
    ):
        """Insert a new menu item."""
        handle = self.doInsert(title, command_type, command, callback)
        if parent is not None:
            if parent not in self.entry_contexts_:
                print(f"Parent menu entry '{parent}' not found", file=sys.stderr)
                return None
            parent_context = self.entry_contexts_[parent]
            parent_context.sub_entries.append(handle)
        else:
            self.top_level_handles_.append(handle)
        return handle

    def setVisible(self, handle, visible):
        """Specify if an entry should be visible or hidden."""
        if handle not in self.entry_contexts_:
            return False

        context = self.entry_contexts_[handle]
        context.visible = visible
        return True

    def setCheckState(self, handle, check_state):
        """Specify if an entry is checked or can't be checked at all."""
        if handle not in self.entry_contexts_:
            return False

        context = self.entry_contexts_[handle]
        context.check_state = check_state
        return True

    def getCheckState(self, handle):
        """
        Get the current state of an entry.

        :return: CheckState if the entry exists and has checkbox, None otherwise.
        """
        if handle not in self.entry_contexts_:
            return None

        context = self.entry_contexts_[handle]
        return context.check_state

    def apply(self, server, marker_name):
        """
        Copy current menu state into the marker given by the specified name.

        Divert callback for MENU_SELECT feedback to this manager.
        """
        marker = server.get(marker_name)
        if marker is None:
            self.managed_markers_.discard(marker_name)
            return False

        marker.menu_entries = []
        self.pushMenuEntries(self.top_level_handles_, marker.menu_entries, 0)

        server.insert(
            marker,
            feedback_callback=self.processFeedback,
            feedback_type=InteractiveMarkerFeedback.MENU_SELECT
        )
        self.managed_markers_.add(marker_name)
        return True

    def reApply(self, server):
        """Re-apply to all markers that this was applied to previously."""
        success = True
        # self.apply() might remove elements from
        # self.managed_markers_. To prevent errors, copy the
        # managed_markers sequence and iterate over the copy
        managed_markers = list(self.managed_markers_)
        for marker in managed_markers:
            success = self.apply(server, marker) and success
        return success

    def getTitle(self, handle):
        """
        Get the title for the given menu entry.

        :return: The title, None if menu entry does not exist.
        """
        if handle not in self.entry_contexts_:
            return None
        return self.entry_contexts_[handle].title

    def processFeedback(self, feedback):
        """Call registered callback functions for given feedback command."""
        if feedback.menu_entry_id not in self.entry_contexts_:
            return

        context = self.entry_contexts_[feedback.menu_entry_id]
        if context.feedback_cb is not None:
            context.feedback_cb(feedback)

    def pushMenuEntries(self, handles_in, entries_out, parent_handle):
        """
        Create and push MenuEntry objects from handles_in onto entries_out.

        Calls itself recursively to add the entire menu tree.
        """
        for handle in handles_in:
            if handle not in self.entry_contexts_:
                print(
                    'Internal error: context handle not found! This is a bug in MenuHandler.',
                    file=sys.stderr
                )
                return False

            context = self.entry_contexts_[handle]
            if not context.visible:
                continue
            entries_out.append(self.makeEntry(context, handle, parent_handle))
            if not self.pushMenuEntries(context.sub_entries, entries_out, handle):
                return False
        return True

    def makeEntry(self, context, handle, parent_handle):
        menu_entry = MenuEntry()
        if context.check_state == self.NO_CHECKBOX:
            menu_entry.title = context.title
        elif context.check_state == self.CHECKED:
            menu_entry.title = '[x] ' + context.title
        elif context.check_state == self.UNCHECKED:
            menu_entry.title = '[ ] ' + context.title

        menu_entry.command = context.command
        menu_entry.command_type = context.command_type
        menu_entry.id = handle
        menu_entry.parent_id = parent_handle

        return menu_entry

    def doInsert(self, title, command_type, command, feedback_cb):
        """Insert without adding a top-level entry."""
        handle = self.current_handle_
        self.current_handle_ += 1

        self.entry_contexts_[handle] = EntryContext(
            title=title,
            command=command,
            command_type=command_type,
            visible=True,
            check_state=self.NO_CHECKBOX,
            feedback_cb=feedback_cb,
        )
        return handle
