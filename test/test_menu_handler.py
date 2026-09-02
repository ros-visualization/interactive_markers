# Copyright (c) 2026, Open Source Robotics Foundation, Inc.
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

"""
Unit tests for the MenuHandler.

MenuHandler has no dependency on a live ROS graph: it only requires an object
exposing get() and insert(), so these tests run against a fake server and need
neither rclpy.init() nor an executor.
"""

from interactive_markers import MenuHandler

import pytest

from visualization_msgs.msg import InteractiveMarker
from visualization_msgs.msg import InteractiveMarkerFeedback
from visualization_msgs.msg import MenuEntry


class FakeServer:
    """Minimal stand-in for InteractiveMarkerServer, recording what was inserted."""

    def __init__(self, markers=()):
        self.markers = {marker.name: marker for marker in markers}
        self.inserted = []

    def get(self, name):
        """Return the marker with the given name, or None."""
        return self.markers.get(name)

    def insert(self, marker, *, feedback_callback=None, feedback_type=None):
        """Record an insertion and store the marker."""
        self.inserted.append((marker, feedback_callback, feedback_type))
        self.markers[marker.name] = marker


def make_marker(name='test_marker'):
    """Build a minimal interactive marker."""
    marker = InteractiveMarker()
    marker.name = name
    marker.header.frame_id = 'test_frame_id'
    return marker


@pytest.fixture
def handler():
    """Return a fresh MenuHandler."""
    return MenuHandler()


def test_insert_returns_distinct_handles(handler):
    """Every inserted entry gets a unique handle."""
    first = handler.insert('First')
    second = handler.insert('Second')
    assert first != second
    assert handler.getTitle(first) == 'First'
    assert handler.getTitle(second) == 'Second'


def test_insert_with_parent(handler):
    """An entry inserted with a parent is nested, not top level."""
    parent = handler.insert('Parent')
    child = handler.insert('Child', parent=parent)
    assert child is not None
    assert handler.getTitle(child) == 'Child'


def test_insert_with_unknown_parent_returns_none(handler):
    """Inserting under a non-existent parent returns None."""
    assert handler.insert('Orphan', parent=42) is None


def test_get_title_unknown_handle_returns_none(handler):
    """getTitle() returns None for an unknown handle."""
    assert handler.getTitle(42) is None


def test_set_visible(handler):
    """setVisible() succeeds for a known handle and fails for an unknown one."""
    handle = handler.insert('Entry')
    assert handler.setVisible(handle, False) is True
    assert handler.setVisible(42, False) is False


def test_set_and_get_check_state(handler):
    """A check state round-trips through set/get."""
    handle = handler.insert('Entry')
    assert handler.getCheckState(handle) == MenuHandler.NO_CHECKBOX
    assert handler.setCheckState(handle, MenuHandler.CHECKED) is True
    assert handler.getCheckState(handle) == MenuHandler.CHECKED


def test_set_check_state_unknown_handle_returns_false(handler):
    """setCheckState() fails for an unknown handle."""
    assert handler.setCheckState(42, MenuHandler.CHECKED) is False


def test_get_check_state_unknown_handle_returns_none(handler):
    """getCheckState() returns None for an unknown handle."""
    assert handler.getCheckState(42) is None


def test_apply_copies_menu_entries_to_marker(handler):
    """apply() populates the marker's menu entries and re-inserts it."""
    handler.insert('Entry', command='some_command')
    marker = make_marker()
    server = FakeServer([marker])

    assert handler.apply(server, 'test_marker') is True

    assert len(marker.menu_entries) == 1
    assert marker.menu_entries[0].title == 'Entry'
    assert marker.menu_entries[0].command == 'some_command'
    assert marker.menu_entries[0].parent_id == 0

    assert len(server.inserted) == 1
    _, feedback_callback, feedback_type = server.inserted[0]
    assert feedback_callback == handler.processFeedback
    assert feedback_type == InteractiveMarkerFeedback.MENU_SELECT


def test_apply_nests_sub_entries(handler):
    """Sub-entries are emitted with their parent's handle as parent_id."""
    parent = handler.insert('Parent')
    handler.insert('Child', parent=parent)
    marker = make_marker()
    server = FakeServer([marker])

    assert handler.apply(server, 'test_marker') is True

    titles = [entry.title for entry in marker.menu_entries]
    assert titles == ['Parent', 'Child']
    child_entry = marker.menu_entries[1]
    assert child_entry.parent_id == parent


def test_apply_skips_invisible_entries(handler):
    """Entries marked invisible are not pushed onto the marker."""
    visible = handler.insert('Visible')
    hidden = handler.insert('Hidden')
    handler.setVisible(hidden, False)
    marker = make_marker()
    server = FakeServer([marker])

    assert handler.apply(server, 'test_marker') is True

    titles = [entry.title for entry in marker.menu_entries]
    assert titles == ['Visible']
    assert handler.getTitle(visible) == 'Visible'


def test_apply_renders_check_state_prefixes(handler):
    """Checked and unchecked entries are prefixed in the rendered title."""
    checked = handler.insert('Checked')
    unchecked = handler.insert('Unchecked')
    handler.setCheckState(checked, MenuHandler.CHECKED)
    handler.setCheckState(unchecked, MenuHandler.UNCHECKED)
    marker = make_marker()
    server = FakeServer([marker])

    assert handler.apply(server, 'test_marker') is True

    titles = [entry.title for entry in marker.menu_entries]
    assert titles == ['[x] Checked', '[ ] Unchecked']


def test_apply_uses_configured_command_type(handler):
    """The command type given to insert() reaches the emitted MenuEntry."""
    handler.insert('Entry', command_type=MenuEntry.ROSRUN, command='a_command')
    marker = make_marker()
    server = FakeServer([marker])

    assert handler.apply(server, 'test_marker') is True
    assert marker.menu_entries[0].command_type == MenuEntry.ROSRUN


def test_apply_unknown_marker_returns_false(handler):
    """
    apply() returns False for a marker the server does not have.

    Regression test: this used to raise KeyError, because the marker name was
    never in managed_markers_ and set.remove() raises when the element is absent.
    """
    handler.insert('Entry')
    server = FakeServer()

    assert handler.apply(server, 'no_such_marker') is False


def test_apply_stops_managing_a_marker_that_disappears(handler):
    """A previously applied marker that vanishes is dropped from management."""
    handler.insert('Entry')
    marker = make_marker()
    server = FakeServer([marker])
    assert handler.apply(server, 'test_marker') is True

    # The marker goes away, e.g. it was erased on the server.
    del server.markers['test_marker']

    assert handler.apply(server, 'test_marker') is False
    # Applying a second time must still not raise, even though it is
    # no longer managed.
    assert handler.apply(server, 'test_marker') is False


def test_reapply_reapplies_to_managed_markers(handler):
    """reApply() re-inserts every marker previously applied to."""
    handler.insert('Entry')
    first = make_marker('marker_0')
    second = make_marker('marker_1')
    server = FakeServer([first, second])
    assert handler.apply(server, 'marker_0') is True
    assert handler.apply(server, 'marker_1') is True
    server.inserted.clear()

    assert handler.reApply(server) is True
    reinserted = {marker.name for marker, _, _ in server.inserted}
    assert reinserted == {'marker_0', 'marker_1'}


def test_reapply_returns_false_when_a_marker_is_gone(handler):
    """reApply() reports failure when one of its markers no longer exists."""
    handler.insert('Entry')
    first = make_marker('marker_0')
    second = make_marker('marker_1')
    server = FakeServer([first, second])
    assert handler.apply(server, 'marker_0') is True
    assert handler.apply(server, 'marker_1') is True

    del server.markers['marker_0']

    assert handler.reApply(server) is False


def test_process_feedback_invokes_callback(handler):
    """Feedback for an entry is routed to that entry's callback."""
    received = []
    handle = handler.insert('Entry', callback=received.append)

    feedback = InteractiveMarkerFeedback()
    feedback.menu_entry_id = handle
    handler.processFeedback(feedback)

    assert received == [feedback]


def test_process_feedback_unknown_entry_is_ignored(handler):
    """Feedback for an unknown entry id is silently ignored."""
    feedback = InteractiveMarkerFeedback()
    feedback.menu_entry_id = 42
    handler.processFeedback(feedback)


def test_process_feedback_entry_without_callback(handler):
    """
    Feedback for an entry inserted without a callback is ignored.

    Regression test: this used to raise TypeError by calling None. Entries
    without a callback are routine -- a submenu parent is created that way.
    """
    parent = handler.insert('Submenu')
    handler.insert('Child', parent=parent, callback=lambda feedback: None)

    feedback = InteractiveMarkerFeedback()
    feedback.menu_entry_id = parent
    handler.processFeedback(feedback)
