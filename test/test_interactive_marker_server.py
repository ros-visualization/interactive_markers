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
Tests for the Python InteractiveMarkerServer.

This mirrors test/interactive_markers/test_interactive_marker_server.cpp. The
C++ client tests have no counterpart here because the Python package ships no
interactive marker client.
"""

import time

from geometry_msgs.msg import Pose

from interactive_markers import InteractiveMarkerServer

import pytest

import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSProfile

from std_msgs.msg import Header

from visualization_msgs.msg import InteractiveMarker
from visualization_msgs.msg import InteractiveMarkerControl
from visualization_msgs.msg import InteractiveMarkerFeedback
from visualization_msgs.msg import InteractiveMarkerUpdate
from visualization_msgs.msg import MenuEntry
from visualization_msgs.srv import GetInteractiveMarkers

TOPIC_NAMESPACE = 'test_namespace'
DISCOVERY_TIMEOUT = 5.0


def get_interactive_markers():
    """Build the same two fixture markers as interactive_marker_fixtures.cpp."""
    markers = []

    marker = InteractiveMarker()
    marker.name = 'test_marker_0'
    marker.header.frame_id = 'test_frame_id'
    markers.append(marker)

    marker = InteractiveMarker()
    marker.name = 'test_marker_1'
    marker.header.frame_id = 'test_frame_id'
    marker.pose.position.x = 1.0
    marker.pose.orientation.w = 1.0
    marker.description = 'My test marker description'
    marker.scale = 3.14
    menu_entry = MenuEntry()
    menu_entry.id = 42
    menu_entry.title = 'My test menu title'
    menu_entry.command = 'Some test command to be run'
    marker.menu_entries.append(menu_entry)
    control = InteractiveMarkerControl()
    control.name = 'test_control_name'
    control.orientation.w = 1.0
    control.always_visible = True
    control.description = 'My test control description'
    marker.controls.append(control)
    markers.append(marker)

    return markers


class MockInteractiveMarkerClient(Node):
    """Stand-in for a client, mirroring mock_interactive_marker_client.hpp."""

    def __init__(self, topic_namespace=TOPIC_NAMESPACE):
        super().__init__('mock_interactive_marker_client')
        self.updates_received = 0
        self.last_update_message = None
        self.client = self.create_client(
            GetInteractiveMarkers, topic_namespace + '/get_interactive_markers')
        self.publisher = self.create_publisher(
            InteractiveMarkerFeedback, topic_namespace + '/feedback', 1)
        self.subscription = self.create_subscription(
            InteractiveMarkerUpdate, topic_namespace + '/update', self._on_update, 1)

    def _on_update(self, message):
        self.updates_received += 1
        self.last_update_message = message

    def publish_feedback(self, feedback):
        """Publish a feedback message to the server."""
        self.publisher.publish(feedback)

    def request_interactive_markers(self):
        """Send a GetInteractiveMarkers request and return the future."""
        return self.client.call_async(GetInteractiveMarkers.Request())


def spin_until(executor, predicate, timeout=DISCOVERY_TIMEOUT):
    """Spin the executor until the predicate holds or the timeout expires."""
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if predicate():
            return True
        executor.spin_once(timeout_sec=0.01)
    return predicate()


def publish_until(executor, publish, predicate, timeout=DISCOVERY_TIMEOUT):
    """
    Republish while spinning, until the predicate holds or the timeout expires.

    The server drops feedback from a client whose id differs from the marker's
    last_client_id if it arrives within a second of the marker's last_feedback
    stamp, which MarkerContext initializes to the marker's creation time. A
    marker's first feedback is therefore rejected for a second after
    applyChanges() creates it. Republishing rides out that window rather than
    depending on discovery happening to take longer than it.
    """
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if predicate():
            return True
        publish()
        spin_until(executor, predicate, timeout=0.25)
    return predicate()


@pytest.fixture(scope='module', autouse=True)
def ros_context():
    """Initialize rclpy once for the whole module."""
    rclpy.init()
    yield
    rclpy.shutdown()


@pytest.fixture
def node():
    """Return a node, destroyed on teardown."""
    node = rclpy.create_node('test_interactive_marker_server_node')
    yield node
    node.destroy_node()


@pytest.fixture
def server_with_markers(node):
    """Return a server preloaded with the fixture markers, plus those markers."""
    server = InteractiveMarkerServer(node, TOPIC_NAMESPACE)
    markers = get_interactive_markers()
    for marker in markers:
        server.insert(marker)
    server.applyChanges()
    yield server, markers
    server.shutdown()


@pytest.fixture
def server_and_client(node):
    """Return a preloaded server wired to a mock client on a spinning executor."""
    server = InteractiveMarkerServer(node, TOPIC_NAMESPACE)
    markers = get_interactive_markers()
    for marker in markers:
        server.insert(marker)
    server.applyChanges()

    client = MockInteractiveMarkerClient(TOPIC_NAMESPACE)
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    executor.add_node(client)

    assert client.client.wait_for_service(timeout_sec=DISCOVERY_TIMEOUT), \
        'Timed out waiting for the get_interactive_markers service'
    assert spin_until(executor, lambda: client.publisher.get_subscription_count() == 1), \
        'Timed out waiting for the server to discover the feedback publisher'

    yield server, markers, client, executor

    executor.remove_node(client)
    executor.remove_node(node)
    server.shutdown()
    client.destroy_node()


def test_construction_and_destruction(node):
    """A server can be constructed with default and explicit QoS, then shut down."""
    server = InteractiveMarkerServer(node, '')
    server.shutdown()

    server = InteractiveMarkerServer(node, 'test_server')
    server.shutdown()

    server = InteractiveMarkerServer(
        node,
        'test_server',
        update_pub_qos=QoSProfile(depth=42),
        feedback_sub_qos=QoSProfile(depth=99),
    )
    server.shutdown()


def test_insert(node):
    """Inserted markers are not visible until applyChanges() is called."""
    server = InteractiveMarkerServer(node, 'test_insert_server')
    markers = get_interactive_markers()
    for marker in markers:
        server.insert(marker)

    assert server.empty()
    assert server.size() == 0
    assert len(server) == 0

    server.applyChanges()

    assert not server.empty()
    assert server.size() == len(markers)
    assert len(server) == len(markers)

    server.shutdown()


def test_erase(server_with_markers):
    """Erasing takes effect only on applyChanges(); erasing an unknown marker fails."""
    server, markers = server_with_markers

    assert server.erase(markers[0].name) is True
    assert server.size() == len(markers)
    server.applyChanges()
    assert server.size() == len(markers) - 1

    # Erase a marker that has just been inserted
    server.insert(markers[0])
    assert server.erase(markers[0].name) is True
    assert server.size() == len(markers) - 1
    server.applyChanges()
    assert server.size() == len(markers) - 1

    # Erase an invalid marker
    assert server.erase("this_Is_the_name_0f_a_marker_that_doesn't ex1st") is False
    server.applyChanges()
    assert server.size() == len(markers) - 1


def test_clear(server_with_markers):
    """Clearing takes effect only on applyChanges(), and is safe when empty."""
    server, markers = server_with_markers

    server.clear()
    assert server.size() == len(markers)
    server.applyChanges()
    assert server.size() == 0

    # Clear an empty server
    server.clear()
    assert server.size() == 0


def test_get_marker_by_name(server_with_markers):
    """Markers round-trip through get(); unknown and pending-erase markers return None."""
    server, markers = server_with_markers

    for input_marker in markers:
        output_marker = server.get(input_marker.name)
        assert output_marker is not None
        assert output_marker.header.frame_id == input_marker.header.frame_id
        assert output_marker.pose.position.x == input_marker.pose.position.x
        assert output_marker.pose.orientation.w == input_marker.pose.orientation.w
        assert output_marker.name == input_marker.name
        assert output_marker.description == input_marker.description

        assert len(output_marker.menu_entries) == len(input_marker.menu_entries)
        for output_entry, input_entry in zip(
                output_marker.menu_entries, input_marker.menu_entries):
            assert output_entry.id == input_entry.id
            assert output_entry.title == input_entry.title
            assert output_entry.command == input_entry.command

        assert len(output_marker.controls) == len(input_marker.controls)
        for output_control, input_control in zip(output_marker.controls, input_marker.controls):
            assert output_control.name == input_control.name
            assert output_control.always_visible == input_control.always_visible

    # Get an invalid marker
    assert server.get('n0t_the_name_of_@ marker') is None

    # Get a pending erased marker
    assert server.erase(markers[0].name) is True
    assert server.get(markers[0].name) is None


def test_set_pose(server_with_markers):
    """setPose() updates the pose, keeps the old header by default, and rejects unknowns."""
    server, markers = server_with_markers

    pose = Pose()
    pose.position.x = 1.0
    pose.position.y = -2.0
    pose.position.z = 3.14
    pose.orientation.w = 0.5
    assert server.setPose(markers[0].name, pose) is True
    server.applyChanges()

    output_marker = server.get(markers[0].name)
    assert output_marker is not None
    assert output_marker.pose.position.x == pose.position.x
    assert output_marker.pose.position.y == pose.position.y
    assert output_marker.pose.position.z == pose.position.z
    assert output_marker.pose.orientation.w == pose.orientation.w
    assert output_marker.header.frame_id == markers[0].header.frame_id


def test_set_pose_with_header(server_with_markers):
    """A header with a frame_id replaces the marker's existing header."""
    server, markers = server_with_markers

    # Snapshot the original frame_id: the server stores the caller's marker by
    # reference, so markers[0] is mutated in place by the update below.
    original_frame_id = markers[0].header.frame_id

    pose = Pose()
    pose.position.x = 1.0
    pose.position.y = -2.0
    pose.position.z = 3.14
    pose.orientation.w = 0.5
    header = Header()
    header.frame_id = 'test_updating_to_a_new_header'
    assert server.setPose(markers[0].name, pose, header) is True
    server.applyChanges()

    output_marker = server.get(markers[0].name)
    assert output_marker is not None
    assert output_marker.pose.position.x == pose.position.x
    assert output_marker.pose.orientation.w == pose.orientation.w
    assert output_marker.header.frame_id != original_frame_id
    assert output_marker.header.frame_id == header.frame_id


def test_set_pose_of_invalid_marker(server_with_markers):
    """setPose() returns False for a marker that does not exist."""
    server, _ = server_with_markers

    pose = Pose()
    pose.orientation.w = 1.0
    assert server.setPose('test_n0t_a_valid_marker_n@me', pose) is False


def test_set_pose_default_header_is_not_shared(server_with_markers):
    """
    The default header argument is not shared between markers.

    Regression guard for the mutable default argument on setPose(): a shared
    default Header instance could be aliased into more than one marker.
    """
    server, markers = server_with_markers

    pose = Pose()
    pose.orientation.w = 1.0
    assert server.setPose(markers[0].name, pose) is True
    assert server.setPose(markers[1].name, pose) is True
    server.applyChanges()

    first = server.get(markers[0].name)
    second = server.get(markers[1].name)
    assert first.header is not second.header


def test_set_callback(server_with_markers):
    """setCallback() succeeds for a known marker and fails for an unknown one."""
    server, markers = server_with_markers

    assert server.setCallback(markers[0].name, None) is True
    assert server.setCallback('test_n0t_a_valid_marker_n@me', None) is False


def test_feedback_communication(server_and_client):
    """Feedback published by a client reaches the registered callback."""
    server, markers, client, executor = server_and_client

    received = []
    assert server.setCallback(markers[0].name, received.append) is True
    server.applyChanges()

    feedback = InteractiveMarkerFeedback()
    feedback.client_id = 'test_client_id'
    feedback.marker_name = markers[0].name
    feedback.event_type = InteractiveMarkerFeedback.POSE_UPDATE
    feedback.pose.position.x = -3.14
    feedback.pose.orientation.w = 1.0

    assert publish_until(
        executor,
        lambda: client.publish_feedback(feedback),
        lambda: len(received) > 0,
    ), 'Timed out waiting for feedback'
    output_feedback = received[0]
    assert output_feedback.client_id == feedback.client_id
    assert output_feedback.marker_name == markers[0].name
    assert output_feedback.pose.position.x == feedback.pose.position.x
    assert output_feedback.pose.orientation.w == feedback.pose.orientation.w


def test_update_communication(server_and_client):
    """Adding, modifying and erasing markers each publish exactly one update."""
    server, markers, client, executor = server_and_client

    assert client.updates_received == 0
    # This should not trigger an update publication
    server.applyChanges()

    # Adding a marker should trigger an update
    marker = InteractiveMarker()
    marker.name = 'test_update_from_added_marker'
    server.insert(marker)
    server.applyChanges()
    assert spin_until(executor, lambda: client.updates_received == 1), \
        'Timed out waiting for the insert update'
    assert client.last_update_message is not None
    assert len(client.last_update_message.markers) == 1
    assert len(client.last_update_message.poses) == 0
    assert len(client.last_update_message.erases) == 0

    # Modifying a marker should trigger an update
    pose = Pose()
    pose.orientation.w = 1.0
    server.setPose(markers[0].name, pose)
    server.applyChanges()
    assert spin_until(executor, lambda: client.updates_received == 2), \
        'Timed out waiting for the pose update'
    assert len(client.last_update_message.markers) == 0
    assert len(client.last_update_message.poses) == 1
    assert len(client.last_update_message.erases) == 0

    # Erasing a marker should trigger an update
    assert server.erase(markers[0].name) is True
    server.applyChanges()
    assert spin_until(executor, lambda: client.updates_received == 3), \
        'Timed out waiting for the erase update'
    assert len(client.last_update_message.markers) == 0
    assert len(client.last_update_message.poses) == 0
    assert len(client.last_update_message.erases) == 1
    assert client.last_update_message.erases[0] == markers[0].name


def test_get_interactive_markers_communication(server_and_client):
    """The GetInteractiveMarkers service returns every managed marker."""
    _, markers, client, executor = server_and_client

    future = client.request_interactive_markers()
    executor.spin_until_future_complete(future, timeout_sec=DISCOVERY_TIMEOUT)
    assert future.done(), 'Timed out waiting for the GetInteractiveMarkers response'
    response = future.result()

    assert len(response.markers) == len(markers)
    for response_marker, input_marker in zip(response.markers, markers):
        assert response_marker.header.frame_id == input_marker.header.frame_id
        assert response_marker.pose.position.x == input_marker.pose.position.x
        assert response_marker.pose.orientation.w == input_marker.pose.orientation.w
        assert response_marker.name == input_marker.name
        assert response_marker.description == input_marker.description

        assert len(response_marker.menu_entries) == len(input_marker.menu_entries)
        for response_entry, input_entry in zip(
                response_marker.menu_entries, input_marker.menu_entries):
            assert response_entry.id == input_entry.id
            assert response_entry.title == input_entry.title
            assert response_entry.command == input_entry.command

        assert len(response_marker.controls) == len(input_marker.controls)
        for response_control, input_control in zip(
                response_marker.controls, input_marker.controls):
            assert response_control.name == input_control.name
            assert response_control.always_visible == input_control.always_visible
