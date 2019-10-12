# Copyright (c) 2011, Willow Garage, Inc.
# All rights reserved.
#
# Software License Agreement (BSD License 2.0)
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of Willow Garage, Inc. nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

# Author: Michael Ferguson

import rclpy
from rclpy.duration import Duration
from rclpy.time import Time

from std_msgs.msg import Header

from visualization_msgs.msg import InteractiveMarker
from visualization_msgs.msg import InteractiveMarkerFeedback
from visualization_msgs.msg import InteractiveMarkerPose
from visualization_msgs.msg import InteractiveMarkerUpdate
from visualization_msgs.srv import GetInteractiveMarkers

from threading import Lock


class MarkerContext:
    """Represents a single marker."""

    def __init__(self, time):
        self.last_feedback = time
        self.last_client_id = ""
        self.default_feedback_callback = None
        self.feedback_callbacks = dict()
        self.int_marker = InteractiveMarker()


class UpdateContext:
    """Represents an update to a single marker."""

    FULL_UPDATE = 0
    POSE_UPDATE = 1
    ERASE = 2

    def __init__(self):
        self.update_type = self.FULL_UPDATE
        self.int_marker = InteractiveMarker()
        self.default_feedback_callback = None
        self.feedback_callbacks = dict()


class InteractiveMarkerServer:
    """
    A server to one or many clients (e.g. rviz) displaying a set of interactive markers.

    Note: Keep in mind that changes made by calling insert(), erase(), setCallback() etc.
          are not applied until calling applyChanges().
    """

    DEFAULT_FEEDBACK_CALLBACK = 255

    def __init__(self, node, topic_ns, q_size=100):
        """
        Create an InteractiveMarkerServer and associated ROS connections.

        :param node: The node to attach this interactive marker server to.
        :param topic_ns: The interface will use the topics topic_ns/update and topic_ns/feedback
            for communication.
        """
        self.node = node
        self.topic_ns = topic_ns
        self.seq_num = 0
        self.mutex = Lock()

        # contains the current state of all markers
        # string : MarkerContext
        self.marker_contexts = dict()

        # updates that have to be sent on the next publish
        # string : UpdateContext
        self.pending_updates = dict()

        self.get_interactive_markers_srv = self.node.create_service(
            GetInteractiveMarkers,
            topic_ns + "/get_interactive_markers",
            self.getInteractiveMarkersCallback
        )

        self.update_pub = self.node.create_publisher(InteractiveMarkerUpdate, topic_ns + "/update", q_size)

        self.feedback_sub = self.node.create_subscription(InteractiveMarkerFeedback, topic_ns + "/feedback", self.processFeedback, q_size)

    def shutdown(self):
        """
        Shutdown the interactive marker server.

        This should be called before the node is destroyed so that the internal ROS entities
        can be destroyed.
        """
        self.clear()
        self.applyChanges()
        self.get_interactive_markers_srv = None
        self.update_pub = None
        self.feedback_sub = None

    def __del__(self):
        """Destruction of the interface will lead to all managed markers being cleared."""

        self.shutdown()

    def insert(self, marker, *, feedback_callback=None, feedback_type=DEFAULT_FEEDBACK_CALLBACK):
        """
        Add or replace a marker.

        Note: Changes to the marker will not take effect until you call applyChanges().
        The callback changes immediately.

        :param marker: The marker to be added or replaced.
        :param feedback_callback: Function to call on the arrival of a feedback message.
        :param feedback_type: Type of feedback for which to call the feedback.
        """

        with self.mutex:
            try:
                update = self.pending_updates[marker.name]
            except:
                update = UpdateContext()
                self.pending_updates[marker.name] = update
            update.update_type = UpdateContext.FULL_UPDATE
            update.int_marker = marker
        if feedback_callback is not None:
            self.setCallback(marker.name, feedback_callback, feedback_type)

    def setPose(self, name, pose, header=Header()):
        """
        Update the pose of a marker with the specified name.

        Note: This change will not take effect until you call applyChanges()

        :param name: Name of the interactive marker.
        :param pose: The new pose.
        :param header: Header replacement. Leave this empty to use the previous one.
        :return: True if a marker with that name exists, False otherwise.
        """

        with self.mutex:
            try:
                marker_context = self.marker_contexts[name]
            except:
                marker_context = None
            try:
                update = self.pending_updates[name]
            except:
                update = None
            # if there's no marker and no pending addition for it, we can't update the pose
            if marker_context is None and update is None:
                return False
            if update is not None and update.update_type == UpdateContext.FULL_UPDATE:
                return False

            if header.frame_id is None or header.frame_id == "":
                # keep the old header
                self.doSetPose(update, name, pose, marker_context.int_marker.header)
            else:
                self.doSetPose(update, name, pose, header)
            return True

    def erase(self, name):
        """
        Erase the marker with the specified name.

        Note: This change will not take effect until you call applyChanges().

        :param name: Name of the interactive marker.
        :return: True if a marker with that name exists, False otherwise.
        """

        with self.mutex:
            try:
                self.pending_updates[name].update_type = UpdateContext.ERASE
                return True
            except:
                try:
                    self.marker_contexts[name]  # check exists
                    update = UpdateContext()
                    update.update_type = UpdateContext.ERASE
                    self.pending_updates[name] = update
                    return True
                except:
                    return False

    def clear(self):
        """
        Clear all markers.

        Note: This change will not take effect until you call applyChanges().
        """

        self.pending_updates = dict()
        for marker_name in self.marker_contexts.keys():
            self.erase(marker_name)

    def setCallback(self, name, feedback_callback, feedback_type=DEFAULT_FEEDBACK_CALLBACK):
        """
        Add or replace a callback function for the specified marker.

        Note: This change will not take effect until you call applyChanges().
        The server will try to call any type-specific callback first.
        If a callback for the given type already exists, it will be replaced.
        To unset a callback, pass a value of None.

        :param name: Name of the interactive marker
        :param feedback_callback: Function to call on the arrival of a feedback message.
        :param feedback_type: Type of feedback for which to call the feedback.
            Leave this empty to make this the default callback.
        """

        with self.mutex:
            try:
                marker_context = self.marker_contexts[name]
            except:
                marker_context = None
            try:
                update = self.pending_updates[name]
            except:
                update = None
            if marker_context is None and update is None:
                return False

            # we need to overwrite both the callbacks for the actual marker
            # and the update, if there's any
            if marker_context:
                # the marker exists, so we can just overwrite the existing callbacks
                if feedback_type == self.DEFAULT_FEEDBACK_CALLBACK:
                    marker_context.default_feedback_callback = feedback_callback
                else:
                    if feedback_callback:
                        marker_context.feedback_callbacks[feedback_type] = feedback_callback
                    elif feedback_type in marker_context.feedback_callbacks:
                        del marker_context.feedback_callbacks[feedback_type]
            if update:
                if feedback_type == self.DEFAULT_FEEDBACK_CALLBACK:
                    update.default_feedback_callback = feedback_callback
                else:
                    if feedback_callback:
                        update.feedback_callbacks[feedback_type] = feedback_callback
                    elif feedback_type in update.feedback_callbacks:
                        del update.feedback_callbacks[feedback_type]
            return True

    def applyChanges(self):
        """Apply changes made since the last call to this method and broadcast to clients."""

        with self.mutex:
            if len(self.pending_updates.keys()) == 0:
                return

            update_msg = InteractiveMarkerUpdate()
            update_msg.type = InteractiveMarkerUpdate.UPDATE

            for name, update in self.pending_updates.items():
                if update.update_type == UpdateContext.FULL_UPDATE:
                    try:
                        marker_context = self.marker_contexts[name]
                    except:
                        self.node.get_logger().debug("Creating new context for " + name)
                        # create a new int_marker context
                        marker_context = MarkerContext(self.node.get_clock().now())
                        marker_context.default_feedback_callback = update.default_feedback_callback
                        marker_context.feedback_callbacks = update.feedback_callbacks
                        self.marker_contexts[name] = marker_context

                    marker_context.int_marker = update.int_marker
                    update_msg.markers.append(marker_context.int_marker)

                elif update.update_type == UpdateContext.POSE_UPDATE:
                    try:
                        marker_context = self.marker_contexts[name]
                        marker_context.int_marker.pose = update.int_marker.pose
                        marker_context.int_marker.header = update.int_marker.header

                        pose_update = InteractiveMarkerPose()
                        pose_update.header = marker_context.int_marker.header
                        pose_update.pose = marker_context.int_marker.pose
                        pose_update.name = marker_context.int_marker.name
                        update_msg.poses.append(pose_update)
                    except:
                        self.node.get_logger().error(
                            'Pending pose update for non-existing marker found. '
                            'This is a bug in InteractiveMarkerServer.')

                elif update.update_type == UpdateContext.ERASE:
                    try:
                        marker_context = self.marker_contexts[name]
                        del self.marker_contexts[name]
                        update_msg.erases.append(name)
                    except:
                        pass
                self.pending_updates = dict()

        self.seq_num += 1
        self.publish(update_msg)

    def get(self, name):
        """
        Get marker by name.

        :param name: Name of the interactive marker.
        :return: Marker if exists, None otherwise.
        """

        try:
            update = self.pending_updates[name]
        except:
            try:
                marker_context = self.marker_contexts[name]
            except:
                return None
            return marker_context.int_marker

        # if there's an update pending, we'll have to account for that
        if update.update_type == UpdateContext.ERASE:
            return None
        elif update.update_type == UpdateContext.POSE_UPDATE:
            try:
                marker_context = self.marker_contexts[name]
            except:
                return None
            int_marker = marker_context.int_marker
            int_marker.pose = update.int_marker.pose
            return int_marker
        elif update.update_type == UpdateContext.FULL_UPDATE:
            return update.int_marker
        return None

    def processFeedback(self, feedback):
        """Update marker pose and call user callback."""

        with self.mutex:
            # ignore feedback for non-existing markers
            try:
                marker_context = self.marker_contexts[feedback.marker_name]
            except:
                return

            # if two callers try to modify the same marker, reject (timeout= 1 sec)
            if marker_context.last_client_id != feedback.client_id \
               and (self.node.get_clock().now() - marker_context.last_feedback) < Duration(seconds=1.0):
                self.node.get_logger().debug("Rejecting feedback for " +
                               feedback.marker_name +
                               ": conflicting feedback from separate clients.")
                return

            marker_context.last_feedback = self.node.get_clock().now()
            marker_context.last_client_id = feedback.client_id

            if feedback.event_type == feedback.POSE_UPDATE:
                if marker_context.int_marker.header.stamp == Time(clock_type=self.node.get_clock().clock_type):
                    # keep the old header
                    try:
                        self.doSetPose(self.pending_updates[feedback.marker_name],
                                       feedback.marker_name,
                                       feedback.pose,
                                       marker_context.int_marker.header)
                    except:
                        self.doSetPose(None,
                                       feedback.marker_name,
                                       feedback.pose,
                                       marker_context.int_marker.header)
                else:
                    try:
                        self.doSetPose(self.pending_updates[feedback.marker_name],
                                       feedback.marker_name,
                                       feedback.pose,
                                       feedback.header)
                    except:
                        self.doSetPose(None, feedback.marker_name, feedback.pose, feedback.header)

        # call feedback handler
        feedback_callback = marker_context.feedback_callbacks.get(
            feedback.event_type, marker_context.default_feedback_callback)
        if feedback_callback is not None:
            feedback_callback(feedback)

    def publish(self, update):
        """Increase the sequence number and publish an update."""

        update.seq_num = self.seq_num
        self.update_pub.publish(update)

    def getInteractiveMarkersCallback(self, request, response):
        """Process a service request to get the current interactive markers."""

        with self.mutex:
            response.sequence_number = self.seq_num

            self.node.get_logger().debug('Markers requested. Responding with the following markers:')
            for name, marker_context in self.marker_contexts.items():
                self.node.get_logger().debug('    ' + name)
                response.markers.append(marker_context.int_marker)

            return response

    def doSetPose(self, update, name, pose, header):
        """Schedule a pose update pose without locking."""

        if update is None:
            update = UpdateContext()
            update.update_type = UpdateContext.POSE_UPDATE
            self.pending_updates[name] = update
        elif update.update_type != UpdateContext.FULL_UPDATE:
            update.update_type = UpdateContext.POSE_UPDATE

        update.int_marker.pose = pose
        update.int_marker.header = header
        self.node.get_logger().debug("Marker '" + name + "' is now at " +
                       str(pose.position.x) + ", " + str(pose.position.y) +
                       ", " + str(pose.position.z))
