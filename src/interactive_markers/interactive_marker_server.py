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

from std_msgs.msg import Header

from visualization_msgs.msg import InteractiveMarker
from visualization_msgs.msg import InteractiveMarkerFeedback
from visualization_msgs.msg import InteractiveMarkerInit
from visualization_msgs.msg import InteractiveMarkerPose
from visualization_msgs.msg import InteractiveMarkerUpdate

from threading import Lock


# Represents a single marker
class MarkerContext:

    def __init__(self):
        self.last_feedback = rospy.Time.now()
        self.last_client_id = ""
        self.default_feedback_cb = None
        self.feedback_cbs = dict()
        self.int_marker = InteractiveMarker()


# Represents an update to a single marker
class UpdateContext:
    FULL_UPDATE = 0
    POSE_UPDATE = 1
    ERASE = 2

    def __init__(self):
        self.update_type = self.FULL_UPDATE
        self.int_marker = InteractiveMarker()
        self.default_feedback_cb = None
        self.feedback_cbs = dict()


## Acts as a server displaying a set of interactive markers.
##
## Note: Keep in mind that changes made by calling insert(), erase(), setCallback() etc.
##       are not applied until calling applyChanges().
class InteractiveMarkerServer:
    DEFAULT_FEEDBACK_CB = 255

    ## @brief Create an InteractiveMarkerServer and associated ROS connections
    ## @param topic_ns      The interface will use the topics topic_ns/update and
    ##                      topic_ns/feedback for communication.
    ## @param server_id     If you run multiple servers on the same topic from
    ##                      within the same node, you will need to assign different names to them.
    ##                      Otherwise, leave this empty.
    def __init__(self, topic_ns, server_id="", q_size=100):
        self.topic_ns = topic_ns
        self.seq_num = 0
        self.mutex = Lock()

        self.server_id = rospy.get_name() + server_id

        # contains the current state of all markers
        # string : MarkerContext
        self.marker_contexts = dict()

        # updates that have to be sent on the next publish
        # string : UpdateContext
        self.pending_updates = dict()

        self.init_pub = rospy.Publisher(topic_ns+"/update_full", InteractiveMarkerInit, latch=True, queue_size=100)
        self.update_pub = rospy.Publisher(topic_ns+"/update", InteractiveMarkerUpdate, queue_size=100)

        rospy.Subscriber(topic_ns+"/feedback", InteractiveMarkerFeedback, self.processFeedback, queue_size=q_size)
        rospy.Timer(rospy.Duration(0.5), self.keepAlive)

        self.publishInit()

    ## @brief Destruction of the interface will lead to all managed markers being cleared.
    def __del__(self):
        self.clear()
        self.applyChanges()

    ## @brief Add or replace a marker.
    ## Note: Changes to the marker will not take effect until you call applyChanges().
    ## The callback changes immediately.
    ## @param marker The marker to be added or replaced
    ## @param feedback_cb Function to call on the arrival of a feedback message.
    ## @param feedback_type Type of feedback for which to call the feedback.
    def insert(self, marker, feedback_cb=-1, feedback_type=DEFAULT_FEEDBACK_CB):
        with self.mutex:
            try:
                update = self.pending_updates[marker.name]
            except:
                update = UpdateContext()
                self.pending_updates[marker.name] = update
            update.update_type = UpdateContext.FULL_UPDATE
            update.int_marker = marker
        if feedback_cb != -1:
            self.setCallback(marker.name, feedback_cb, feedback_type)

    ## @brief Update the pose of a marker with the specified name
    ## Note: This change will not take effect until you call applyChanges()
    ## @param name Name of the interactive marker
    ## @param pose The new pose
    ## @param header Header replacement. Leave this empty to use the previous one.
    ## @return True if a marker with that name exists
    def setPose(self, name, pose, header=Header()):
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

    ## @brief Erase the marker with the specified name
    ## Note: This change will not take effect until you call applyChanges().
    ## @param name Name of the interactive marker
    ## @return True if a marker with that name exists
    def erase(self, name):
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

    ## @brief Clear all markers.
    ## Note: This change will not take effect until you call applyChanges().
    def clear(self):
        self.pending_updates = dict()
        for marker_name in self.marker_contexts.keys():
            self.erase(marker_name)

    ## @brief Add or replace a callback function for the specified marker.
    ## Note: This change will not take effect until you call applyChanges().
    ## The server will try to call any type-specific callback first.
    ## If none is set, it will call the default callback.
    ## If a callback for the given type already exists, it will be replaced.
    ## To unset a type-specific callback, pass in an empty one.
    ## @param name           Name of the interactive marker
    ## @param feedback_cb    Function to call on the arrival of a feedback message.
    ## @param feedback_type  Type of feedback for which to call the feedback.
    ##                       Leave this empty to make this the default callback.
    def setCallback(self, name, feedback_cb, feedback_type=DEFAULT_FEEDBACK_CB):
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
                if feedback_type == self.DEFAULT_FEEDBACK_CB:
                    marker_context.default_feedback_cb = feedback_cb
                else:
                    if feedback_cb:
                        marker_context.feedback_cbs[feedback_type] = feedback_cb
                    else:
                        del marker_context.feedback_cbs[feedback_type]
            if update:
                if feedback_type == self.DEFAULT_FEEDBACK_CB:
                    update.default_feedback_cb = feedback_cb
                else:
                    if feedback_cb:
                        update.feedback_cbs[feedback_type] = feedback_cb
                    else:
                        del update.feedback_cbs[feedback_type]
            return True

    ## @brief Apply changes made since the last call to this method &
    ## broadcast an update to all clients.
    def applyChanges(self):
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
                        rospy.logdebug("Creating new context for " + name)
                        # create a new int_marker context
                        marker_context = MarkerContext()
                        marker_context.default_feedback_cb = update.default_feedback_cb
                        marker_context.feedback_cbs = update.feedback_cbs
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
                        rospy.logerr("""\
Pending pose update for non-existing marker found. This is a bug in InteractiveMarkerInterface.""")

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
        self.publishInit()

    ## @brief Get marker by name
    ## @param name Name of the interactive marker
    ## @return Marker if exists, None otherwise
    def get(self, name):
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

    # update marker pose & call user callback.
    def processFeedback(self, feedback):
        with self.mutex:
            # ignore feedback for non-existing markers
            try:
                marker_context = self.marker_contexts[feedback.marker_name]
            except:
                return

            # if two callers try to modify the same marker, reject (timeout= 1 sec)
            if marker_context.last_client_id != feedback.client_id \
               and (rospy.Time.now() - marker_context.last_feedback).to_sec() < 1.0:
                rospy.logdebug("Rejecting feedback for " +
                               feedback.marker_name +
                               ": conflicting feedback from separate clients.")
                return

            marker_context.last_feedback = rospy.Time.now()
            marker_context.last_client_id = feedback.client_id

            if feedback.event_type == feedback.POSE_UPDATE:
                if marker_context.int_marker.header.stamp == rospy.Time():
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
        try:
            feedback_cb = marker_context.feedback_cbs[feedback.event_type]
            feedback_cb(feedback)
        except KeyError:
            #try:
            marker_context.default_feedback_cb(feedback)
            #except:
                #pass

    # send an empty update to keep the client GUIs happy
    def keepAlive(self, msg):
        empty_msg = InteractiveMarkerUpdate()
        empty_msg.type = empty_msg.KEEP_ALIVE
        self.publish(empty_msg)

    # increase sequence number & publish an update
    def publish(self, update):
        update.server_id = self.server_id
        update.seq_num = self.seq_num
        self.update_pub.publish(update)

    # publish the current complete state to the latched "init" topic
    def publishInit(self):
        with self.mutex:
            init = InteractiveMarkerInit()
            init.server_id = self.server_id
            init.seq_num = self.seq_num

            for name, marker_context in self.marker_contexts.items():
                rospy.logdebug("Publishing " + name)
                init.markers.append(marker_context.int_marker)

            self.init_pub.publish(init)

    # update pose, schedule update without locking
    def doSetPose(self, update, name, pose, header):
        if update is None:
            update = UpdateContext()
            update.update_type = UpdateContext.POSE_UPDATE
            self.pending_updates[name] = update
        elif update.update_type != UpdateContext.FULL_UPDATE:
            update.update_type = UpdateContext.POSE_UPDATE

        update.int_marker.pose = pose
        update.int_marker.header = header
        rospy.logdebug("Marker '" + name + "' is now at " +
                       str(pose.position.x) + ", " + str(pose.position.y) +
                       ", " + str(pose.position.z))
