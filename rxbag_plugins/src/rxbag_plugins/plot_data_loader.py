# Software License Agreement (BSD License)
#
# Copyright (c) 2009, Willow Garage, Inc.
# All rights reserved.
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

from __future__ import with_statement

PKG = 'rxbag_plugins'
import roslib; roslib.load_manifest(PKG)
import rospy

import bisect
import collections
import csv
import itertools
import sys
import threading
import time

from dataset import DataSet

class PlotDataLoader(threading.Thread):
    def __init__(self, timeline, topic):
        threading.Thread.__init__(self, target=self._run)

        self._timeline = timeline
        self._topic    = topic

        self._use_header_stamp = False

        self._start_stamp     = self._timeline.start_stamp
        self._end_stamp       = self._timeline.end_stamp
        self._paths           = []
        self._max_interval    = 0.0
        self._dirty           = True
        self._dirty_cv        = threading.Condition()
        self._last_reload     = None                           # time that entries were reloaded
        self._min_reload_secs = 0.1                            # minimum time to wait before loading entries

        self._data = {}

        self._load_complete = False

        self._progress_listeners = []
        self._complete_listeners = []

        self._stop_flag = False
        self.setDaemon(True)
        self.start()

    @property
    def data(self): return self._data

    @property
    def is_load_complete(self): return self._load_complete

    # listeners

    def add_progress_listener(self, listener):    self._progress_listeners.append(listener)
    def remove_progress_listener(self, listener): self._progress_listeners.remove(listener)

    def add_complete_listener(self, listener):    self._complete_listeners.append(listener)
    def remove_complete_listener(self, listener): self._complete_listeners.remove(listener)

    def invalidate(self):
        with self._dirty_cv:
            self._dirty = True
            self._dirty_cv.notify()

    # property: start_stamp

    def _get_start_stamp(self): return self._start_stamp
    
    def _set_start_stamp(self, start_stamp):
        with self._dirty_cv:
            if start_stamp != self._start_stamp:
                self._start_stamp = start_stamp
                self._dirty = True
                self._dirty_cv.notify()
        
    start_stamp = property(_get_start_stamp, _set_start_stamp)

    # property: end_stamp

    def _get_end_stamp(self): return self._end_stamp
    
    def _set_end_stamp(self, end_stamp):
        with self._dirty_cv:
            if end_stamp != self._end_stamp:
                self._end_stamp = end_stamp
                self._dirty = True
                self._dirty_cv.notify()
        
    end_stamp = property(_get_end_stamp, _set_end_stamp)

    def set_interval(self, start_stamp, end_stamp):
        with self._dirty_cv:
            updated = False
            if start_stamp != self._start_stamp:
                self._start_stamp = start_stamp
                updated = True
            if end_stamp != self._end_stamp:
                self._end_stamp = end_stamp
                updated = True
            if updated:
                self._dirty = True
                self._dirty_cv.notify()

    # property: paths

    def _get_paths(self): return self._paths

    def _set_paths(self, paths):
        with self._dirty_cv:
            if set(paths) != set(self._paths):
                self._paths = paths
                self._dirty = True
                self._dirty_cv.notify()

    paths = property(_get_paths, _set_paths)

    # property: max_interval
    
    def _get_max_interval(self): return self._max_interval
    
    def _set_max_interval(self, max_interval):
        with self._dirty_cv:
            if max_interval != self._max_interval:
                self._max_interval = max_interval
                self._dirty = True
                self._dirty_cv.notify()

    max_interval = property(_get_max_interval, _set_max_interval)

    def stop(self):
        self._stop_flag = True
        with self._dirty_cv:
            self._dirty_cv.notify()
        self.join()

    ##

    def _trim_data(self):
        # Keep data 25% outside of view range
        extension = rospy.Duration((self._end_stamp - self._start_stamp).to_sec() * 0.25)

        min_x = (self._start_stamp - extension - self._timeline.start_stamp).to_sec()
        max_x = (self._end_stamp   + extension - self._timeline.start_stamp).to_sec()

        for series in list(self._data.keys()):
            new_data = DataSet()

            points     = self._data[series].points
            num_points = len(points)

            for i, (x, y) in enumerate(points):
                if x >= min_x and x <= max_x and (i == 0 or x - points[i - 1][0] >= self._max_interval):
                    new_data.add(x, y)

            self._data[series] = new_data

    def _run(self):
        while not self._stop_flag:
            with self._dirty_cv:
                # If dirty, then regenerate the entries to load, and re-initialize the loaded indexes
                if self._dirty and (self._last_reload is None or time.time() - self._last_reload >= self._min_reload_secs):
                    # Load data outside of the view range
                    extension = rospy.Duration((self._end_stamp - self._start_stamp).to_sec() * 0.25)
                    
                    entries        = list(self._timeline.get_entries_with_bags(self._topic, self._start_stamp - extension, self._end_stamp + extension))
                    loaded_indexes = set()
                    loaded_stamps  = []
                    if len(entries) == 0:
                        subdivider = None
                    else:
                        subdivider = _subdivide(0, len(entries) - 1)

                    self._trim_data()

                    self._last_reload = time.time()
                    self._load_complete = False
                    self._dirty = False

                if subdivider is None or len(self._paths) == 0:
                    # Wait for dirty flag
                    if not self._dirty:
                        self._dirty_cv.wait()
                    continue

            try:
                index = subdivider.next()
            except StopIteration:
                self._load_complete = True
                
                # Notify listeners of completion
                for listener in itertools.chain(self._progress_listeners, self._complete_listeners):
                    listener()

                subdivider = None
                continue

            if index in loaded_indexes:
                continue

            bag, entry = entries[index]

            # If the timestamp is too close to an existing index, don't load it
            closest_stamp_index = bisect.bisect_left(loaded_stamps, entry.time)
            if closest_stamp_index < len(loaded_stamps):
                dist_to_closest_msg = abs((entry.time - loaded_stamps[closest_stamp_index]).to_sec())
                if dist_to_closest_msg < self._max_interval:
                    loaded_indexes.add(index)
                    continue

            topic, msg, msg_stamp = self._timeline.read_message(bag, entry.position)
            if not msg:
                continue

            bisect.insort_left(loaded_stamps, msg_stamp)

            # Extract the field data from the message
            for path in self._paths:
                if self._use_header_stamp:
                    if msg.__class__._has_header:
                        header = msg.header
                    else:
                        header = _get_header(msg, path)
                    
                    plot_stamp = header.stamp
                else:
                    plot_stamp = msg_stamp

                try:
                    y = eval('msg.' + path)
                except Exception:
                    continue

                x = (plot_stamp - self._timeline.start_stamp).to_sec()

                if path not in self._data:
                    self._data[path] = DataSet()

                self._data[path].add(x, y)

            loaded_indexes.add(index)
            
            # Notify listeners of progress
            for listener in self._progress_listeners:
                listener()

def _get_header(msg, path):
    fields = path.split('.')
    if len(fields) <= 1:
        return None

    parent_path = '.'.join(fields[:-1])

    parent = eval('msg.' + parent_path)
    
    for slot in parent.__slots__:
        subobj = getattr(parent, slot)
        if subobj is not None and hasattr(subobj, '_type') and getattr(subobj, '_type') == 'roslib/Header':
            return subobj

    return _get_header(msg, parent_path)

def _subdivide(left, right):
    if left == right:
        yield left
        return
    
    yield left
    yield right

    intervals = collections.deque([(left, right)])
    while True:
        try:
            (left, right) = intervals.popleft()
        except Exception:
            break

        mid = (left + right) / 2

        if right - left <= 1:
            continue

        yield mid

        if right - left <= 2:
            continue

        intervals.append((left, mid))
        intervals.append((mid,  right))
