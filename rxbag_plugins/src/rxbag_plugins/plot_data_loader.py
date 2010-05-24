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

PKG = 'rxbag_plugins'
import roslib; roslib.load_manifest(PKG)
import rospy

import bisect
import collections
import sys
import threading
import time

class PlotDataLoader(threading.Thread):
    def __init__(self, plot, topic):
        threading.Thread.__init__(self)

        self.plot  = plot
        self.topic = topic
        
        self.stop_flag = False

        self._view_entries = None
        self._loaded = set()
        self._data = {}

        self.set_view_region(self.plot.timeline.start_stamp.to_sec(), self.plot.timeline.end_stamp.to_sec())

        self.setDaemon(True)
        self.start()

    def set_view_region(self, start_stamp, end_stamp):
        self.start_stamp = start_stamp
        self.end_stamp   = end_stamp

        self.view_region_dirty = True

    def run(self):
        while not self.stop_flag:
            if self.view_region_dirty:
                self._view_entries = list(self.plot.timeline.get_entries(self.topic, rospy.Time.from_sec(self.start_stamp), rospy.Time.from_sec(self.end_stamp)))
                subdivider = self.subdivide(0, len(self._view_entries) - 1)
                self.view_region_dirty = False

            try:
                index = subdivider.next()
            except StopIteration:
                break

            if index in self._loaded:
                continue
            
            stamp = self._view_entries[index].time.to_sec()
            
            t = rospy.Time.from_sec(stamp)

            with self.plot.timeline._bag_lock:
                bag, entry = self.plot.timeline.get_entry(t, self.topic)
                if entry is None:
                    continue
                
                (topic, msg, msg_stamp) = self.plot.timeline.read_message(bag, entry.position)
                if not msg:
                    continue

            use_header_stamp = False

            series_index = 0
            for plot in self.plot.plot_paths:
                for plot_path in plot:
                    if use_header_stamp:
                        if msg.__class__._has_header:
                            header = msg.header
                        else:
                            header = PlotDataLoader.get_header(msg, plot_path)
                        
                        plot_stamp = header.stamp.to_sec()
                    else:
                        plot_stamp = stamp

                    value = eval('msg.' + plot_path)

                    x = plot_stamp - self.plot.timeline.start_stamp.to_sec()
                    y = value

                    if plot_path not in self._data:
                        self._data[plot_path] = []

                    bisect.insort_right(self._data[plot_path], (x, y))

                    series_index += 1

            self._loaded.add(index)

            if len(self._loaded) % 20 == 0:
                self.plot.invalidate()
                time.sleep(0.01)
            
            if len(self._loaded) == len(self._view_entries):
                print 'Finished loading.'
                break

    def stop(self):
        self.stop_flag = True
        self.join()

    @staticmethod
    def get_header(msg, path):
        fields = path.split('.')
        if len(fields) <= 1:
            return None
        
        parent_path = '.'.join(fields[:-1])

        parent = eval('msg.' + parent_path)
        
        for slot in parent.__slots__:
            subobj = getattr(parent, slot)
            if subobj is not None and hasattr(subobj, '_type') and getattr(subobj, '_type') == 'roslib/Header':
                return subobj

        return PlotDataLoader.get_header(msg, parent_path)

    @staticmethod
    def subdivide(left, right):
        yield left
        yield right

        intervals = collections.deque([(left, right)])
        while True:
            (left, right) = intervals.popleft()
            
            mid = (left + right) / 2
            
            intervals.append((left, mid))
            intervals.append((mid,  right))
            
            yield mid
