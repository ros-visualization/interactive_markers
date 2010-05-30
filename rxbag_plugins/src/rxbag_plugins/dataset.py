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

import bisect

class DataSet(object):
    def __init__(self):
        self._points = []
        
        self._num_points = 0

        # The range of the data
        self._min_x = None
        self._max_x = None
        self._min_y = None
        self._max_y = None

        # The range of intervals between successive points
        self._min_dx = None
        self._max_dx = None
        self._min_dy = None
        self._max_dy = None

    @property
    def points(self): return self._points

    @property
    def num_points(self): return self._num_points

    @property
    def min_x(self): return self._min_x

    @property
    def max_x(self): return self._max_x

    @property
    def min_y(self): return self._min_y

    @property
    def max_y(self): return self._max_y

    @property
    def min_dx(self): return self._min_dx

    @property
    def max_dx(self): return self._max_dx

    @property
    def min_dy(self): return self._min_dy

    @property
    def max_dy(self): return self._max_dy

    def add(self, x, y):
        pt = (x, y)

        index = bisect.bisect_right(self._points, pt)

        self._points.insert(index, pt)

        self._num_points += 1

        # Update the min and max
        if self._num_points == 1:
            self._min_x = x
            self._max_x = x
            self._min_y = y
            self._max_y = y
            return

        if x < self._min_x:
            self._min_x = x
        elif x > self._max_x:
            self._max_x = x
        if y < self._min_y:
            self._min_y = y
        elif y > self._max_y:
            self._max_y = y
        
        # Update min_dx and min_dy (the range of intervals between successive data points)
        if self._num_points == 2:
            dx, dy = self._points[1][0] - self._points[0][0], self._points[1][1] - self._points[0][1]
            self._min_dx, self._max_dx, self._min_dy, self._max_dy = dx, dx, dy, dy
        else:
            if index > 0:
                prev_pt = self._points[index - 1]
                dx, dy = x - prev_pt[0], y - prev_pt[1]
                if dx < self._min_dx:
                    self._min_dx = dx
                elif dx > self._max_dx:
                    self._max_dx = dx
                if dy < self._min_dy:
                    self._min_dy = dy
                elif dy > self._max_dy:
                    self._max_dy = dy

            if index < len(self._points) - 1:
                next_pt = self._points[index + 1]
                dx, dy = next_pt[0] - x, next_pt[1] - y
                if dx < self._min_dx:
                    self._min_dx = dx
                elif dx > self._max_dx:
                    self._max_dx = dx
                if dy < self._min_dy:
                    self._min_dy = dy
                elif dy > self._max_dy:
                    self._max_dy = dy
