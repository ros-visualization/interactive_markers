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
import math
import sys
import threading
import time

import cairo
import wx
import wx.lib.wxcairo

class ChartWindow(wx.Window):
    def __init__(self, *args, **kwargs):
        wx.Window.__init__(self, *args, **kwargs)
        
        self.background_brush = wx.WHITE_BRUSH

        self._chart = Chart()

        self.Bind(wx.EVT_PAINT, self._on_paint)
        self.Bind(wx.EVT_SIZE,  self._on_size)

    def _on_size(self, event):
        self._chart.set_size(*self.ClientSize)
        self.Refresh()

    def _on_paint(self, event):
        window_dc = wx.PaintDC(self)
        window_dc.SetBackground(self.background_brush)
        window_dc.Clear()

        dc = wx.lib.wxcairo.ContextFromDC(window_dc)
        self._chart.paint(dc)

class Chart(object):
    def __init__(self):
        self._lock = threading.RLock()

        self._width  = 400
        self._height = 400

        self._margin_left   = 50
        self._margin_right  = 10
        self._margin_top    =  8
        self._margin_bottom =  2
        
        self._tick_length        = 4
        self._tick_label_padding = 30
        
        self._show_lines  = False
        self._show_points = True

        self._x_interval   = None
        self._y_interval   = None
        self._show_x_ticks = True

        # The range of the data
        self._min_x = None
        self._max_x = None
        self._min_y = None
        self._max_y = None

        # The desired viewport
        self._x_zoom = None
        self._y_zoom = None

        # The displayed viewport (takes into account interval rounding)
        self._x_view = None
        self._y_view = None

        self._layout()

        self._data = {}

        self._update_ranges()

    @property
    def view_range_x(self): return self.view_max_x - self.view_min_x
    
    @property
    def view_min_x(self): return self._x_view[0]

    @property
    def view_max_x(self): return self._x_view[1]

    @property
    def view_range_y(self): return self.view_max_y - self.view_min_y

    @property
    def view_min_y(self): return self._y_view[0]
    
    @property
    def view_max_y(self): return self._y_view[1]
    
    # show_x_ticks

    def _get_show_x_ticks(self):
        return self._show_x_ticks
    
    def _set_show_x_ticks(self, show_x_ticks):
        self._show_x_ticks = show_x_ticks
        self._layout()

    show_x_ticks = property(_get_show_x_ticks, _set_show_x_ticks)

    # zoom_interval
    
    def _get_zoom_interval(self):
        return self._x_zoom

    def _set_zoom_interval(self, zoom_interval):
        self._x_zoom = zoom_interval

    zoom_interval = property(_get_zoom_interval, _set_zoom_interval)

    ## Data

    def add_datum(self, series, x, y):
        with self._lock:
            if series not in self._data:
                self._data[series] = []
            bisect.insort_right(self._data[series], (x, y))

    def clear(self):
        with self._lock:
            self._data = {}
            self._update_ranges()

    def set_size(self, width, height):
        self._width, self._height = width, height
        self._layout()
    
    @property
    def chart_right(self): return self.chart_left + self.chart_width

    @property
    def chart_bottom(self): return self.chart_top + self.chart_height

    ## Coordinate transformations

    def coord_data_to_chart(self, x, y): return self.x_data_to_chart(x), self.y_data_to_chart(y)
    def x_data_to_chart(self, x):        return self.chart_left   + (x - self.view_min_x) / self.view_range_x * self.chart_width
    def y_data_to_chart(self, y):        return self.chart_bottom - (y - self.view_min_y) / self.view_range_y * self.chart_height
    def dx_data_to_chart(self, dx):      return dx / self.view_range_x * self.chart_width
    def dy_data_to_chart(self, dy):      return dy / self.view_range_y * self.chart_height

    def x_chart_to_data(self, x):        return self.view_min_x + (x - self.chart_left)   / self.chart_width  * self.view_range_x
    def y_chart_to_data(self, y):        return self.view_min_y + (self.chart_bottom - y) / self.chart_height * self.view_range_y
    
    def format_x(self, x):
        if self._x_interval is None:
            return '%.3f' % x
        
        dp = max(0, int(math.ceil(-math.log10(self._x_interval))))
        return '%.*f' % (dp, x)
    
    def format_y(self, y):
        if self._y_interval is None:
            return '%.3f' % y

        dp = max(0, int(math.ceil(-math.log10(self._y_interval))))
        return '%.*f' % (dp, y)

    ## Implementation

    @property
    def x_zoom(self):
        if self._x_zoom is None:
            return (self._min_x, self._max_x)
        else:
            return self._x_zoom

    @property
    def y_zoom(self):
        if self._y_zoom is None:
            return (self._min_y, self._max_y)
        else:
            return self._y_zoom

    def _update_axes(self, dc):
        """
        Calculates intervals and viewport.
        """
        self._update_x_interval(dc)
        self._update_y_interval(dc)

        self._x_view = (self._round_min_to_interval(self.x_zoom[0], self._x_interval),
                        self._round_max_to_interval(self.x_zoom[1], self._x_interval))

        self._y_view = (self._round_min_to_interval(self.y_zoom[0], self._y_interval),
                        self._round_max_to_interval(self.y_zoom[1], self._y_interval))
        
        #print 'min_x:', self._min_x, 'zoom_x[0]:', self.x_zoom[0], 'view_x[0]:', self._x_view[0]
        #print 'max_x:', self._max_x, 'zoom_x[1]:', self.x_zoom[1], 'view_x[1]:', self._x_view[1]
        #print 'min_y:', self._min_y, 'zoom_y[0]:', self.y_zoom[0], 'view_y[0]:', self._y_view[0]
        #print 'max_y:', self._max_y, 'zoom_y[1]:', self.y_zoom[1], 'view_y[1]:', self._y_view[1]
        #print

    def _update_x_interval(self, dc):
        min_x_width = dc.text_extents(self.format_x(self.x_zoom[0]))[2]
        max_x_width = dc.text_extents(self.format_x(self.x_zoom[1]))[2]
        max_label_width = max(min_x_width, max_x_width) * 2 + self._tick_label_padding

        num_ticks = self.chart_width / max_label_width

        self._x_interval = self._get_axis_interval((self.x_zoom[1] - self.x_zoom[0]) / num_ticks)

    def _update_y_interval(self, dc):
        label_height = dc.font_extents()[2] + self._tick_label_padding
        
        num_ticks = self.chart_height / label_height

        self._y_interval = self._get_axis_interval((self.y_zoom[1] - self.y_zoom[0]) / num_ticks)

    def _get_axis_interval(self, range, intervals=[1.0, 2.0, 5.0]):
        exp = -8
        found = False
        prev_threshold = None
        while True:
            multiplier = pow(10, exp)
            for interval in intervals:
                threshold = multiplier * interval
                if threshold > range:
                    return prev_threshold
                prev_threshold = threshold
            exp += 1

    def _round_min_to_interval(self, min_val, interval, extend_touching=False):
        rounded = interval * math.floor(min_val / interval)
        if min_val > rounded:
            return rounded
        if extend_touching:
            return min_val - interval
        return min_val

    def _round_max_to_interval(self, max_val, interval, extend_touching=False):
        rounded = interval * math.ceil(max_val / interval)
        if max_val < rounded:
            return rounded
        if extend_touching:
            return max_val + interval
        return max_val

    def _layout(self):
        self.chart_left   = self._margin_left
        self.chart_top    = self._margin_top
        self.chart_width  = self._width  - self._margin_left - self._margin_right
        self.chart_height = self._height - self._margin_top  - self._margin_bottom

        if self._show_x_ticks:
            self.chart_height -= 18

    def _update_ranges(self):
        with self._lock:
            min_x, max_x, min_y, max_y = None, None, None, None
            if any(self._data):
                x, y = self._data.values()[0][0]
                min_x, max_x, min_y, max_y = x, x, y, y

                for series, data in self._data.items():
                    for x, y in data:
                        if x < min_x:
                            min_x = x
                        elif x > max_x:
                            max_x = x
                        if y < min_y:
                            min_y = y
                        elif y > max_y:
                            max_y = y

            self._min_x, self._max_x, self._min_y, self._max_y = min_x, max_x, min_y, max_y

    def paint(self, dc):
        self._draw_border(dc)

        dc.save()
        dc.rectangle(self.chart_left, self.chart_top, self.chart_width, self.chart_height)
        dc.clip()
        
        self._update_axes(dc)

        self._draw_grid(dc)
        self._draw_axes(dc)
        with self._lock:
            self._draw_data(dc)

        dc.restore()

        self._draw_ticks(dc)

    def _draw_border(self, dc):
        dc.set_antialias(cairo.ANTIALIAS_NONE)
        dc.set_line_width(1.0)
        dc.set_source_rgba(0, 0, 0, 0.8)
        dc.rectangle(self.chart_left, self.chart_top - 1, self.chart_width, self.chart_height + 1)
        dc.stroke()

    def _draw_grid(self, dc):
        dc.set_antialias(cairo.ANTIALIAS_NONE)
        dc.set_line_width(1.0)
        dc.set_dash([2, 4])
        
        if self.view_min_x != self.view_max_x:
            dc.set_source_rgba(0, 0, 0, 0.2)
            self._draw_lines(dc, self._generate_lines_x(self.view_min_x, self.view_max_x, self._x_interval))
        if self.view_min_y != self.view_max_y:
            dc.set_source_rgba(0, 0, 0, 0.2)
            self._draw_lines(dc, self._generate_lines_y(self.view_min_y, self.view_max_y, self._y_interval))

        dc.set_dash([])

    def _draw_axes(self, dc):
        dc.set_antialias(cairo.ANTIALIAS_NONE)
        dc.set_line_width(1.0)
        dc.set_source_rgba(0, 0, 0, 0.3)
        
        if self.view_min_y != self.view_max_y:
            x_intercept = self.y_data_to_chart(0.0)
            dc.move_to(self.chart_left,  x_intercept)
            dc.line_to(self.chart_right, x_intercept)
            dc.stroke()
        
        if self.view_min_x != self.view_max_x:
            y_intercept = self.x_data_to_chart(0.0)
            dc.move_to(y_intercept, self.chart_bottom)
            dc.line_to(y_intercept, self.chart_top)
            dc.stroke()

    def _draw_ticks(self, dc):
        dc.set_antialias(cairo.ANTIALIAS_NONE)
        dc.set_line_width(1.0)

        if self._show_x_ticks:
            if self.view_min_x != self.view_max_x:
                lines = list(self._generate_lines_x(self.view_min_x, self.view_max_x + self._x_interval, self._x_interval, self.chart_bottom, self.chart_bottom + self._tick_length))
    
                dc.set_source_rgba(0, 0, 0, 1)
                self._draw_lines(dc, lines)
    
                for x0, y0, x1, y1 in lines:
                    s = self.format_x(self.x_chart_to_data(x0))
                    text_width, text_height = dc.text_extents(s)[2:4]
                    dc.move_to(x0 - text_width / 2, y1 + 3 + text_height)
                    dc.show_text(s)

        if self.view_min_y != self.view_max_y:
            lines = list(self._generate_lines_y(self.view_min_y, self.view_max_y + self._y_interval, self._y_interval, self.chart_left - self._tick_length, self.chart_left))

            dc.set_source_rgba(0, 0, 0, 1)
            self._draw_lines(dc, lines)

            for x0, y0, x1, y1 in lines:
                s = self.format_y(self.y_chart_to_data(y0))
                text_width, text_height = dc.text_extents(s)[2:4]
                dc.move_to(x0 - text_width - 3, y0 + text_height / 2)
                dc.show_text(s)

    def _draw_lines(self, dc, lines):
        for x0, y0, x1, y1 in lines:
            dc.move_to(x0, y0)
            dc.line_to(x1, y1)
        dc.stroke()

    def _generate_lines_x(self, x0, x1, x_step, py0=None, py1=None):
        px0 = self.x_data_to_chart(x0)
        px1 = self.x_data_to_chart(x1)
        if py0 is None:
            py0 = self.y_data_to_chart(self.view_min_y)
        if py1 is None:
            py1 = self.y_data_to_chart(self.view_max_y)
        px_step = self.dx_data_to_chart(x_step)

        px = px0
        while True:
            yield px, py0, px, py1
            px += px_step
            if px >= px1:
                break

    def _generate_lines_y(self, y0, y1, y_step, px0=None, px1=None):
        py0 = self.y_data_to_chart(y0)
        py1 = self.y_data_to_chart(y1)
        if px0 is None:
            px0 = self.x_data_to_chart(self.view_min_x)
        if px1 is None:
            px1 = self.x_data_to_chart(self.view_max_x)
        py_step = self.dy_data_to_chart(y_step)
        
        py = py0
        while True:
            yield px0, py, px1, py
            py -= py_step
            if py <= py1:
                break

    def _draw_data(self, dc):
        if not any(self._data):
            return

        # @todo: handle min_x = max_x / min_y = max_y cases
        
        palette = [(0,   0,   0.8),
                   (0,   0.8, 0),
                   (0.8, 0,   0)]

        dc.set_antialias(cairo.ANTIALIAS_SUBPIXEL)
        
        for i, (series, series_data) in enumerate(self._data.items()):
            dc.set_source_rgb(*palette[i % len(palette)])

            coords = [self.coord_data_to_chart(x, y) for x, y in series_data]

            # Draw lines
            if self._show_lines:
                dc.set_line_width(1.0)
                dc.move_to(*coords[0])
                for px, py in coords:
                    dc.line_to(px, py)
                dc.stroke()

            # Draw points
            if self._show_points:
                dc.set_line_width(1.5)
                for px, py in coords:
                    dc.move_to(px - 2, py - 2)
                    dc.line_to(px + 2, py + 2)
                    dc.move_to(px + 2, py - 2)
                    dc.line_to(px - 2, py + 2)
        
                dc.stroke()
