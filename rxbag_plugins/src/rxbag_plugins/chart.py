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

        self.Bind(wx.EVT_PAINT,       self._on_paint)
        self.Bind(wx.EVT_SIZE,        self._on_size)
        """
        self.Bind(wx.EVT_LEFT_DOWN,   self.on_left_down)
        self.Bind(wx.EVT_MIDDLE_DOWN, self.on_middle_down)
        self.Bind(wx.EVT_RIGHT_DOWN,  self.on_right_down)
        self.Bind(wx.EVT_LEFT_UP,     self.on_left_up)
        self.Bind(wx.EVT_MIDDLE_UP,   self.on_middle_up)
        self.Bind(wx.EVT_RIGHT_UP,    self.on_right_up)
        self.Bind(wx.EVT_MOTION,      self.on_mouse_move)
        self.Bind(wx.EVT_MOUSEWHEEL,  self.on_mousewheel)
        """

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
        self._margin_top    =  6
        self._margin_bottom =  2
        
        self._tick_length        = 4
        self._tick_label_padding = 10
        
        self._grid_x      = 1.0
        self._grid_y      = 0.5
        self._show_lines  = True
        self._show_points = False

        self._show_x_ticks = True

        self._zoom_interval = None

        self._layout()

        self._data = {}

        self._update_ranges()

    @property
    def view_range_x(self): return self.view_max_x - self.view_min_x
    
    @property
    def view_min_x(self):
        if self._zoom_interval:
            return self._zoom_interval[0]
            #return min(self._zoom_interval[1], max(self._zoom_interval[0], self._min_x))
        return self._min_x

    @property
    def view_max_x(self):
        if self._zoom_interval:
            return self._zoom_interval[1]
            #return min(self._zoom_interval[1], max(self._zoom_interval[0], self._max_x))
        return self._max_x

    @property
    def view_range_y(self): return self.view_max_y - self.view_min_y
    @property
    def view_min_y(self): return self._min_y
    @property
    def view_max_y(self): return self._max_y
    
    # show_x_ticks

    def _get_show_x_ticks(self):
        return self._show_x_ticks
    
    def _set_show_x_ticks(self, show_x_ticks):
        self._show_x_ticks = show_x_ticks
        self._layout()

    show_x_ticks = property(_get_show_x_ticks, _set_show_x_ticks)

    # zoom_interval
    
    def _get_zoom_interval(self):
        return self._zoom_interval
    
    def _set_zoom_interval(self, zoom_interval):
        self._zoom_interval = zoom_interval
        
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

    ## Implementation
    
    def _get_x_ticks(self, dc):
        if self.view_min_x == self.view_max_x:
            return None
        
        min_x_width = dc.text_extents(self.format_x(self.view_min_x))[2]
        max_x_width = dc.text_extents(self.format_x(self.view_max_x))[2]
        max_label_width = max(min_x_width, max_x_width) + self._tick_label_padding

        num_ticks = self.chart_width / max_label_width
        
        return self.view_range_x / num_ticks

    def _get_y_ticks(self, dc):
        if self.view_min_y == self.view_max_y:
            return None

        label_height = dc.font_extents()[2] + self._tick_label_padding
        
        num_ticks = self.chart_height / label_height
        
        return self.view_range_y / num_ticks

    def format_x(self, x):
        return '%.2f' % x

    def format_y(self, y):
        return '%.2f' % y

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
            min_x_grid = self.view_min_x + ((self.view_min_x + self._grid_x) % self._grid_x)
            dc.set_source_rgba(0, 0, 0, 0.2)
            self._draw_lines(dc, self._generate_lines_x(min_x_grid, self.view_max_x, self._grid_x))
        if self.view_min_y != self.view_max_y:
            min_y_grid = self.view_min_y + ((self.view_min_y + self._grid_y) % self._grid_y)
            dc.set_source_rgba(0, 0, 0, 0.2)
            self._draw_lines(dc, self._generate_lines_y(min_y_grid, self.view_max_y, self._grid_y))

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
            x_tick = self._get_x_ticks(dc)
            if self.view_min_x != self.view_max_x:
                min_x_tick = self.view_min_x + ((self.view_min_x + x_tick) % x_tick)
                lines = list(self._generate_lines_x(min_x_tick, self.view_max_x, x_tick, self.chart_bottom, self.chart_bottom + self._tick_length))
    
                dc.set_source_rgba(0, 0, 0, 1)
                self._draw_lines(dc, lines)
    
                for x0, y0, x1, y1 in lines:
                    s = self.format_x(self.x_chart_to_data(x0))
                    text_width, text_height = dc.text_extents(s)[2:4]
                    dc.move_to(x0 - text_width / 2, y1 + 3 + text_height)
                    dc.show_text(s)

        y_tick = self._get_y_ticks(dc)
        if self.view_min_y != self.view_max_y:
            min_y_tick = self.view_min_y + ((self.view_min_y + y_tick) % y_tick)
            lines = list(self._generate_lines_y(min_y_tick, self.view_max_y, y_tick, self.chart_left - self._tick_length, self.chart_left))

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
