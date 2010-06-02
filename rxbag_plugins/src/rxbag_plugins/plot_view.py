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
import csv
import sys
import time

import Image
import numpy
import pylab
import wx
import wx.lib.wxcairo

from rxbag import TopicMessageView

from chart import Chart
from plot_configure_frame import PlotConfigureFrame
from plot_data_loader     import PlotDataLoader
import image_helper

class PlotView(TopicMessageView):
    name = 'Plot'

    periods = [(  -1, 'All'),
               ( 0.1, '0.1 s'),
               ( 0.2, '0.2 s'),
               ( 0.5, '0.5 s'),
               (   1, '1 s'),
               (   2, '2 s'),
               (   5, '5 s'),
               (  10, '10 s'),
               (  15, '15 s'),
               (  30, '30 s'),
               (  60, '60 s'),
               (  90, '90 s'),
               ( 120, '2 m'),
               ( 300, '5 m'),
               ( 600, '10 m'),
               (1200, '20 m'),
               (1800, '30 m'),
               (3600, '1 h')]

    rows = [(   1, 'All'),
            (   2, 'Every 2nd message'),
            (   3, 'Every 3rd message'),
            (   4, 'Every 4th message'),
            (   5, 'Every 5th message'),
            (  10, 'Every 10th message'),
            (  20, 'Every 20th message'),
            (  50, 'Every 50th message'),
            ( 100, 'Every 100th message'),
            (1000, 'Every 1000th message')]

    def __init__(self, timeline, parent):
        TopicMessageView.__init__(self, timeline, parent)

        self._topic         = None
        self._message       = None
        self._plot_paths    = []
        self._period        = -1
        self._playhead      = None
        self._charts        = []
        self._data_loader   = None
        self._zoom_interval = None

        self._configure_frame = None

        self._max_interval_secs = 1.5

        tb = self.parent.GetToolBar()
        icons_dir = roslib.packages.get_pkg_dir(PKG) + '/icons/'
        tb.AddSeparator()
        tb.Bind(wx.EVT_TOOL, lambda e: self.configure(), tb.AddLabelTool(wx.ID_ANY, '', wx.Bitmap(icons_dir + 'cog.png')))

        self.parent.Bind(wx.EVT_SIZE,       self._on_size)
        self.parent.Bind(wx.EVT_PAINT,      self._on_paint)
        self.parent.Bind(wx.EVT_RIGHT_DOWN, self._on_right_down)
        self.parent.Bind(wx.EVT_MOUSEWHEEL, self._on_mousewheel)
        self.parent.Bind(wx.EVT_CLOSE,      self._on_close)

        wx.CallAfter(self.configure)

    # property: plot_paths
    
    def _get_plot_paths(self): return self._plot_paths
    
    def _set_plot_paths(self, plot_paths):
        self._plot_paths = plot_paths

        # Update the data loader with the paths to plot
        if self._data_loader:
            paths = []
            for plot in self._plot_paths:
                for path in plot:
                    if path not in paths:
                        paths.append(path)

            self._data_loader.paths = paths

        self._setup_charts()

    plot_paths = property(_get_plot_paths, _set_plot_paths)

    ## TopicMessageView implementation

    def message_viewed(self, bag, msg_details):
        TopicMessageView.message_viewed(self, bag, msg_details)

        topic, msg, t = msg_details

        if not self._data_loader:
            self._topic = topic
            self.start_loading()

        self._message = msg

        self.playhead = (t - self.timeline.start_stamp).to_sec()

    def message_cleared(self):
        self._message = None
        
        TopicMessageView.message_cleared(self)
        
        wx.CallAfter(self.parent.Refresh)
        
    def timeline_changed(self):
        self._update_view_region()

    # property: period

    def _get_period(self): return self._period
    
    def _set_period(self, period):
        self._period = period        
        self._update_view_region()

    period = property(_get_period, _set_period)

    # property: playhead

    def _get_playhead(self): return self._playhead
    
    def _set_playhead(self, playhead):
        self._playhead = playhead
        self._update_view_region()

    playhead = property(_get_playhead, _set_playhead)

    def _update_view_region(self):
        if not self._data_loader:
            return
        
        if self._period < 0:
            start_stamp = self.timeline.start_stamp
            end_stamp   = self.timeline.end_stamp
        else:
            start_stamp = self.timeline.start_stamp + rospy.Duration.from_sec(self._playhead - (self._period / 2))
            end_stamp   = start_stamp + rospy.Duration.from_sec(self._period)

        self._data_loader.start_stamp = start_stamp
        self._data_loader.end_stamp   = end_stamp
        
        self._update_max_interval()

        self._zoom_interval = ((start_stamp - self.timeline.start_stamp).to_sec(),
                               (end_stamp   - self.timeline.start_stamp).to_sec())

        wx.CallAfter(self.parent.Refresh)

    def _update_max_interval(self):
        secs_per_px = (self._data_loader.end_stamp - self._data_loader.start_stamp).to_sec() / self.parent.Size[0]  # conservative: use entire width of control instead of just plot area

        self._data_loader.max_interval = secs_per_px * self._max_interval_secs

    ## Events

    def _on_paint(self, event):
        if not self._data_loader or len(self._charts) == 0:
            return

        dc = wx.lib.wxcairo.ContextFromDC(wx.PaintDC(self.parent))
        
        dc.set_source_rgb(1, 1, 1)
        dc.rectangle(0, 0, self.parent.Size[0], self.parent.Size[1])
        dc.fill()

        data = {}

        chart_height = self.parent.GetClientSize()[1] / len(self._charts)

        dc.save()

        for chart_index, plot in enumerate(self._plot_paths):
            chart = self._charts[chart_index]

            chart.zoom_interval = self._zoom_interval
            if self._message:
                chart.x_indicator = self._playhead
            else:
                chart.x_indicator = None

            data = {}
            for plot_path in plot:
                if plot_path in self._data_loader._data:
                    data[plot_path] = self._data_loader._data[plot_path]
            chart._series_list = plot
            chart._series_data = data

            chart.paint(dc)
            dc.translate(0, chart_height)

        dc.restore()

    def _on_size(self, event):
        self._layout_charts()

        self._update_max_interval()

    def _on_right_down(self, event):
        self.clicked_pos = event.GetPosition()
        self.parent.PopupMenu(PlotPopupMenu(self.parent, self), self.clicked_pos)

    def _on_mousewheel(self, event):
        dz = event.GetWheelRotation() / event.GetWheelDelta()

        index = None
        for i, (period, _) in enumerate(self.periods):
            if period == self._period:
                index = i
                break

        if dz < 0:
            self.period = self.periods[min(len(self.periods) - 1, index + 1)][0]
        elif dz > 0:
            self.period = self.periods[max(1, index - 1)][0]

        wx.CallAfter(self.parent.Refresh)

    def _on_close(self, event):
        if self._configure_frame:
            self._configure_frame.Close()

        self.stop_loading()

        event.Skip()

    ##

    def _setup_charts(self):
        self._charts = []
        
        palette_offset = 0
        for i, plot in enumerate(self._plot_paths):
            chart = Chart()

            chart.palette_offset = palette_offset
            palette_offset += len(plot)
            
            if i < len(self._plot_paths) - 1:
                chart.show_x_ticks = False

            self._charts.append(chart)

        self._layout_charts()

    def _layout_charts(self):
        w, h = self.parent.GetClientSize()
        num_charts = len(self._charts)
        if num_charts > 0:
            chart_height = h / num_charts
            for chart in self._charts:
                chart.set_size(w, chart_height)

        wx.CallAfter(self.parent.Refresh)

    def stop_loading(self):
        if self._data_loader:
            self._data_loader.stop()
            self._data_loader = None

    def start_loading(self):
        if self._topic and not self._data_loader:
            self._data_loader = PlotDataLoader(self.timeline, self._topic)
            self._data_loader.add_progress_listener(self._data_loader_updated)

    def _data_loader_updated(self):
        wx.CallAfter(self.parent.Refresh)

    def configure(self):
        if self._configure_frame:
            return
        
        if self._message:
            self._configure_frame = PlotConfigureFrame(self)
            
            frame = self.parent.GetTopLevelParent()
            self._configure_frame.SetPosition((frame.Position[0] + frame.Size[0] + 10, frame.Position[1]))
            self._configure_frame.Show()

    def export_csv(self, rows):
        dialog = wx.FileDialog(self.parent.GetParent(), 'Export to CSV...', wildcard='CSV files (*.csv)|*.csv', style=wx.FD_SAVE)
        if dialog.ShowModal() == wx.ID_OK:
            csv_path = dialog.GetPath()
    
            export_series = set()
            for plot in self._plot_paths:
                for path in plot:
                    export_series.add(path)
    
            self._data_loader.export_csv(csv_path, export_series, self._zoom_interval[0], self._zoom_interval[1], rows)

        dialog.Destroy()

    # @todo
    def do_export_csv(self, path, series_list, x_min, x_max, rows):
        plot_loader = PlotDataLoader(self._topic)
        
        # Collate data
        i = 0
        series_dict = {}
        unique_stamps = set()
        for series in series_list:
            d = {}
            series_dict[series] = d

            point_num = 0
            for x, y in self. data[series].points:
                if x >= x_min and x <= x_max:
                    if point_num % rows == 0:
                        d[x] = y
                        unique_stamps.add(x)
                    point_num += 1
            i += 1
        series_columns = sorted(series_dict.keys())

        try:
            csv_writer = csv.DictWriter(open(path, 'w'), ['Timestamp'] + series_columns)
 
            # Write header row
            header_dict = { 'Timestamp' : 'Timestamp' }
            for column in series_columns:
                header_dict[column] = column            
            csv_writer.writerow(header_dict)

            # Write data
            for stamp in sorted(unique_stamps):
                row = { 'Timestamp' : stamp }
                for column in series_dict:
                    if stamp in series_dict[column]:
                        row[column] = series_dict[column][stamp]
 
                csv_writer.writerow(row)

        except Exception, ex:
            print >> sys.stderr, 'Error writing to CSV file: %s' % str(ex)
        
    def export_image(self):
        dialog = wx.FileDialog(self.parent.GetParent(), 'Save plot to...', wildcard='PNG files (*.png)|*.png', style=wx.FD_SAVE)
        if dialog.ShowModal() == wx.ID_OK:
            path = dialog.GetPath()

            bitmap = wx.EmptyBitmap(self.width, self.height)
            mem_dc = wx.MemoryDC()
            mem_dc.SelectObject(bitmap)
            mem_dc.SetBackground(wx.WHITE_BRUSH)
            mem_dc.Clear()
            cairo_dc = wx.lib.wxcairo.ContextFromDC(mem_dc)
            self.paint(cairo_dc)
            mem_dc.SelectObject(wx.NullBitmap)

            bitmap.SaveFile(path, wx.BITMAP_TYPE_PNG)
            
        dialog.Destroy()

class PlotPopupMenu(wx.Menu):
    def __init__(self, parent, plot):
        wx.Menu.__init__(self)

        self.parent = parent
        self.plot   = plot

        # Configure...
        configure_item = wx.MenuItem(self, wx.NewId(), 'Configure...')
        self.AppendItem(configure_item)
        self.Bind(wx.EVT_MENU, lambda e: self.plot.configure(), id=configure_item.Id)

        # Interval...
        self.interval_menu = wx.Menu()
        self.AppendSubMenu(self.interval_menu, 'Interval...', 'Timeline interval to plot')

        for period, label in plot.periods:
            period_item = self.PeriodMenuItem(self.interval_menu, wx.NewId(), label, period, plot)
            self.interval_menu.AppendItem(period_item)
            period_item.Check(plot.period == period_item.period)
            
            if label == 'All':
                self.interval_menu.AppendSeparator()

        # Export to PNG...
        export_image_item = wx.MenuItem(self, wx.NewId(), 'Export to PNG...')
        self.AppendItem(export_image_item)
        self.Bind(wx.EVT_MENU, lambda e: self.plot.export_image(), id=export_image_item.Id)
        
        # Export to CSV...
        self.export_csv_menu = wx.Menu()
        self.AppendSubMenu(self.export_csv_menu, 'Export to CSV...', 'Export data to CSV file')

        for rows, label in plot.rows:
            rows_item = self.ExportCSVMenuItem(self.export_csv_menu, wx.NewId(), label, rows, plot)
            self.export_csv_menu.AppendItem(rows_item)
            
            if label == 'All':
                self.export_csv_menu.AppendSeparator()

    class PeriodMenuItem(wx.MenuItem):
        def __init__(self, parent, id, label, period, plot):
            wx.MenuItem.__init__(self, parent, id, label, kind=wx.ITEM_CHECK)
            
            self.period = period
            self.plot   = plot

            parent.Bind(wx.EVT_MENU, self._on_menu, id=self.Id)
    
        def _on_menu(self, event):
            self.plot.period = self.period
            wx.CallAfter(self.plot.parent.Refresh)
    
    class ExportCSVMenuItem(wx.MenuItem):
        def __init__(self, parent, id, label, rows, plot):
            wx.MenuItem.__init__(self, parent, id, label)
            
            self.rows = rows
            self.plot = plot

            parent.Bind(wx.EVT_MENU, self._on_menu, id=self.Id)

        def _on_menu(self, event):
            self.plot.export_csv(self.rows)
