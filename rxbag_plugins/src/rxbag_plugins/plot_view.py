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
#
# Revision $Id$

PKG = 'rxbag_plugins'
import roslib; roslib.load_manifest(PKG)
import rospy

import rosbag

import collections
import string
import threading
import time

import rxtools.vizutil
rxtools.vizutil.check_matplotlib_deps()

import matplotlib
matplotlib.use('WXAgg')
from matplotlib.figure import Figure
from matplotlib.backends.backend_wxagg import FigureCanvasWxAgg as FigCanvas, NavigationToolbar2WxAgg as NavigationToolbar

import numpy
import pylab
import wx

from rxbag import TopicMessageView

from plot_configure_frame import PlotConfigureFrame

class PlotView(TopicMessageView):
    name = 'Plot'

    def __init__(self, timeline, parent, title, x, y, width, height, max_repaint=0.1):
        TopicMessageView.__init__(self, timeline, parent, title, x, y, width, height, max_repaint)

        self.bag_file  = None
        self.bag_index = None
        self.topic     = None

        # Input data
        self.series_list = []
        self.datax       = []
        self.datay       = []
        self.playhead    = 0
        
        self._msg = None
        
        self.plot_paths = []

        # View parameters
        self.period = -1
        self.marker = 'o'

        # Drawing objects
        self._axes          = None
        self._series_data   = None
        self._playhead_line = None

        # Create window, figure, canvas
        self._create_figure()

        self._init_plot([])
        
        self._data_thread = None

        # Toolbar
        tb = self.frame.GetToolBar()
        icons_dir = roslib.packages.get_pkg_dir(PKG) + '/icons/'
        tb.AddSeparator()
        tb.Bind(wx.EVT_TOOL, lambda e: self.configure(), tb.AddLabelTool(wx.ID_ANY, '', wx.Bitmap(icons_dir + 'cog.png')))

    def set_series_data(self, series, datax, datay):
        data = sorted([(datax[i], datay[i]) for i in range(len(datax))])

        self.datax[series] = [x for (x, y) in data]
        self.datay[series] = [y for (x, y) in data]

        self.invalidate()

    def set_playhead(self, playhead):
        self.playhead = playhead
        
        self.invalidate()

    def message_viewed(self, bag_file, msg_details):
        TopicMessageView.message_viewed(self, bag_file, msg_details)

        topic, msg, t = msg_details

        if not self._data_thread:
            self.bag_file, self.topic = bag_file, topic
            self.start_loading()

        self._msg = msg

        self.set_playhead(t.to_sec() - self.timeline.start_stamp)

    def message_cleared(self):
        TopicMessageView.message_cleared(self)

    def _create_figure(self):
        self._window = wx.Window(self.parent, -1, (self._x, self._y), (self._width, self._height))
        self._window.Show(False)

        self.dpi = 100
        figsize = (float(self.width) / self.dpi, float(self.height) / self.dpi)  # 1px border
        rc      = matplotlib.figure.SubplotParams(left=0.05, bottom=0.08, right=0.99, top=0.98, hspace=0.03)
        self.figure = Figure(figsize, dpi=self.dpi, subplotpars=rc, facecolor='w')

        self.canvas = FigCanvas(self._window, -1, self.figure)
        self.canvas.mpl_connect('pick_event', self.on_pick)

    def _init_plot(self, series_list):
        self.figure.clear()
        
        self.series_list = series_list

        self.series_data = []
        self.axes        = []

        fp = matplotlib.font_manager.FontProperties(size=9)
        
        for subplot_series in self.series_list:
            # Create axes for this plot
            first_axes = None
            if len(self.axes) > 0:
                first_axes = self.axes[0]
            axes = self.figure.add_subplot(len(self.series_list), 1, len(self.axes) + 1, sharex=first_axes)
            axes.set_axis_bgcolor('white')
            axes.grid(True, color='gray')
            pylab.setp(axes.get_xticklabels(), fontsize=9)
            pylab.setp(axes.get_yticklabels(), fontsize=9)
            self.axes.append(axes)

            # Create the series data
            if len(subplot_series) == 0:
                continue

            subplot_series_data = []
            
            for series in subplot_series:
                series_index = len(self.series_data)
                
                self.datax.append([])
                self.datay.append([])
                
                series_data = axes.plot(self.datax[series_index],
                                        self.datay[series_index],
                                        marker=self.marker,
                                        markersize=3,
                                        linewidth=1,
                                        picker=self.plot_picker)[0]
                                        
                subplot_series_data.append(series_data)

            self.series_data.extend(subplot_series_data)

            # ros:#2588 handle matplotlib's incompatible API's
            use_get_position = matplotlib.__version__ >= '0.98.3'
            if use_get_position:
                left   = (axes.get_position().xmin + 10) / self.figure.get_window_extent().width
                bottom = (axes.get_position().ymin + 10) / self.figure.get_window_extent().height 
            else:
                left   = (axes.left.get()   + 10) / self.figure.get_window_extent().width()
                bottom = (axes.bottom.get() + 10) / self.figure.get_window_extent().height()

            use_handlelength = matplotlib.__version__ >= '0.98.5.2'
            if use_handlelength:
                subplot_legend = self.figure.legend(subplot_series_data, subplot_series, loc=(left, bottom), prop=fp, handlelength=0.02, handletextpad=0.02, borderaxespad=0.0, labelspacing=0.002, borderpad=0.2)
            else:
                subplot_legend = self.figure.legend(subplot_series_data, subplot_series, loc=(left, bottom), prop=fp, handlelen=0.02, handletextsep=0.02, axespad=0.0, labelsep=0.002, pad=0.2)

            #subplot_legend.draw_frame(False)
            subplot_legend.set_axes(axes)

        # Hide the x tick labels for every subplot except for the last
        for ax in self.axes:
            pylab.setp(ax.get_xticklabels(), visible=(ax == self.axes[-1]))

    def paint(self, dc):
        # Draw plot (not visible)
        self._draw_plot(True)

        # todo
        dc.DrawBitmap(self.canvas.bitmap, 0, 0)

    def _draw_plot(self, relimit=False):
        if self.series_data and relimit and self.datax[0]:
            axes_index = 0
            plot_index = 0
            # axes are indexed by topic_list, plots are indexed by topic number
            for subplot_series in self.series_list:
                axes = self.axes[axes_index] 
                axes_index += 1
                
                xmin = xmax = None
                ymin = ymax = None
                for series in subplot_series:
                    datax = self.datax[plot_index]
                    datay = self.datay[plot_index]
                    plot_index += 1

                    if len(datax) == 0 or len(datay) == 0:
                        continue

                    if self.period < 0:
                        if xmin is None:
                            xmin = min(datax)
                            xmax = max(datax)
                        else:
                            xmin = min(min(datax), xmin)
                            xmax = max(max(datax), xmax)
                    else:
                        xmax = self.playhead + (self.period / 2)
                        xmin = xmax - self.period

                    if ymin is None:
                        ymin = min(datay)
                        ymax = max(datay)
                    else:
                        ymin = min(min(datay), ymin)
                        ymax = max(max(datay), ymax)

                    # Pad the min/max
                    delta = ymax - ymin
                    ymin -= 0.1 * delta
                    ymax += 0.1 * delta
                    
                    axes.set_xbound(lower=xmin, upper=xmax)
                    axes.set_ybound(lower=ymin, upper=ymax)

        if self.series_data:
            for plot_index in xrange(0, len(self.series_data)):
                datax = self.datax[plot_index]
                datay = self.datay[plot_index]
    
                series_data = self.series_data[plot_index]
                series_data.set_data(numpy.array(datax), numpy.array(datay))

        self.canvas.draw()

    # Events

    def on_size(self, event):
        self.resize_figure()

    def resize_figure(self):
        w, h = self.parent.GetClientSize()

        self._window.SetSize((w, h))
        self.figure.set_size_inches((float(w) / self.dpi, float(h) / self.dpi))
        self.resize(w, h)

    def on_right_down(self, event):
        self.clicked_pos = event.GetPosition()

        if self.contains(*self.clicked_pos):
            self.parent.PopupMenu(PlotPopupMenu(self.parent, self), self.clicked_pos)

    def on_close(self, event):
        self.stop_loading()

    def reload(self):
        self.stop_loading()
        self.resize_figure()
        self.start_loading()

    def stop_loading(self):
        if self._data_thread:
            self._data_thread.stop()
            self._data_thread = None

    def start_loading(self):
        if self.bag_file is None or self.bag_index is None or self.topic is None:
            return
        
        if not self._data_thread:
            self._data_thread = PlotDataLoader(self, self.bag_file, self.bag_index, self.topic)
            self._data_thread.start()

    @staticmethod
    def plot_picker(artist, mouseevent):
        return True, {}

    def on_pick(self, event):
        print event.artist.get_xdata()

    def configure(self):
        frame = PlotConfigureFrame(self)
        frame.Show()

class PlotPopupMenu(wx.Menu):
    periods = [(  -1, 'All'),
               (   1, '1s'),
               (   5, '5s'),
               (  10, '10s'),
               (  30, '30s'),
               (  60, '1min'),
               ( 120, '2min'),
               ( 300, '5min'),
               ( 600, '10min'),
               (1800, '30min'),
               (3600, '1hr')]

    def __init__(self, parent, plot):
        wx.Menu.__init__(self)

        self.parent = parent
        self.plot   = plot

        # Configure...
        configure_item = wx.MenuItem(self, wx.NewId(), 'Configure...')
        self.AppendItem(configure_item)
        self.Bind(wx.EVT_MENU, lambda e: self.plot.configure(), id=configure_item.GetId())

        # Interval...
        self.interval_menu = wx.Menu()
        self.AppendSubMenu(self.interval_menu, 'Interval...', 'Timeline interval to plot')

        for period, label in self.periods:
            period_item = self.PeriodMenuItem(self.interval_menu, wx.NewId(), label, period, plot)
            self.interval_menu.AppendItem(period_item)
            period_item.Check(plot.period == period_item.period)
            
    class PeriodMenuItem(wx.MenuItem):
        def __init__(self, parent, id, label, period, plot):
            wx.MenuItem.__init__(self, parent, id, label, kind=wx.ITEM_CHECK)
            
            self.period = period
            self.plot   = plot

            parent.Bind(wx.EVT_MENU, self.on_menu, id=self.GetId())
    
        def on_menu(self, event):
            self.plot.period = self.period
            self.plot.force_repaint()

class PlotDataLoader(threading.Thread):
    def __init__(self, plot, bag_file, bag_index, topic):
        threading.Thread.__init__(self)

        self.setDaemon(True)

        self.plot      = plot
        self.bag_file  = bag_file
        self.bag_index = bag_index
        self.topic     = topic
        
        self.update_freq = 20   # how many msgs to load before updating the plot

    def run(self):
        try:
            bag_file = rosbag.Bag(self.bag_file.filename)
            start_stamp, end_stamp = self.bag_index.start_stamp, self.bag_index.end_stamp 
            last_stamp = None
            datax, datay = None, None
            load_count = 0
            
            for stamp in self.subdivide(start_stamp, end_stamp):
                if not self.bag_index:
                    break
                
                index = self.bag_index.find_stamp_index(self.topic, stamp)
                if index is None:
                    continue

                # todo: fix
                pos = self.bag_index.msg_positions[self.topic][index][1]
    
                (topic, msg, msg_stamp) = bag_file._read_message(pos)
                if not msg:
                    continue
    
                if datax is None:
                    self.plot._init_plot(self.plot.plot_paths)
    
                    datax, datay = [], []
                    for plot in self.plot.plot_paths:
                        for plot_path in plot:
                            datax.append([])
                            datay.append([])
    
                # Load the data
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

                        datax[series_index].append(plot_stamp - start_stamp)
                        datay[series_index].append(value)

                        series_index += 1
                        
                load_count += 1
    
                # Update the plot
                if load_count % self.update_freq == 0:
                    series_index = 0
                    for plot in self.plot.plot_paths:
                        for plot_path in plot:
                            self.plot.set_series_data(series_index, datax[series_index], datay[series_index])
                            series_index += 1
                
                # Stop loading if the resolution is enough
                if last_stamp and abs(stamp - last_stamp) < 0.1:
                    break
                last_stamp = stamp
    
                time.sleep(0.001)
        finally:
            bag_file.close()

    def stop(self):
        self.bag_index = None

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
