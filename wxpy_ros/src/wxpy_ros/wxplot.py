#!/usr/bin/env python
# Software License Agreement (BSD License)
#
# Copyright (c) 2008, Willow Garage, Inc.
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
#  * Neither the name of the Willow Garage nor the names of its
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

##"""WXPlot
##A small utility class for embedding dynamic plots in WXpython.
##"""

"""
WXPlot, some utility panels and classes for embedding dynamic plots in WXPython.
@author: Timothy Hunter <tjhunter@willowgarage.com>
"""

import wx

# Check for numpy and matplotlib
try:
  import matplotlib
  BAD_MPL_VERSION=True
finally:
  BAD_MPL_VERSION=False


#import matplotlib
import roslib
roslib.load_manifest('matplotlib')
import matplotlib
matplotlib.use('WXAgg')

from matplotlib.backends.backend_wxagg import  FigureCanvasWxAgg as FigureCanvas
from matplotlib.figure import Figure
from matplotlib.transforms import Bbox, BboxTransform
import numpy
import time
import copy

# We want to check if the user has a different version of matplotlib installed on the computer
__mpl_proper__ = ['0.98.2','0.98.1']
__numpy_proper__ = ['1.1.0', '1.1.1']

# Check for numpy version:
if not numpy.__version__ in __numpy_proper__:
  print "*** Wrong version of numpy detected ***\n The following version was found : %s and it is not a supported version (the supported versions are: %s). I cannot continue.\n This usually happens when a different version was packaged and installed with your operating system, as they conflict with the version located in the 3rd party packge directory. In this case, uninstall the packages \'numpy\' and try again." % (numpy.__version__, __numpy_proper__ )
  raise ImportError('wrong version of numpy')


if BAD_MPL_VERSION:
  print "*** Wrong version of matplotlib detected ***\n The following version was found : %s and it is not a supported version (the supported versions are: %s). I cannot continue.\n This usually happens when a different version was packaged and installed with your operating system, as they conflict with the version located in the 3rd party packge dir. In this case, uninstall the packages \'matplotlib\' and try again." % (matplotlib.__version__, __mpl_proper__ )
  raise ImportError('wrong version of matplotlib')


__all__ = ['Channel','WXPlot','WXSlidingPlot','NestPanel']


def NestPanel(top_panel, sub_panel):
  """A wrapper method to embed panels into XRC-managed panels.
  @author: Jeremy Wright
  Imported from the qualification module and adapted for WXPlot by Timothy Hunter."""

  # Cache the original size of the top-level frame
  start_size = top_panel.GetTopLevelParent().GetSize()

  # Get the top_level's sizer
  sizer = top_panel.GetSizer()

  # Create a new one if it doesn't exist
  if (sizer == None):
    sizer = wx.BoxSizer()
    top_panel.SetSizer(sizer)

  # Clear the sizer (destructively)
  sizer.Clear(True)

  # Add our sub_panel to the sizer
  sizer.Add(sub_panel,1,wx.EXPAND)

  # Reset size hints for the top_panel
  sizer.SetSizeHints(top_panel)

  # Also reset size hints for the top level frame
  sizer2 = top_panel.GetTopLevelParent().GetSizer()
  if not sizer2:
      sizer2 = wx.BoxSizer()
      top_panel.GetTopLevelParent().SetSizer(sizer2)
  sizer2.SetSizeHints(top_panel.GetTopLevelParent())

  # Reset size to our cached size, and then layout
  top_panel.GetTopLevelParent().SetSize(start_size)
  top_panel.GetTopLevelParent().Layout()


class Channel:
    """The interaction class with the plot panel.
    Note you have to generate one instance for each panel (i.e. you cannot 
    share a Channel between several plots.
    Channel acts as a buffer between the data input and the plot widget."""
    def __init__(self, style='b'):
        self.style = style
        self.flush()
        
    def getNext(self):
        """Returns the data waiting to be plotted and cleans the cache afterwards."""
        x = self.x
        self.x = []
        y = self.y
        self.y = []
        self.dataChanged = False
        return (x,y)
    
    def addPointXY(self,x,y):
        """Adds a point to be plotted.
        x,y: floats"""
        self.x.append(x)
        self.y.append(y)
        self.dataChanged = True
    
    def addPoint(self, y):
        """Adds a point and a timestamp.
        y: float"""
        self.index += 1
        self.x.append(time.time() - self.time_begin)
        self.y.append(y)
        self.dataChanged = True
        
    def flush(self):
        """Reinitializes a channel. Needed when a channel is added in a plot"""
        self.x = []
        self.y = []
        self.dataChanged = True
        self.time_begin = time.time()
        self.index = -1

class WXPlot(wx.Panel):
    """WXPlot: a plot pannel for WxPython applications.
    This plots acts as an oscilloscope.
    use addChannels() to register some channels to be plotted
    set the length of the run with setTimespan
    by default, WXPlot will try to guess the vertical limits of the plot after a while.
    You can fix them by calling setYLim"""
    # Todo: right now, the graph is redrawn on idelevents, which means it will 
    # try to eat as much cpu as possible. Not optimal...
    def __init__(self, parent):
        wx.Panel.__init__(self,parent, -1)
    
        self.fig = None
        self.canvas = None
        self.ax = None
        self.background = None
        self.lines = []
        self.cached_datax = []
        self.cached_datay = []
        self._doRePlot = True
                
        self.ylim = None
        self.autolim = None
        self.span = 500
        self.begin = 0
        self.channels = []
        
        self._SetSize()
        
        self.Bind(wx.EVT_IDLE, self._onIdle)
        self.Bind(wx.EVT_SIZE, self._onSize)
        self._resizeFlag = True
        
        sizer=wx.BoxSizer(wx.VERTICAL)
        sizer.Add(self.canvas,1,wx.GROW)
        self.SetSizer(sizer)
        self.canvas.Show() 
    
    def addChannel(self, channel):
        """Add a channel to plot.
        channel: object of type Channel."""
        self.channels.append(channel)
        self._doRePlot = True
        
    def setTimespan(self, span):
        """Set the length of the graph.
        span: double (seconds)"""
        self.span = span
        self._doRePlot = True
        
    def setYlim(self, ymin, ymax):
        """Set the vertical limits of the graph:
        ymin: minimum printed on the graph (float)
        ymin: maximum printed on the graph (float)"""
        self.ylim = [ymin, ymax]
        self._doRePlot = True
        
    def draw_plot(self):
        if self._doRePlot:
            self._resizeCreateContent()
        if self.background is None:
            self.background = self.canvas.copy_from_bbox(self.ax.bbox)
        
        changes_box = None
        #try:
            
        for i in range(len(self.lines)):
            data=self.channels[i].getNext()

            if len(data[0]) > 0:
              if len(data[0]) != len(data[1]):
                print 'incoherent data',  len(data[0]),  len(data[1])
                return
              # Add new points
              #print type(self.cached_datax[i]), type(data[0])
              if isinstance(data[0], numpy.ndarray):
                self.cached_datax[i] += list(data[0])
                self.cached_datay[i] += data[1].tolist()
              else:
                self.cached_datax[i] += data[0]
                self.cached_datay[i] += data[1]
            
            if len(data[1])>0:
                if self.autolim:
                    self.autolim = [ min(self.autolim[0], min(data[1])), \
                        max(self.autolim[1], max(data[1])) ]
                else:
                    self.autolim = [ min(data[1]), min(data[1]) ]
                
                if changes_box is None:
                    changes_box = Bbox.unit()
                changes_box.update_from_data(numpy.array(data[0]), \
                        numpy.array(data[1]), ignore=changes_box.is_unit())
                
                if not self._doRePlot and len(data[0]) > 0 :
                    end = data[0][-1]
                    
                    if end > self.begin+self.span:
                        self.begin = end
                        self._doRePlot = True
                #print 'C'
                #self.cached_datax[i] += (data[0])
                #self.cached_datay[i] += (data[1])
                #print self.cached_datay[i], self.cached_datax[i]
                self.lines[i].set_data(self.cached_datax[i], \
                    self.cached_datay[i])
                #self.lines[i].set_data(data[0], data[1])
            else:
                self.lines[i].set_data([], [])
        
        if not changes_box:
            return
        
        for line in self.lines:
            self.ax.draw_artist(line)
            tr = line.get_transform()
            
        changes_box_inframe = changes_box.transformed(tr)
        
        box_xpadding = 20
        box_ypadding = 100
        (x,y,l,w) = changes_box_inframe.bounds
        changes_box_inframe = Bbox.from_bounds(x-box_xpadding, \
            y-box_ypadding, l+2*box_xpadding, w+2*box_ypadding)
        
        #self.canvas.blit(None)
        self.canvas.blit(changes_box_inframe)
        #except :
                #pass
                #print '>>>>>>>>>>>>>>'
                #print e
    
    def _resizeCreateContent(self):
        """Resize graph according to user input and initialize plots"""
        self.lines=[]        
        for c in self.channels:
            data=c.getNext()
            line, = self.ax.plot(data[0],data[1], c.style, animated = True)
            self.lines.append(line)
            self.cached_datax.append([])
            self.cached_datay.append([])
        gca = self.fig.gca()
        #TODO: add an auto mode here
        if self.ylim:
            gca.set_ylim(self.ylim)
        else:
            if self.autolim:
                diff = self.autolim[1] - self.autolim[0]
                gca.set_ylim([self.autolim[0] - 0.1*diff, self.autolim[1] + 0.1*diff])
            else:
                gca.set_ylim([-1,1])
        gca.set_xlim([self.begin, (self.begin+self.span)])
        self.ax.grid()
        
        self.canvas.draw()        
        self.background = None
        self._doRePlot = False
        
        
    def _createGraphics(self):
        """Reallocate new figure and take care of panel resizing issues"""
        self.fig=Figure()
        self.canvas=FigureCanvas(self,-1,self.fig)
        self.ax = self.fig.add_subplot(111)
        
        self.ax._cachedRenderer=self.canvas.get_renderer()
        
    def _onSize(self, evt):
        self._resizeFlag = True
        
    def _onIdle(self, event):
        event.RequestMore(True)
        if self._resizeFlag:
            self._resizeFlag = False
            self._SetSize()
        self.draw_plot()

    def _SetSize(self, pixels=None):
        if not pixels:
            pixels = self.GetClientSize()
        self._createGraphics()
        self.canvas.SetSize(pixels)
        
        self.fig.set_size_inches(pixels[0]/self.fig.get_dpi(),
        pixels[1]/self.fig.get_dpi(), forward=True)
        self._doRePlot = True


class WXSlidingPlot(wx.Panel):
    """WXPlot: a plot pannel for WxPython applications.
    This plots acts as an oscilloscope.
    use addChannels() to register some channels to be plotted
    set the length of the run with setTimespan
    by default, WXPlot will try to guess the vertical limits of the plot after a while.
    You can fix them by calling setYLim"""
    # Todo: right now, the graph is redrawn on idelevents, which means it will 
    # try to eat as much cpu as possible. Not optimal...
    def __init__(self, parent):
        wx.Panel.__init__(self,parent, -1, size=(400,400))
    
        self.fig = None
        self.canvas = None
        self.ax = None
        self.background = None
        self.lines = []
        self.cached_datax = []
        self.cached_datay = []
        self._doRePlot = True
        self._doFlush = True
                
        self.ylim = None
        self.autolim = None
        self.span = 500
        self.begin = 0
        self.channels = []
        
        self._SetSize()
        
        self.Bind(wx.EVT_IDLE, self._onIdle)
        self.Bind(wx.EVT_SIZE, self._onSize)
        self._resizeFlag = True
        
        sizer=wx.BoxSizer(wx.VERTICAL)
        sizer.Add(self.canvas,1,wx.GROW)
        self.SetSizer(sizer)
        self.canvas.Show() 
    
    def addChannel(self, channel):
        """Add a channel to plot.
        channel: object of type Channel."""
        self.channels.append(channel)
        self._doFlush = True
        
    def setTimespan(self, span):
        """Set the length of the graph.
        span: double (seconds)"""
        self.span = span
        self._doRePlot = True
        
    def setYlim(self, ymin, ymax):
        """Set the vertical limits of the graph:
        ymin: minimum printed on the graph (float)
        ymin: maximum printed on the graph (float)"""
        self.ylim = [ymin, ymax]
        self._doRePlot = True
        
    def draw_plot(self):
        if self._doFlush:
            self._createContent()
            self._doRePlot = True
        if self._doRePlot:
            self._resizeCreateContent()
        if self.background is None:
            padding = 0
            print self.ax.bbox.bounds
            bbox = Bbox.from_bounds(self.ax.bbox.bounds[0],self.ax.bbox.bounds[1],self.ax.bbox.bounds[2]+2*padding,self.ax.bbox.bounds[3]+2*padding)
            self.background = self.canvas.copy_from_bbox(bbox)
        
        changes_box = None
        #try:
            
        for i in range(len(self.lines)):
            data=self.channels[i].getNext()
            if len(data[0]) > 0:
              if len(data[0]) != len(data[1]):
                print 'incoherent data',  len(data[0]),  len(data[1])
                return
              # Add new points
              #print type(self.cached_datax[i]), type(data[0])
              if isinstance(data[0], numpy.ndarray):
                self.cached_datax[i] += data[0].tolist()
                self.cached_datay[i] += data[1].tolist()
              else:
                self.cached_datax[i] += data[0]
                self.cached_datay[i] += data[1]
                
              # Do we need to reshape the graph?
              if self.autolim:
                  data_ymin = min(data[1])
                  data_ymax = max(data[1])
                  if self.autolim[0] > data_ymin or self.autolim[1] < data_ymax:
                      self.autolim = [ min(self.autolim[0], data_ymin), \
                          max(self.autolim[1], data_ymax) ]
                      self._doRePlot = True
                      print 'change autolim', self.autolim, data_ymin, data_ymax
                      
              else:
                  self.autolim = [ min(data[1]), max(data[1]) ]
                  print 'set autolim', self.autolim
                  self._doRePlot = True
                  

              #print i,self.cached_datax[i]
  
              # If some points are beyond the window, move the window
              if len(self.cached_datax[i]) > 0 and self.cached_datax[i][-1] > self.begin + self.span:
                    #print 42, self.cached_datax[i][-1]
                    self.begin = self.cached_datax[i][-1] - self.span
        # No need for replot, we have to reallocate                    
        if self._doRePlot:
            return
        # Remove the points behind the window
        for i in range(len(self.lines)):
            # Bad points to move?
            if len(self.cached_datax[i]) > 0 and self.cached_datax[i][0] < self.begin:
                index = 0
                # Could be faster by divide and conquer...
                while index < len(self.cached_datax[i]) and self.cached_datax[i][index] < self.begin :
                  index += 1
                # Prune out old data
                self.cached_datax[i] = self.cached_datax[i][index:]
                self.cached_datay[i] = self.cached_datay[i][index:]
                # Set line data.
            self.lines[i].set_data(numpy.array(self.cached_datax[i])-self.begin, \
                numpy.array(self.cached_datay[i]))
        
        self.canvas.restore_region(self.background)

        for line in self.lines:
            self.ax.draw_artist(line)
        
        self.canvas.blit(None)
        #self.canvas.blit(changes_box_inframe)
        #except :
                #pass
                #print '>>>>>>>>>>>>>>'
                #print e
    def _createContent(self):
        print 'create content'
        self.cached_datax = []  
        self.cached_datay = []
        print self.channels
        for c in self.channels:
            self.cached_datax.append([])
            self.cached_datay.append([])
            c.flush()
        self._doFlush = False
    
    def _resizeCreateContent(self):
        """Resize graph according to user input and initialize plots"""
        self.lines=[]      
        for c in self.channels:
            data=c.getNext()
            line, = self.ax.plot(data[0],data[1], c.style, animated = True)
            self.lines.append(line)
        gca = self.fig.gca()
        #TODO: add an auto mode here
        if self.ylim:
            gca.set_ylim(self.ylim)
        else:
            if self.autolim:
                diff = self.autolim[1] - self.autolim[0]
                gca.set_ylim([self.autolim[0] - 0.1*diff, self.autolim[1] + 0.1*diff])
            else:
                gca.set_ylim([-1,1])
        #gca.set_xlim([self.begin, (self.begin+self.span)])
        gca.set_xlim([0, (self.span)])
        self.ax.grid()
        
        self.canvas.draw()        
        self.background = None
        self._doRePlot = False
        print 'R'
        
        
    def _createGraphics(self):
        """Reallocate new figure and take care of panel resizing issues"""
        self.fig=Figure()
        self.canvas=FigureCanvas(self,-1,self.fig)
        self.ax = self.fig.add_subplot(111)
        
        self.ax._cachedRenderer=self.canvas.get_renderer()
        
    def _onSize(self, evt):
        self._resizeFlag = True
        
    def _onIdle(self, event):
        event.RequestMore(True)
        if self._resizeFlag:
            self._resizeFlag = False
            self._SetSize()
        self.draw_plot()

    def _SetSize(self, pixels=None):
        if not pixels:
            pixels = self.GetClientSize()
        self._createGraphics()
        self.canvas.SetSize(pixels)
        
        self.fig.set_size_inches(pixels[0]/self.fig.get_dpi(),
        pixels[1]/self.fig.get_dpi(), forward=True)
        self._doRePlot = True

if __name__ == "__main__":
    
    class MyChannel(Channel):
        
        def __init__(self, style):
            Channel.__init__(self, style)
            self.pos = 0
            self.dataChanged = True
            
        def getNext(self):
            epsi=0.03
            self.pos += epsi
            self.x = numpy.arange(self.pos, self.pos+epsi, 0.001)
            self.y = numpy.cos(self.x)
            return (self.x, self.y)
        
        def transform(self,y):
            print 'C'
            return numpy.exp(y)
    
    class MyChannel2(Channel):
        
        def __init__(self, style):
            Channel.__init__(self, style)
            self.pos = 0
            self.dataChanged = True
            
        def getNext(self):
            epsi=0.03
            self.pos += epsi
            self.x = numpy.arange(self.pos, self.pos+epsi, 0.001)
            self.y = numpy.sin(self.x)
            return (self.x, self.y)

    app = wx.PySimpleApp(0)
    frame = wx.Frame(None, -1,"")
    panel = WXSlidingPlot(frame)
    panel.setTimespan(10.0)
    channel = MyChannel('r')
    channel2 = MyChannel2('y+')
    panel.addChannel(channel)
    panel.addChannel(channel2)
    frame.Show()
    app.MainLoop()
