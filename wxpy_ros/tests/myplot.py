import wx
import matplotlib
matplotlib.use('WXAgg')
from matplotlib.backends.backend_wxagg import  FigureCanvasWxAgg as FigureCanvas
from matplotlib.figure import Figure
#import matplotlib.transforms
from matplotlib.transforms import Bbox
from matplotlib.transforms import BboxTransform
print Bbox
import numpy
import time

class Channel:
    """"""
    def __init__(self):
        self.x = []
        self.y = []
        self.dataChanged = True
        self.index = -1
        
    def getNext(self):
        x = self.x
        self.x = []
        y = self.y
        self.y = []
        self.dataChanged = False
        return (x,y)
    
    def transform(self,y):
        return y
    
    def addPoint(self,x,y):
        self.x.append(x)
        self.y.append(self.transforms(y))
        self.dataChanged = True
    
    def addPoint(self, y):
        self.index += 1
        self.x.append(self.index)
        self.y.append(self.transform(y))
        self.dataChanged = True

class MyChannel(Channel):
    
    def __init__(self):
        Channel.__init__(self)
        self.pos = 0
        self.dataChanged = True
        
    def getNext(self):
        epsi=0.03
        self.pos += epsi
        self.x = numpy.arange(self.pos, self.pos+epsi, 0.001)
        self.y = numpy.cos(self.x)
        return (self.x, self.y)

class MyChannel2(Channel):
    
    def __init__(self):
        Channel.__init__(self)
        self.pos = 0
        self.dataChanged = True
        
    def getNext(self):
        epsi=0.03
        self.pos += epsi
        self.x = numpy.arange(self.pos, self.pos+epsi, 0.001)
        self.y = numpy.sin(self.x)
        return (self.x, self.y)

class MyPlot(wx.Panel):
    def __init__(self, parent):
        wx.Panel.__init__(self,parent, -1)
    
        self.fig = None
        self.canvas = None
        self.ax = None
        self.background = None
        self.lines = []
        self._doRePlot = True
                
        
        self.foo = 1
        self.t = time.time()
        self.blit_time=0
        self.y = numpy.cos(numpy.arange(0.0,1.0,0.1))
                
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
        self.channels.append(channel)
        
    def setTimespan(self, span):
        self.span = span
        
    def setYlim(self, ymin, ymax):
        self.ylim = [ymin, ymax]
        
    def _resizeCreateContent(self):
        '''Resize graph according to user input and initialize plots'''
        
        self.lines=[]        
        for c in self.channels:
            data=c.getNext()
            line, = self.ax.plot(data[0],data[1], animated = True)
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
        gca.set_xlim([self.begin, (self.begin+self.span)])
        self.ax.grid()
        #self.fig.clear()
        
        self.canvas.draw()        
        self.background = None
        print 'content'
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
        #if self.foo > 2000:
            #u=time.time()
            #print self.foo/(u-self.t), self.blit_time/(u-self.t)
            #exit(0)

    def _SetSize(self, pixels=None):
        if not pixels:
            pixels = self.GetClientSize()
        self._createGraphics()
        self.canvas.SetSize(pixels)
        
        self.fig.set_size_inches(pixels[0]/self.fig.get_dpi(),
        pixels[1]/self.fig.get_dpi(), forward=True)
        self._doRePlot = True
       
    def draw_plot(self):
                if self._doRePlot:
                    self._resizeCreateContent()
                if self.background is None:
                    self.background = self.canvas.copy_from_bbox(self.ax.bbox)
                self.foo += 1
                #self.y = numpy.cos(numpy.arange(0.0,1.0,0.1)+self.foo*0.1)
                # Optimization on the blitting: we compute the box where the changes happen
                changes_box = None

                for i in range(len(self.lines)):
                    data=self.channels[i].getNext()
                    
                    if len(data[1])>0:
                        if self.autolim:
                            print self.autolim[0], data[1], self.autolim[1]
                            self.autolim = [ min(self.autolim[0], min(data[1])), \
                                max(self.autolim[1], max(data[1])) ]
                        else:
                            self.autolim = [ min(data[1]), min(data[1]) ]
                        
                        if changes_box is None:
                            changes_box = Bbox.unit()
                        print '>>>>>>>>'
                        print data[0], data[1]
                        changes_box.update_from_data(numpy.array(data[0]), \
                                numpy.array(data[1]), ignore=changes_box.is_unit())
                        
                        if not self._doRePlot and len(data[0]) > 0 :
                            end = data[0][-1]
                            
                            if end > self.begin+self.span:
                                self.begin += self.span
                                self._doRePlot = True
                                print 'do replot'
                        self.lines[i].set_data(data[0], data[1])
                    else:
                        self.lines[i].set_data([], [])
                
                if not changes_box:
                    return
                #self.canvas.restore_region(self.background)
                for line in self.lines:
                    self.ax.draw_artist(line)
                    #print line.get_transform()
                    tr = line.get_transform()
                    
                changes_box_inframe = changes_box.transformed(tr)
                
                box_padding = 5
                (x,y,l,w) = changes_box_inframe.bounds
                changes_box_inframe = Bbox.from_bounds(x-box_padding, \
                    y-box_padding, l+2*box_padding, w+2*box_padding)
                
                #print 
                t0 = time.time()
                self.canvas.blit(None)
                #self.canvas.blit(changes_box_inframe)
                self.blit_time += time.time() - t0
        
class MyPlotNoBlit(wx.Panel):
    def __init__(self, parent):
        wx.Panel.__init__(self,parent, -1)
            
        self.fig=Figure()
        self.canvas=FigureCanvas(self,-1,self.fig)
        
        self.Bind(wx.EVT_IDLE, self._onIdle)
        self.Bind(wx.EVT_SIZE, self._onSize)
        self._resizeFlag = True
        
        sizer=wx.BoxSizer(wx.VERTICAL)
        sizer.Add(self.canvas,1,wx.GROW)
        self.SetSizer(sizer)
        
    def _onSize(self, evt):
        self._resizeFlag = True
        
    def _onIdle(self, evt):
        if self._resizeFlag:
            self._resizeFlag = False
            self._SetSize()
            self.draw()
            
    def _SetSize(self, pixels=None):
        if not pixels:
            pixels = self.GetClientSize()
        self.canvas.SetSize(pixels)
        self.fig.set_size_inches(pixels[0]/self.fig.get_dpi(),
        pixels[1]/self.fig.get_dpi())
        
    def Plot(self, y):
        a = self.fig.add_subplot(111)
        a.plot(y)
        
    def draw(self):
        print 'draw!'


if __name__ == "__main__":
    app = wx.PySimpleApp(0)
    frame = wx.Frame(None, -1,"")
    panel = MyPlot(frame)
    channel = MyChannel()
    channel2 = MyChannel2()
    panel.addChannel(channel)
    panel.addChannel(channel2)
    panel.setTimespan(1000)
    #panel.Plot(numpy.sin(numpy.arange(0.0,1.0,0.1)))
    #panel.Plot(numpy.cos(numpy.arange(0.0,1.0,0.1)))
    frame.Show()
    app.MainLoop()
