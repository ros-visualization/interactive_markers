import wx
import roscom

import rostools
rostools.update_path('wxpy_ros')
import wxpy_ros

class wxRosTopicsListBox(wx.Panel):
  
  def __init__(self, parent, id):
    wx.Panel.__init__(self, parent, id)
    
    vbox = wx.BoxSizer(wx.VERTICAL)
    hbox = wx.BoxSizer(wx.HORIZONTAL)
    mainbox = wx.BoxSizer(wx.HORIZONTAL)
    
    # List of the topics
    self.listBox = wx.ListBox(self, -1, size=(200,200))
    # List of elements inside a topic
    self.topicList = wx.ListBox(self, -1, size=(200,200))
    self.refreshBut = wx.Button(self, -1, 'Refresh',size=(80,30))
    self.subscribeBut = wx.Button(self, -1, 'Subscribe',size=(80,30))
    
    self.plot = wxpy_ros.WXSlidingPlot(self)
    
    hbox.Add(self.refreshBut)
    hbox.Add(self.subscribeBut)
        
    vbox.Add(hbox)
    vbox.Add(self.listBox)
    vbox.Add(self.topicList)
    
    mainbox.Add(vbox)
    mainbox.Add(self.plot)
    self.SetSizer(mainbox)
    
    
    self.topics = []
    self.selectedTopic = None
    self.selectedSlot = None
    self.slots = None
    
    self.messageHandler = roscom.RosMessageHandler()
    
    self.Bind(wx.EVT_BUTTON, self.OnRefreshTopicList, self.refreshBut)
    self.Bind(wx.EVT_BUTTON, self.OnSubscribe, self.subscribeBut)
    self.Bind(wx.EVT_LISTBOX, self.OnSelectedTopic, self.listBox)
    self.Bind(wx.EVT_LISTBOX, self.OnSelectedSlot, self.topicList)

    
  def OnRefreshTopicList(self,e):
    self.topics = roscom.getTopicsList()
    print [ t[0] for t in self.topics ]
    self.listBox.Set([ t[0] for t in self.topics ])
    
  def OnSelectedTopic(self, e):
    index = e.GetSelection()
    self.selectedTopic = self.topics[index]
    print 'selected %s' % self.selectedTopic[0]
    obj = roscom.getMessageInstance(self.selectedTopic[1])
    self.slots = obj.__slots__
    self.topicList.Set(self.slots)
  
  def OnSubscribe(self, e):
    print 'subscribing ',self.selectedTopic, self.selectedSlot
    channel = self.messageHandler.subscribe(self.selectedTopic[0], self.selectedTopic[1], self.selectedSlot, 'b')
    print channel
    
  def OnSelectedSlot(self, e):
    index = e.GetSelection()
    self.selectedSlot = self.slots[index]
    print 'selected ',self.selectedTopic, self.selectedSlot
    #channel = self.messageHandler.subscribe(self.selectedTopic[0], self.selectedTopic[1], self.selectedSlot, 'b')

import rostools
rostools.update_path('rospy')
from rospy import client
client.get_published_topics()

def getTopicsList():
  pass

# Creating a wx app
app = wx.PySimpleApp(0)
#Crete a frame
frame = wx.Frame(None, -1,"")
#Add a plot to the frame
panel = wxRosTopicsListBox(frame, -1)
# Show it
frame.Show()
# Wait for some things to happen
app.MainLoop()