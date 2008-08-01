    # import rostools ; will handle importing all the ros depedencies
import rostools
rostools.update_path('rospy')
import rospy

#import the visualitzation package
rostools.update_path('wxpy_ros')
import wxpy_ros
from wxpy_ros import *

#import the GUI
import wx

#import the messages if they are non standard
rostools.update_path('rosControllers')
from rosControllers.msg import RotaryJointState

#Define the channels
class MyChannel(Channel):
    
    def __init__(self):
        Channel.__init__(self, 'r+')    
    
    def callback(self, state):
        self.addPoint(state.PosCmd)
  
class MyChannelAct(Channel):
    
    def callback(self, state):
        self.addPoint(state.PosAct)

class MySubscriber:
    def __init__(self):
        self.channels = []
    
    def callback(self, state):
        for channel in self.channels:
            channel.callback(state)

# Creating channels
channel = MyChannel()
channel2 = MyChannelAct()

#Create a subsciber to the ROS topics:
subscriber = MySubscriber()
subscriber.channels.append(channel)
subscriber.channels.append(channel2)

#Registering to the topics
rospy.TopicSub('ARM_L_PAN_state', RotaryJointState, subscriber.callback)
#Starting ROS in the background
ROSListener("My Node Name").start()

# Creating a wx app
app = wx.PySimpleApp(0)
#Crete a frame
frame = wx.Frame(None, -1,"")
#Add a plot to the frame
panel = WXSlidingPlot(frame)
# set how long we want to display (seconds)
panel.setTimespan(10.0)
#Add the channels to the frame
panel.addChannel(channel)
panel.addChannel(channel2)
# Show it
frame.Show()
# Wait for some things to happen
app.MainLoop()