import wx

PKG = 'wxpy_ros'
# LOADER #####################
import sys, os, subprocess
try:
    rostoolsDir = (subprocess.Popen(['rospack', 'find', 'rostools'], stdout=subprocess.PIPE).communicate()[0] or '').strip()
    sys.path.append(os.path.join(rostoolsDir,'src'))
    import rostools.launcher
    rostools.launcher.updateSysPath(sys.argv[0], PKG, bootstrapVersion="0.6")
except ImportError:
    print >> sys.stderr, "\nERROR: Cannot locate rostools"
    sys.exit(1)  
# END LOADER #################

# A hack for now for finding the packages in the ROS architecture
def addpackagedir(packagename):
  packageDir = (subprocess.Popen(['rospack', 'find', packagename], stdout=subprocess.PIPE).communicate()[0] or '').strip()
  sys.path.append(os.path.join(packageDir,'src'))
  
  
import sys, traceback, logging, rospy
from std_msgs.msg import *
from threading import Thread


addpackagedir('rosControllers')
import rostools
from rosControllers.msg import RotaryJointState

import time

from myplot import Channel, MyPlot
NAME = 'listener'


class MyChannel(Channel):
    
    def callback(self, state):
        self.addPoint(state.PosAct-state.PosCmd)
  
class MyChannelAct(Channel):
    
    def callback(self, state):
        self.addPoint(state.PosAct)

# A worker class for the ros node
# To be deprecated soon
class WorkerListener(Thread):
  
  def __init__(self):
    Thread.__init__(self)
    
  def run(self):
    rospy.init_node(NAME, anonymous=True)
    rospy.spin()
    
channel = MyChannel()
channel2 = MyChannelAct()
rospy.TopicSub('ARM_L_PAN_state', RotaryJointState, channel.callback)
rospy.TopicSub('ARM_L_PAN_state', RotaryJointState, channel2.callback)
app = wx.PySimpleApp(0)
worker=WorkerListener()
worker.start()
frame = wx.Frame(None, -1,"")
panel = MyPlot(frame)
panel.addChannel(channel)
panel.addChannel(channel2)
frame.Show()
print '>>>>>>>>>>>>>>'
app.MainLoop()
print '>>>>>>>>>>>>>>'
#rospy.spin()
