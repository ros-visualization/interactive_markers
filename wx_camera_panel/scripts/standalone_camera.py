#!/usr/bin/python

import rostools
rostools.update_path('wx_camera_panel')

import wx
import wx_camera_panel
from optparse import OptionParser

parser = OptionParser()
parser.add_option( "-i", "--image", action="store", type="string", dest="image_topic", help="Set the image topic to subscribe to" )
parser.add_option( "-s", "--ptz_state", action="store", type="string", dest="ptz_state_topic", help="Set the incoming ptz state topic to subscribe to" )
parser.add_option( "-c", "--ptz_control", action="store", type="string", dest="ptz_control_topic", help="Set the outgoing ptz control topic to advertise on" )
parser.add_option( "-n", "--name", action="store", type="string", dest="name", help="Set the name of the camera" )
(options, args) = parser.parse_args()

app = wx.PySimpleApp()
titlebar_string = "Camera"
if ( options.name != None ):
    titlebar_string = titlebar_string + " " + options.name
frame = wx.Frame(None, wx.ID_ANY, titlebar_string)

camera_panel = wx_camera_panel.CameraPanel(frame)
camera_panel.SetEnabled(True)

if ( options.image_topic != None ):
    camera_panel.SetImageSubscription( options.image_topic )

if ( options.ptz_state_topic != None ):
    camera_panel.SetPTZStateSubscription( options.ptz_state_topic )
    
if ( options.ptz_control_topic != None ):
    camera_panel.SetPTZControlCommand( options.ptz_control_topic )    

frame.Show(True)
app.MainLoop()
