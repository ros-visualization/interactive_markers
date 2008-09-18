#!/usr/bin/python

import rostools
rostools.update_path('wx_camera_panel')

import wx
import wx_camera_panel
import sys
from optparse import OptionParser

parser = OptionParser(usage="usage: %prog camera_namespace [options]")
parser.add_option( "--ptz", action="store_true", dest="ptz", help="Enable PTZ controls" )
(options, args) = parser.parse_args()

if ( len(args) == 0 ):
    print( "No namespace specified!\n" )
    parser.print_usage()
    sys.exit(1)
    
name = args[0]

app = wx.PySimpleApp()
titlebar_string = "Camera " + name
frame = wx.Frame(None, wx.ID_ANY, titlebar_string, wx.DefaultPosition, wx.Size( 500, 500 ) )

camera_panel = wx_camera_panel.CameraPanel(frame)
camera_panel.setEnabled(True)

if ( options.ptz != None ):
    camera_panel.setPTZEnabled(options.ptz)
    
camera_panel.setName(name)

frame.Show(True)
app.MainLoop()
