#!/usr/bin/python

import rostools
rostools.update_path('wx_rosout')

WXVER = '2.8'
import wxversion
if wxversion.checkInstalled(WXVER):
  wxversion.select(WXVER)
else:
  print >> sys.stderr, "This application requires wxPython version %s"%(WXVER)
  sys.exit(1)

import wx
import wx_rosout

app = wx.PySimpleApp()
frame = wx.Frame(None, wx.ID_ANY, "Rosout", wx.DefaultPosition, wx.Size( 800, 600 ) )

panel = wx_rosout.RosoutPanel(frame)
panel.setEnabled(True)   

frame.Show(True)
app.MainLoop()

