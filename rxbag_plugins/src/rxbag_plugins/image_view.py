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

import os
import shutil
import sys
import threading
import time

import numpy
import wx

import Image

from rxbag import bag_helper, TopicMessageView
import image_helper

class ImageView(TopicMessageView):
    name = 'Image'
    
    def __init__(self, timeline, parent, title, x, y, width, height):
        TopicMessageView.__init__(self, timeline, parent, title, x, y, width, height)
        
        self._image_lock  = threading.RLock()
        self._image       = None
        self._image_topic = None
        self._image_stamp = None

        self._image_surface = None

        self._overlay_font_size = 14.0
        self._overlay_indent    = (4, 4)
        self._overlay_color     = (0.2, 0.2, 1.0)

        self._size_set = False

    def message_viewed(self, bag, msg_details):
        TopicMessageView.message_viewed(self, bag, msg_details)
        
        topic, msg, t = msg_details

        if not msg:
            self.set_image(None, topic, stamp)
        else:
            self.set_image(image_helper.imgmsg_to_pil(msg), topic, msg.header.stamp)
    
            if not self._size_set:
                self._size_set = True
                self.reset_size()

    def message_cleared(self):
        TopicMessageView.message_cleared(self)

        self.set_image(None, None, None)

    def set_image(self, image, image_topic, image_stamp):
        with self._image_lock:
            self._image         = image
            self._image_surface = None
            
            self._image_topic = image_topic
            self._image_stamp = image_stamp
            
        self.invalidate()

    def reset_size(self):
        if self._image:
            self.parent.GetParent().SetSize(self._image.size)

    def on_size(self, event):
        with self._image_lock:
            self.resize(*self.parent.GetClientSize())
    
            self._image_surface = None
            
        self.invalidate()

    def paint(self, dc):
        with self._image_lock:
            if not self._image:
                return

            ix, iy, iw, ih = 0, 0, self.width, self.height

            # Rescale the bitmap if necessary
            if not self._image_surface:
                if self._image.size[0] != iw or self._image.size[1] != ih:
                    self._image_surface = image_helper.pil_to_cairo(self._image.resize((iw, ih), Image.NEAREST))
                else:
                    self._image_surface = image_helper.pil_to_cairo(self._image)
    
            # Draw bitmap
            dc.set_source_surface(self._image_surface, ix, iy)
            dc.rectangle(ix, iy, iw, ih)
            dc.fill()
    
            # Draw overlay
            dc.set_font_size(self._overlay_font_size)
            font_height = dc.font_extents()[2]
            dc.set_source_rgb(*self._overlay_color)
            dc.move_to(self._overlay_indent[0], self._overlay_indent[1] + font_height)
            dc.show_text(bag_helper.stamp_to_str(self._image_stamp))

    def on_right_down(self, event):
        self.parent.PopupMenu(ImagePopupMenu(self.parent, self), event.GetPosition())

    def export_frame(self):
        dialog = wx.FileDialog(self.parent.GetParent(), 'Save frame to...', wildcard='PNG files (*.png)|*.png', style=wx.FD_SAVE)
        if dialog.ShowModal() == wx.ID_OK:
            self._image.save(dialog.GetPath(), 'png')
        dialog.Destroy()

    def export_video(self):
        bag_file = self.timeline.bag_file

        msg_positions = bag_index.msg_positions[self._image_topic]
        if len(msg_positions) == 0:
            return
        
        dialog = wx.FileDialog(self.parent.GetParent(), 'Save video to...', wildcard='AVI files (*.avi)|*.avi', style=wx.FD_SAVE)
        if dialog.ShowModal() != wx.ID_OK:
            return
        video_filename = dialog.GetPath()

        import tempfile
        tmpdir = tempfile.mkdtemp()

        frame_count = 0
        w, h = None, None
        
        total_frames = len(bag_file.read_messages(self._image_topic, raw=True))

        for i, (topic, msg, t) in enumerate(bag_file.read_messages(self._image_topic)):
            img = image_helper.imgmsg_to_wx(msg)
            if img:
                frame_filename = '%s/frame-%s.png' % (tmpdir, str(t))
                print '[%d / %d]' % (i + 1, total_frames)
                img.SaveFile(frame_filename, wx.BITMAP_TYPE_PNG)
                frame_count += 1

                if w is None:
                    w, h = img.GetWidth(), img.GetHeight()

        if frame_count > 0:
            positions = numpy.array([stamp for (stamp, pos) in msg_positions])
            if len(positions) > 1:
                spacing = positions[1:] - positions[:-1]
                fps = 1.0 / numpy.median(spacing)

            print 'Encoding %dx%d at %d fps' % (w, h, fps)

            try:
                command = ('mencoder',
                           'mf://' + tmpdir + '/*.png',
                           '-mf',
                           'type=png:w=%d:h=%d:fps=%d' % (w, h, fps),
                           '-ovc',
                           'lavc',
                           '-lavcopts',
                           'vcodec=mpeg4',
                           '-oac',
                           'copy',
                           '-o',
                           video_filename)
                os.spawnvp(os.P_WAIT, 'mencoder', command)
            finally:
                shutil.rmtree(tmpdir)

        dialog.Destroy()

class ImagePopupMenu(wx.Menu):
    def __init__(self, parent, image_view):
        wx.Menu.__init__(self)

        self.image_view = image_view

        # Reset Size
        reset_item = wx.MenuItem(self, wx.NewId(), 'Reset Size')
        self.AppendItem(reset_item)
        self.Bind(wx.EVT_MENU, lambda e: self.image_view.reset_size(), id=reset_item.GetId())

        # Export to PNG...
        export_frame_item = wx.MenuItem(self, wx.NewId(), 'Export to PNG...')
        self.AppendItem(export_frame_item)
        self.Bind(wx.EVT_MENU, lambda e: self.image_view.export_frame(), id=export_frame_item.GetId())

        # Export to AVI...
        #export_video_item = wx.MenuItem(self, wx.NewId(), 'Export to AVI...')
        #self.AppendItem(export_video_item)
        #self.Bind(wx.EVT_MENU, lambda e: self.image_view.export_video(), id=export_video_item.GetId())
