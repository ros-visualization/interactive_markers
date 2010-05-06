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
import bisect
import numpy
import os
import shutil
import time

import wxversion
WXVER = '2.8'
if wxversion.checkInstalled(WXVER):
    wxversion.select(WXVER)
else:
    print >> sys.stderr, 'This application requires wxPython version %s' % WXVER
    sys.exit(1)
import wx
import wx.lib.wxcairo
# This is a crazy hack to get this to work on 64-bit systems
if 'wxMac' in wx.PlatformInfo:
    pass # Implement if necessary
elif 'wxMSW' in wx.PlatformInfo:
    pass # Implement if necessary
elif 'wxGTK' in wx.PlatformInfo:
    import ctypes
    gdkLib = wx.lib.wxcairo._findGDKLib()
    gdkLib.gdk_cairo_create.restype = ctypes.c_void_p

import Image

from rxbag import BagHelper, TimelineRenderer, TopicMessageView
from image_helper import ImageHelper

## Draws thumbnails of sensor_msgs/Image or sensor_msgs/CompressedImage in the timeline
class ImageTimelineRenderer(TimelineRenderer):
    def __init__(self, timeline, thumbnail_height=64):
        TimelineRenderer.__init__(self, timeline, msg_combine_px=20.0)

        self.thumbnail_height     = thumbnail_height

        self.thumbnail_combine_px = 20.0                      # use cached thumbnail if it's less than this many pixels away
        self.min_thumbnail_width  = 8                         # don't display thumbnails if less than this many pixels across
        self.quality              = Image.NEAREST             # quality hint for thumbnail scaling

        self.thumbnail_cache      = {}
        self.thumbnail_mem_dc     = wx.MemoryDC()
        self.max_cache_size       = 200                       # max number of thumbnails to cache (per topic)

    # TimelineRenderer implementation

    def get_segment_height(self, topic):
        if not self._valid_image_topic(topic):
            return None

        return self.thumbnail_height

    def draw_timeline_segment(self, dc, topic, stamp_start, stamp_end, x, y, width, height):
        if not self._valid_image_topic(topic):
            return False

        max_interval_thumbnail = self.timeline.map_dx_to_dstamp(self.thumbnail_combine_px)

        dc.rectangle(x, y, width, height)
        dc.stroke()

        thumbnail_x, thumbnail_y, thumbnail_height = x + 1, y + 1, height - 2   # leave 1px border

        while True:
            available_width = (x + width) - thumbnail_x
            if available_width < self.min_thumbnail_width:
                # No space remaining to draw thumbnail
                break

            # Load a thumbnail at (or near) this timestamp 
            thumbnail_bitmap = self._get_thumbnail(topic, self.timeline.map_x_to_stamp(thumbnail_x), thumbnail_height, max_interval_thumbnail)
            if not thumbnail_bitmap:
                break

            thumbnail_width = thumbnail_bitmap.get_width()

            if available_width < thumbnail_width:
                # Space remaining, but have to chop off thumbnail
                thumbnail_width = available_width - 2

            dc.set_source_surface(thumbnail_bitmap, thumbnail_x, thumbnail_y)
            dc.rectangle(thumbnail_x, thumbnail_y, thumbnail_width, thumbnail_height)
            dc.fill()

            thumbnail_x += thumbnail_width + 1    # 1px border (but overlap adjacent message)

        return True

    #

    def _valid_image_topic(self, topic):
        return True

    def _get_thumbnail(self, topic, stamp, thumbnail_height, time_threshold):
        """
        Loads the thumbnail from either the bag or the cache
        """
        # Attempt to get a thumbnail from the cache that's within time_threshold secs from stamp
        topic_cache = self.thumbnail_cache.get(topic)
        if topic_cache:
            cache_index = bisect.bisect_right(topic_cache, (stamp, None))
            if cache_index < len(topic_cache):
                (cache_stamp, cache_thumbnail) = topic_cache[cache_index]
                
                cache_dist = abs(cache_stamp - stamp)
                if cache_dist < time_threshold: 
                    return cache_thumbnail

        # Find position of stamp using index
        t = roslib.rostime.Time.from_sec(stamp)
        bag = self.timeline.bag_file
        entry = bag._get_entry(t, bag._get_connections(topic))
        if not entry:
            return None
        pos = entry.position

        # Not in the cache; load from the bag file
        msg_topic, msg, msg_stamp = bag._read_message(pos)

        # Convert from ROS image to PIL image
        pil_image = ImageHelper.imgmsg_to_pil(msg)
        if not pil_image:
            return None
        
        # Calculate width to maintain aspect ratio
        pil_image_size = pil_image.size
        thumbnail_width = int(round(thumbnail_height * (float(pil_image_size[0]) / pil_image_size[1])))

        # Scale to thumbnail size
        thumbnail = pil_image.resize((thumbnail_width, thumbnail_height), self.quality)

        # Convert from PIL Image to Cairo ImageSurface
        thumbnail_bitmap = ImageHelper.pil_to_cairo(thumbnail)
        
        # Store in the cache
        self._cache_thumbnail(topic, msg_stamp, thumbnail_bitmap)
        
        return thumbnail_bitmap
    
    def _cache_thumbnail(self, topic, t, thumbnail):
        # Store in the cache
        if topic not in self.thumbnail_cache:
            self.thumbnail_cache[topic] = []
        topic_cache = self.thumbnail_cache[topic]

        cache_value = (t.to_sec(), thumbnail)

        # Maintain the cache sorted
        topic_cache.insert(bisect.bisect_right(topic_cache, cache_value), cache_value)

        # Limit cache size - remove the farthest entry in the cache
        cache_size = len(topic_cache)
        if cache_size > self.max_cache_size:
            if cache_index < cache_size / 2:
                del topic_cache[cache_size - 1]
            else:
                del topic_cache[0]

class ImageView(TopicMessageView):
    name = 'Image'
    
    def __init__(self, timeline, parent, title, x, y, width, height, max_repaint=None):
        TopicMessageView.__init__(self, timeline, parent, title, x, y, width, height, max_repaint)
        
        self._image         = None
        self._image_topic   = None
        self._image_stamp   = None
        
        self._image_surface = None
        
        self.quality       = Image.NEAREST
        self.indent        = (4, 4)
        self.font          = wx.Font(9, wx.FONTFAMILY_SCRIPT, wx.FONTSTYLE_NORMAL, wx.FONTWEIGHT_NORMAL)
        self.header_color  = wx.BLUE
        
        self.size_set = False
        
    def message_viewed(self, bag, topic, stamp, datatype, msg_index, msg):
        TopicMessageView.message_viewed(self, bag, topic, stamp, datatype, msg_index, msg)

        if not msg:
            self.set_image(None, topic, stamp)
            return

        self.set_image(ImageHelper.imgmsg_to_pil(msg), topic, msg.header.stamp)
                    
        if not self.size_set:
            self.size_set = True
            self.reset_size()

    def message_cleared(self):
        TopicMessageView.message_cleared(self)

        self.set_image(None, None, None)

    def set_image(self, image, image_topic, image_stamp):
        self._image         = image
        self._image_surface = None
        
        self._image_topic = image_topic
        self._image_stamp = image_stamp
        
        self.invalidate()

    def reset_size(self):
        if self._image:
            self.parent.GetParent().SetSize(self._image.size)

    def export_frame(self):
        dialog = wx.FileDialog(self.parent.GetParent(), 'Save frame to...', wildcard='PNG files (*.png)|*.png', style=wx.FD_SAVE)
        if dialog.ShowModal() == wx.ID_OK:
            self._image.SaveFile(dialog.GetPath(), wx.BITMAP_TYPE_PNG)
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
            img = ImageHelper.imgmsg_to_wx(msg)
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

    def on_size(self, event):
        self.resize(*self.parent.GetClientSize())

        self._image_surface = None
        
        self.force_repaint()

    def _get_image_rect(self):
        if self.border:
            return 1, 1, self.width - 2, self.height - 2
        else:
            return 0, 0, self.width, self.height

    def paint(self, dc):
        #dc.SetBrush(wx.WHITE_BRUSH)
        #if self.border:
        #    dc.SetPen(wx.BLACK_PEN)
        #    dc.DrawRectangle(0, 0, self.width, self.height)
        #else:
        #    dc.Clear()

        if not self._image:
            return
        
        ix, iy, iw, ih = self._get_image_rect()

        # Rescale the bitmap if necessary
        if not self._image_surface:
            if self._image.size[0] != iw or self._image.size[1] != ih:
                self._image_surface = ImageHelper.pil_to_cairo(self._image.resize((iw, ih), self.quality))
            else:
                self._image_surface = ImageHelper.pil_to_cairo(self._image)

        # Draw bitmap
        dc.set_source_surface(self._image_surface, ix, iy)
        dc.rectangle(ix, iy, iw, ih)
        dc.fill()

        # Draw overlay
        #dc.SetFont(self.font)
        #dc.SetTextForeground(self.header_color)
        #dc.DrawText(self._image_topic, self.indent[0], self.indent[1])
        #dc.DrawText(BagHelper.stamp_to_str(self._image_stamp.to_sec()), self.indent[0], self.indent[1] + dc.GetTextExtent(self._image_topic)[1])

    def on_right_down(self, event):
        self.parent.PopupMenu(ImagePopupMenu(self.parent, self), event.GetPosition())

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
