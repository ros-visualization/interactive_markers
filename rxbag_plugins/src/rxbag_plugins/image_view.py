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

from rxbag import bag_index, msg_view
from image_helper import ImageHelper

## Draws thumbnails of sensor_msgs/Image in the timeline
class ImageTimelineRenderer(msg_view.TimelineRenderer):
    def __init__(self, timeline, thumbnail_height=48):
        msg_view.TimelineRenderer.__init__(self, timeline, msg_combine_px=10.0)

        self.thumbnail_height     = thumbnail_height

        self.thumbnail_combine_px = 20.0                      # use cached thumbnail if it's less than this many pixels away
        self.min_thumbnail_width  = 8                         # don't display thumbnails if less than this many pixels across
        self.quality              = wx.IMAGE_QUALITY_NORMAL   # quality hint for thumbnail scaling

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

        dc.DrawRectangle(x, y, width, height)

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

            thumbnail_width = thumbnail_bitmap.GetWidth()

            if available_width < thumbnail_width:
                # Space remaining, but have to chop off thumbnail
                thumbnail_width = available_width - 2

                self.thumbnail_mem_dc.SelectObject(thumbnail_bitmap)
                dc.Blit(thumbnail_x, thumbnail_y, thumbnail_width, thumbnail_height, self.thumbnail_mem_dc, 0, 0)
                self.thumbnail_mem_dc.SelectObject(wx.NullBitmap)
            else:
                # Enough space to draw entire thumbnail
                dc.DrawBitmap(thumbnail_bitmap, thumbnail_x, thumbnail_y)

            thumbnail_x += thumbnail_width + 1    # 1px border (but overlap adjacent message)

        return True

    #

    def _valid_image_topic(self, topic):
        return True

    ## Loads the thumbnail from either the bag file or the cache
    def _get_thumbnail(self, topic, stamp, thumbnail_height, time_threshold):
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
        pos = self.timeline.bag_index._data.find_stamp_position(topic, stamp)
        if not pos:
            return None

        # Not in the cache; load from the bag file
        (msg_datatype, msg, msg_stamp) = self.timeline.bag_file.load_message(pos)
        
        # Convert from ROS image to wxImage
        wx_image = ImageHelper.imgmsg_to_wx(msg)
        if not wx_image:
            return None

        # Calculate width to maintain aspect ratio
        thumbnail_width = int(round(thumbnail_height * (float(wx_image.GetWidth()) / wx_image.GetHeight())))
        
        # Scale to thumbnail size
        thumbnail = wx_image.Scale(thumbnail_width, thumbnail_height, self.quality)

        # Convert to bitmap
        thumbnail_bitmap = thumbnail.ConvertToBitmap()

        # Store in the cache
        if not topic_cache:
            topic_cache = []
            self.thumbnail_cache[topic] = topic_cache

        cache_value = (msg_stamp.to_sec(), thumbnail_bitmap)

        # Maintain the cache sorted
        cache_index = bisect.bisect_right(topic_cache, cache_value)
        topic_cache.insert(cache_index, cache_value)

        # Limit cache size - remove the farthest entry in the cache
        cache_size = len(topic_cache)
        if cache_size > self.max_cache_size:
            if cache_index < cache_size / 2:
                del topic_cache[cache_size - 1]
            else:
                del topic_cache[0]
        
        return thumbnail_bitmap

class ImageView(msg_view.TopicMsgView):
    name = 'Image'
    
    def __init__(self, timeline, parent, title, x, y, width, height, max_repaint=None):
        msg_view.TopicMsgView.__init__(self, timeline, parent, title, x, y, width, height, max_repaint)
        
        self._image        = None
        self._image_topic  = None
        self._image_stamp  = None
        
        self._image_bitmap = None
        
        self.quality       = wx.IMAGE_QUALITY_NORMAL
        self.indent        = (4, 4)
        self.font          = wx.Font(9, wx.FONTFAMILY_SCRIPT, wx.FONTSTYLE_NORMAL, wx.FONTWEIGHT_NORMAL)
        self.header_color  = wx.BLUE
        
        self.size_set = False
        
    def message_viewed(self, bag_file, bag_index, topic, stamp, datatype, msg_index, msg):
        msg_view.TopicMsgView.message_viewed(self, bag_file, bag_index, topic, stamp, datatype, msg_index, msg)
        
        if not msg:
            self.set_image(None, topic, stamp)
        else:
            self.set_image(ImageHelper.imgmsg_to_wx(msg), topic, msg.header.stamp)
            
            if not self.size_set:
                self.size_set = True
                self.reset_size()

    def message_cleared(self):
        msg_view.TopicMsgView.message_cleared(self)

        self.set_image(None, None, None)

    def set_image(self, image, image_topic, image_stamp):
        self._image        = image
        self._image_bitmap = None
        
        self._image_topic = image_topic
        self._image_stamp = image_stamp
        
        self.invalidate()

    def reset_size(self):
        if self._image:
            self.parent.GetParent().SetSize(self._image.GetSize())

    def export_frame(self):
        dialog = wx.FileDialog(self.parent.GetParent(), 'Save frame to...', wildcard='PNG files (*.png)|*.png', style=wx.FD_SAVE)
        if dialog.ShowModal() == wx.ID_OK:
            self._image.SaveFile(dialog.GetPath(), wx.BITMAP_TYPE_PNG)
        dialog.Destroy()

    def export_video(self):
        bag_index, bag_file = self.timeline.bag_index, self.timeline.bag_file

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
        for i, (stamp, pos) in enumerate(msg_positions):
            datatype, msg, msg_stamp = bag_file.load_message(pos, bag_index)
            if msg:
                img = ImageHelper.imgmsg_to_wx(msg)
                if img:
                    frame_filename = '%s/frame-%s.png' % (tmpdir, str(stamp)) 
                    print '[%d / %d]' % (i + 1, len(msg_positions))
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

        self._image_bitmap = None
        
        self.force_repaint()

    def _get_image_rect(self):
        if self.border:
            return 1, 1, self.width - 2, self.height - 2
        else:
            return 0, 0, self.width, self.height

    def paint(self, dc):
        dc.SetBrush(wx.WHITE_BRUSH)
        if self.border:
            dc.SetPen(wx.BLACK_PEN)
            dc.DrawRectangle(0, 0, self.width, self.height)
        else:
            dc.Clear()

        if not self._image:
            return
        
        ix, iy, iw, ih = self._get_image_rect()

        # Rescale the bitmap if necessary
        if not self._image_bitmap:
            if self._image.GetWidth() != iw or self._image.GetHeight() != ih:
                self._image_bitmap = self._image.Scale(iw, ih, self.quality).ConvertToBitmap()
            else:
                self._image_bitmap = self._image.ConvertToBitmap()

        # Draw bitmap
        dc.DrawBitmap(self._image_bitmap, ix, iy)

        # Draw overlay
        dc.SetFont(self.font)
        dc.SetTextForeground(self.header_color)
        dc.DrawText(self._image_topic, self.indent[0], self.indent[1])
        dc.DrawText(bag_index.BagIndex.stamp_to_str(self._image_stamp.to_sec()), self.indent[0], self.indent[1] + dc.GetTextExtent(self._image_topic)[1])

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
