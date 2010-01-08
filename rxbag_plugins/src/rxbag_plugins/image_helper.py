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

import Image
import wx

## Helper class for converting ROS sensor_msgs/Image <-> wxImage
class ImageHelper:
    @staticmethod
    def imgmsg_to_wx(img_msg):
        # Can use rgb8 encoding directly
        if img_msg.encoding == 'rgb8':
            return wx.ImageFromBuffer(img_msg.width, img_msg.height, img_msg.data)

        # Otherwise, use PIL to convert image
        alpha = False
        if img_msg.encoding == 'mono8':
            mode = 'L'
        elif img_msg.encoding == 'rgb8':
            mode = 'RGB'
        elif img_msg.encoding == 'bgr8':
            mode = 'BGR'
        elif img_msg.encoding in ['bayer_rggb8', 'bayer_bggr8', 'bayer_gbrg8', 'bayer_grbg8']:
            mode = 'L'
        elif img_msg.encoding == 'mono16':
            if img_msg.is_bigendian:
                mode = 'F;16B'
            else:
                mode = 'F:16'
        elif img_msg.encoding == 'rgba8':
            mode = 'RGB'
            alpha = True
        elif img_msg.encoding == 'bgra8':
            mode = 'BGR'
            alpha = True

        # TODO: handle alpha
        try:
            pil_img = Image.frombuffer('RGB', (img_msg.width, img_msg.height), img_msg.data, 'raw', mode, 0, 1)
            if pil_img.mode != 'RGB':
                pil_img = pil_img.convert('RGB')
    
            return wx.ImageFromData(pil_img.size[0], pil_img.size[1], pil_img.tostring())
        except:
            print 'Can\'t convert:', mode
            return None
