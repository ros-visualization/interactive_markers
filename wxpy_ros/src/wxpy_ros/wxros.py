#!/usr/bin/env python
# Software License Agreement (BSD License)
#
# Copyright (c) 2008, Willow Garage, Inc.
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
#  * Neither the name of the Willow Garage nor the names of its
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


#Some utilities for launching a ROS listener process
#To be deprecated soon with a new implementation of rospy

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

import rostools, rospy
from threading import Thread


class ROSListener(Thread):
  """Creates a thread that handles ros communications."""
  def __init__(self, name):
    """name: the name of the ROS node"""
    Thread.__init__(self)
    self.name = name
    
  def run(self):
   #pass
   print('xx')
   rospy.init_node(self.name,anonymous=True)
   print('xx')
   #rospy.ready(self.name, anonymous=True)
   rospy.spin()

    