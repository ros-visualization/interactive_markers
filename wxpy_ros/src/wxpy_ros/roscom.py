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

## Some common ROS communication utilities. See the wiki documentation and the demo scripts for how to use it.

import rostools
rostools.update_path('rospy')
from rostools import msgspec as msgspec
from rospy import client
import rospy
import copy

# Timestamp imports
import rostools.msg._Time 
_time_class_name = 'rostools.msg._Time.Time'

import wxplot
import wxros

__all__ = ['RosChannel', 'RosSubscriber', 'RosMessageHandler', 'getTopicsList', 'getMessageHandler']

_ros_timestamp = None

## @brief converts a timespamp into a duration in seconds
def _convert_time(stamp):
    return stamp.secs + 0.000000001 * stamp.nsecs

## @class RosChannel
## @brief Channel communication class
#  Gets called by a RosSubscriber and acts as a buffer between ROS and the user event loop
class RosChannel(wxplot.Channel):
    
    def __init__(self, slotName, style):
        wxplot.Channel.__init__(self, style)
        self.slotName = slotName

    ## The fucntion that gets called by ROS
    ## @param value: the value to be stored (any type)
    ## @param time: an optional timestamp. If no timestamp is provided, RosChannel will try to use a global timestamp advertised through ROS. If it not available, it will use an internal integer counter as time source 
    def callback(self, value, time=None):
      global _ros_timestamp
      # So far, no use of a time value even if it is possible
      # To be added when available
      # See documentation in wxpy_ros.Channel
      if time:
        self.addPointXY(time, value)
        return
      if _ros_timestamp:
        self.addPointXY(_ros_timestamp, value)
        return
      self.addPoint(value)
      #print 'received ', self.slotName, value
          
## Channel subscriber to a topic
#  Gets a new message from a channel and dispatch the result to the associated TreeItem
class RosSubscriber:
    """Since only one callback should subscribe to each topic, this class subscribes."""
    def __init__(self, treeItem):
        self.treeItem = treeItem
    
    def callback(self, message):
        global _ros_timestamp
        if hasattr(message, 'has_header') and message.has_header:
            timestamp = _convert_time(message.header.stamp)
            #print "teimstamp", timestamp
            return self.treeItem.update(message, timestamp)
        if _ros_timestamp:
            return self.treeItem.update(message, _ros_timestamp)
        return self.treeItem.update(message, None)

## takes a name 'foo[3]' into ('foo','3')
def _parseSlotName(s):
    l = s.split('[')
    return (l[0], (l[1].split(']'))[0])

## Base class of the tree representing the data advertised through ROSListener
#  Data advertised through ROS can be conceived as a forest of tree items. Each
#  root item correspnds to a topic, each inner element is either a list of items 
#  of a complex type (i.e not one of the atomic types byte, bool, float32, etc.)
#  A leaf item holds the data
#  Thus each element can be represented by a unique path of the form:
#  /TOPIC/ComplexType1/ComplexType2[integer]/AtomicType
class RosMessageItem:
  
    def __init__(self, isList = False):
        # mapping of the children by the name of the field in the message
        # a list item is a dummy item that will get deep copied for each element 
        # of the list
        self.children = {} 
        # for graphical purpose
        self.isList = isList
        self.channels = []
        
    ## Recursively updates the values in the trees given a message coming from ROS    
    def update(self, message, timestamp = None):
        slots = message.__slots__
        structureChanged = False #unused
        #print slots
        for slot in slots:
            object = getattr(message, slot)
            #print "processing slot", slot, object
            if isinstance(object, list):
                #print "is a list"
                for i in range(len(object)):
                    #print "processing %i" % i
                    innerObject = object[i]
                    innerName = '%s[%i]' % (slot, i)
                    #print innerName, innerObject
                    if not innerName in self.children:
                        #print "new copy of ", innerName, self.children[slot]
                        self.children[innerName] = copy.deepcopy(self.children[slot])
                    self.children[innerName].update(innerObject, timestamp)
            else:
                if not slot in self.children:
                    print slot, self.children
                assert slot in self.children
                self.children[slot].update(object, timestamp)
                structureChanged = True
                self.children[slot].update(object, timestamp)
        for channel in self.channels:
            if timestamp:
                channel.callback(message, timestamp)
            else:
                channel.callback(message)
        return structureChanged
                
    # Recursively gets the tree item corresponding to path
    # Only valid for leaves (used for validating user-input paths)                
    def getSlot(self, path):
        if not path or (len(path)==1 and path[0]==''):
          return self
        assert len(path) > 0
        thisSlotName = path[0]
        if not thisSlotName in self.children: # Assumes we have a list
            (baseName, index) = _parseSlotName(thisSlotName)
            item = copy.deepcopy(self.children[baseName])
            self.children[thisSlotName] = item
        return self.children[thisSlotName].getSlot(path[1:])
    
    # Recursively returns a list of the paths of all the leaves below
    def itemPaths(self):
        l = []
        for i in self.children:
            fs = self.children[i].itemPaths()
            if len(fs) == 0:
                if self.children[i].isList:
                    l.append('/%s[]'%(i))
                else:
                    l.append('/%s'%(i))
            for f in fs:
                if self.children[i].isList:
                    l.append('/%s[]%s'%(i,f))
                else:
                    l.append('/%s%s'%(i,f))
        return l
            
## RosMessageLeaf : updates the RosChannel items connected to it
class RosMessageLeaf(RosMessageItem):
          
    def __init__(self, isNumeric = False):
        RosMessageItem.__init__(self)
        # Used for checking is the data is plottable:
        self.isNumeric = isNumeric
        self.channels = []
        
    ## Overloaded function        
    def update(self, value, timestamp):
        for channel in self.channels:
            if timestamp:
                channel.callback(value, timestamp)
            else:
                channel.callback(value)
            
    # Overloaded function        
    def getSlot(self, path):
        assert len(path) == 0
        return self
    
    ## Overloaded function
    def itemPaths(self):
        return []
        
## RosMessageRoot: receives callbacks from ROS
class RosMessageRoot(RosMessageItem):
    def __init__(self, qName):
        RosMessageItem.__init__(self)
        self.hasChanged = True
        self.isRegistered = False
        self.qualifiedMessageType = qName
   
    # unused
    def register(sel):
        self.isRegistered = True
        
def _getContext(mType):
    s = mType.split('/')
    if len(s) == 2:
        return s[0]
    return None

def _messageNameInContext(msg_name, context):
    if context and context+msgspec.SEP+msg_name in msgspec.REGISTERED_TYPES:
        return context+msgspec.SEP+msg_name
    return msg_name
## RosMessageHandler: manages all comunications with ROSListener
#  The usual way to use it is:
#  mh = roscom.RosMessageHandler()
#  Query all the current topics in ROS:
#  mh.queryTopicsTree()
#  get a channel corresponding to a path
#  c = mh.subscribe('/mechanisms/joint_states[1]/velocity','b')
#  do something with the channel
class RosMessageHandler:
    def __init__(self, name='WXROSPY_MESSAGE_HANDLER'):
        # List of the topics
        self.topicsSubs = dict()
        wxros.ROSListener(name).start() 
        self.topics = {} 
        self.queryTopicsTree()
        
    def subscribe(self, itemPath, style, ChannelType=RosChannel):
        
        item = self._getItem(itemPath)
        topicName = self._getTopic(itemPath)
        assert topicName in self.topics
        if not self.topics[topicName].isRegistered:
            self.topicsSubs[topicName] = RosSubscriber(self.topics[topicName])
            messageName, messageModName, messageType = self.topics[topicName].qualifiedMessageType
            exec('import %s' % messageModName)
            rospy.TopicSub(self._getTopic(itemPath), eval(messageType), self.topicsSubs[topicName].callback)
        channel = ChannelType(itemPath, style)
        item.channels.append(channel)
        return channel

        
    def getItemPaths(self, topicName):
        #print self.topics
        return [ '%s%s' % (topicName, s) for s in self.topics[topicName].itemPaths()]
        
        
    def isValidItemPath(self, itemPath):
        try:
            if itemPath.count('[]')>0:
                return False
            l = self._getItem(itemPath)
            if l and l.isNumeric:
                return True
        except:
            return False
        
    def _getItem(self, itemPath):
        l = itemPath.split('/')
        topic = '/%s'%l[1]
        assert topic in self.topics
        return self.topics[ topic ].getSlot(l[2:])

    
    def queryTopicsTree(self):
        """Queries botherder to get the list of the advertised topics and builds a tree of the available elements in the topics."""
        l = getTopicsList()
        # Registers and imports all the types
        for (topic_name, topic_type) in l:
            (modName, messageModName, messageName) = messageNames(topic_type)
            msgspec.load_package(modName)
            importMessageModule(modName, messageModName)
            exec('import %s' % messageModName)
            #print 'import %s' % modName
            #exec('import %s' % modName)
            #print 'import %s' % messageModName 
            #exec('import %s' % messageModName)
          
        # Now we have all the types we need loaded.
        for (topic_name, topic_type) in l:
            topicQualifiedType = messageNames(topic_type)
            self.topics[topic_name] = RosMessageRoot(topicQualifiedType)                       
            self._buildMessageTree(self.topics[topic_name], msgspec.REGISTERED_TYPES[topic_type], _getContext(topic_type))
    
    def _buildMessageTree(self, messageItem, messageType, context=None):
        
        for (slot_type, slot_name) in messageType.fields():
            if isAtomic(slot_type):
                messageItem.children[slot_name] = RosMessageLeaf(isNumeric(slot_type))
            elif slot_type.count('[]') > 0:
                stripped_slot_type = (slot_type.split('[]'))[0]
                if isAtomic(stripped_slot_type):
                    messageItem.children[slot_name] = RosMessageLeaf(isNumeric(stripped_slot_type))
                else:
                  messageItem.children[slot_name] = RosMessageItem(True)
                  self._buildMessageTree(messageItem.children[slot_name], msgspec.REGISTERED_TYPES[_messageNameInContext(stripped_slot_type, context)], context)
                
            else:
                messageItem.children[slot_name] = RosMessageItem()
                self._buildMessageTree(messageItem.children[slot_name], msgspec.REGISTERED_TYPES[_messageNameInContext(slot_type, context)], context)
                
    def _getTopic(self, itemPath):
        l = itemPath.split('/')
        topic = '/%s'%l[1]
        return topic
   
    
numericTypes=['byte','float32', 'float64', 'int16', 'int32', 'int64', 'uint32', 'uint64']    


def isNumeric(obj_type):
    return obj_type in numericTypes   
    
def isAtomic(obj_type):
    return obj_type in ['string'] or isNumeric(obj_type)   
    
def getTopicsList():
    return client.get_published_topics()


def messageNames(rosMessageName):
    """ Given the name of the message as advertised by ROS, returns the name of the module and the fully qualified name of the message.
    This assumes the module respects the naming conventions."""
    l = rosMessageName.split('/')
    modName = l[0]
    messageModName = l[0]+'.msg._'+l[1]
    messageName = l[0]+'.msg._'+l[1]+'.'+l[1]
    return (modName, messageModName, messageName)
    
    
def importMessageModule(moduleName, messageModName):
    """Given the fully qualified name of the message class, tries to import the message class.
    Will add some exception handling."""
    upp_com = 'rostools.update_path(\'%s\')'%moduleName
    exec(upp_com)
    print messageModName
    exec('import %s' % messageModName)

_handler = RosMessageHandler()

## @brief Returns a handler
#This should be the only way to get a handler. This way, only one handler is created per process. This minimiizes the workload on ROS.   
def getMessageHandler():
    global _handler
    return _handler
