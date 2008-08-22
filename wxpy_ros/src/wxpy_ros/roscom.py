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

__all__ = ['RosChannel', 'RosSubscriber', 'RosMessageHandler', 'getTopicsList']

_ros_timestamp = None

def _convert_time(stamp):
    return stamp.secs + 0.000000001 * stamp.nsecs

## Channel communication class
#  Gets called by a RosSubscriber and acts as a buffer between ROS and the user event loop
class RosChannel(wxplot.Channel):
    
    def __init__(self, style, slotName):
        wxplot.Channel.__init__(self, style='b')
        self.slotName = slotName
        #print 'Created ROS channel'
    
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
            print "teimstamp", timestamp
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
        return structureChanged
                
    # Recursively gets the tree item corresponding to path
    # Only valid for leaves (used for validating user-input paths)                
    def getSlot(self, path):
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
    
    #def subscribeTopic(self, topicName, topicType):
        #if self.topicsSubs.has_key(topicName):
            ## Registration already done.
            #return
        #self.topicsSubs[topicName] = RosSubscriber()
       
    #def registerTopic(self,topicName, topicTypeName, callback):
        #"""Register to the topic using only the names (strings) of the topic, type."""
        ## TODO: encapsulate with exceptions
        ## Loading the packageName
        ## TODO: how to check if it is already loaded?
        #print 'topic type: %s' % topicTypeName
        #(modName, messageName) = messageNames(topicTypeName)
        
        #importMessageModule(modName, messageName)
        #exec('import %s' % messageName)
        
        #print eval(messageName)
        #rospy.TopicSub(topicName, eval(messageName), callback)
        
    def subscribe(self, itemPath, style):
        
        item = self._getItem(itemPath)
        topicName = self._getTopic(itemPath)
        assert topicName in self.topics
        if not self.topics[topicName].isRegistered:
            self.topicsSubs[topicName] = RosSubscriber(self.topics[topicName])
            messageName, messageModName, messageType = self.topics[topicName].qualifiedMessageType
            print messageType
            print self._getTopic(itemPath)
            exec('import %s' % messageModName)
            print eval(messageType)
            rospy.TopicSub(self._getTopic(itemPath), eval(messageType), self.topicsSubs[topicName].callback)
        channel = RosChannel(style, itemPath)
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
            print l
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
            print 'import %s' % messageModName
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
        print self.topics
        print 'done'
    
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
   
    
numericTypes=['byte','float32', 'float64', 'int32', 'int64', 'uint32', 'uint64']    


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
    print upp_com
    exec(upp_com)
    print messageModName
    exec('import %s' % messageModName)
    
    
#def getMessageInstance(rosMessageName):   
    #"""Returns an object of type rosMessageName, or None if it could not be found."""
    #(modName, messageName) = messageNames(rosMessageName)
    
    #importMessageModule(modName, messageName)
    #exec('import %s' % messageName)
    
    #command = 'myobj = %s()' % messageName
    #print command
    #exec(command)
    #return myobj
    
#def getMessageFloatSlots(rosMessageName):
    #"""Returns an object of type rosMessageName, or None if it could not be found."""
    ##(modName, messageName) = messageNames(rosMessageName)
    #obj = getMessageInstance(rosMessageName)
