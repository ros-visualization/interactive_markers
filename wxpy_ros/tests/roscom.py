import rostools; rostools.load_manifest('wxpy_ros')

import rospy
rospy.get_published_topics()

import wxpy_ros

class MySubscriber:
    """Since only one callback should subscribe to each topic, this class subscribes """
    def __init__(self):
        self.channels = []
    
    def callback(self, state):
        #print 'call'
        for channel in self.channels:
            channel.callback(state)

class MyChannel:
    
    def callback(self, state):
        print 'received state ',(state.PosCmd)

class RosChannel(wxpy_ros.Channel):
    def __init__(self, style, slotName):
        wxpy_ros.Channel.__init__(self, style)
        self.slotName = slotName
        print 'Created ROS channel'
    
    def callback(self, state):
      self.addPoint(getattr(state, self.slotName))
      #print 'received ', self.slotName, getattr(state, self.slotName)

class RosMessageHandler:
    def __init__(self, ):
        self.topicsSubs = dict()
        wxpy_ros.ROSListener("My Node Name").start()  
        
    def subscribe(self, topicName, topicType, slotName, style):
        if not self.topicsSubs.has_key(topicName):
            self.topicsSubs[topicName] = MySubscriber()
            self.registerTopic(topicName, topicType, self.topicsSubs[topicName].callback)
        channel = RosChannel(style, slotName)
        self.topicsSubs[topicName].channels.append(channel)
        print self.topicsSubs[topicName].channels
        return channel
        
    def registerTopic(self,topicName, topicTypeName, callback):
        """Register to the topic using only the names (strings) of the topic, type."""
        # TODO: encapsulate with exceptions
        # Loading the packageName
        # TODO: how to check if it is already loaded?
        print 'topic type: %s' % topicTypeName
        (modName, messageName) = messageNames(topicTypeName)
        
        importMessageModule(modName, messageName)
        exec('import %s' % messageName)
        
        print eval(messageName)
        rospy.TopicSub(topicName, eval(messageName), callback)
        
        print 'registering done'
    
    
    #def messageNames(rosMessageName):
        #""" Given the name of the message as advertised by ROS, returns the name of the module and the fully qualified name of the message"""
        #l = rosMessageName.split('/')
        #modName = l[0]
        #messageName = l[0]+'.msg.'+l[1]
        #return (modName, messageName)
        
        
    #def importMessageModule(moduleName, messageName):
        #"""Given the fully qualified name of the message class, tries to import the class.
        #Will add some exception handling."""
        #upp_com = 'rostools.load_manifest(\'%s\')'%moduleName
        #print upp_com
        #exec(upp_com)
        #exec('import %s' % messageName)
        
        
    #def getMessageInstance(rosMessageName):   
        #"""Returns an object of type rosMessageName, or None if it could not be found"""
        #(modName, messageName) = messageNames(rosMessageName)
        
        #importMessageModule(modName, messageName)
        #exec('import %s' % messageName)
        
        #command = 'myobj = %s()' % messageName
        #print command
        #exec(command)
        #return myobj
    
def getTopicsList():
    return rospy.get_published_topics()

#def registerTopic(topicName, topicTypeName, callback):
    #"""Register to the topic using only the names (strings) of the topic, type."""
    ## TODO: encapsulate with exceptions
    ## Loading the packageName
    ## TODO: how to check if it is already loaded?
    #(modName, messageName) = messageNames(topicName)
    
    #importMessageModule(modName, messageName)
    ## This does not work:
    ##mod_msg = __import__(messageName, globals(), locals())
    ##print mod_msg 
    
    #callbackPyName = str(callback)
    ##print callbackPyName
    ##l = callbackPyName.split(' ')
    ##print l
    ##callbackName = l[4]+'.'+l[2]
    
    #command = 'rospy.TopicSub(\'%s\', %s, %s)' % (topicName, messageName, callbackPyName)
    #print command
    ##rospy.TopicSub('ARM_L_PAN_state', rosControllers.msg.RotaryJointState, sub.callback)
    #exec(command)


def messageNames(rosMessageName):
    """ Given the name of the message as advertised by ROS, returns the name of the module and the fully qualified name of the message.
    This assumes the module respects the naming conventions."""
    l = rosMessageName.split('/')
    modName = l[0]
    messageName = l[0]+'.msg.'+l[1]
    return (modName, messageName)
    
    
def importMessageModule(moduleName, messageName):
    """Given the fully qualified name of the message class, tries to import the message class.
    Will add some exception handling."""
    upp_com = 'rostools.load_manifest(\'%s\')'%moduleName
    print upp_com
    exec(upp_com)
    exec('import %s' % messageName)
    
    
def getMessageInstance(rosMessageName):   
    """Returns an object of type rosMessageName, or None if it could not be found."""
    (modName, messageName) = messageNames(rosMessageName)
    
    importMessageModule(modName, messageName)
    exec('import %s' % messageName)
    
    command = 'myobj = %s()' % messageName
    print command
    exec(command)
    return myobj
    
def getMessageFloatSlots(rosMessageName):
    """Returns an object of type rosMessageName, or None if it could not be found."""
    #(modName, messageName) = messageNames(rosMessageName)
    obj = getMessageInstance(rosMessageName)
    #slots = 
    
#if __name__ == '__main__':    
    #sub = MySubscriber()
    #sub.channels.append(MyChannel())
    
    #rostools.load_manifest('rosControllers')
    #import rosControllers.msg.RotaryJointState
    #rospy.TopicSub('ARM_L_PAN_state', rosControllers.msg.RotaryJointState, sub.callback)
    #registerTopic('ARM_L_PAN_state', 'rosControllers/RotaryJointState', 'sub.callback')
    
