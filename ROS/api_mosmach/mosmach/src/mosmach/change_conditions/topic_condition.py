#!/usr/bin/env python

# Description: Topic condition is a change condition designed to create a ros topic 
#              subscriber for a user-defined topic name. It receives the topic data 
#              and if the condition to change the state is met, the MonarchState will 
#              change. 


import rospy

from mosmach.change_condition import ChangeCondition
from geometry_msgs.msg import *
from std_msgs.msg import *
from monarch_msgs.msg import *


class TopicCondition(ChangeCondition):

    def __init__(self, monarchState, topicName, topicMsgType, condition):
        # TopicCondition initialization
        """Constructor.
        @type monarchState: MonarchState
        @type topicName: string
        @type topicMsgType: msgs type
        @type condition: function
        @param monarchState: the MonarchState to which this action belongs
        @param topicName: the topicName to which this action belongs
        @param topicMsgType: the msg type of this action
        @param condition: the function that will act as the topic condition callback and trigger the MonarchState change"""
        super(TopicCondition, self).__init__(monarchState, condition)
        self.topicName = topicName
        self.topicMsgType = topicMsgType
        self.sub = None

    def execute(self):
        self.sub = rospy.Subscriber(self.topicName, self.topicMsgType, self.execute_cb)

    def execute_cb(self, data):
        value = self._condition(data, self._userdata)
        super(TopicCondition, self).notify_state(value)

    def stop(self):
        self.sub.unregister()
        return True
