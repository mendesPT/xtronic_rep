#!/usr/bin/env python

# Description: Topic reader action is a state action designed to create a ROS topic subscriber
#              for a user-defined topic. It receives the ROS topic data and notifies the
#              MonarchState sending that data.


import rospy

from mosmach.state_action import StateAction
from geometry_msgs.msg import *
from std_msgs.msg import *
from monarch_msgs.msg import *


class TopicReaderAction(StateAction):

	def __init__(self, monarchState, topicName, topicMsgType, condition):
		# TopicReaderAction initialization
		"""Constructor.
		@type monarchState: an object of type MonarchState
		@type topicName: string
		@type topicMsgType: an object of topic message type
		@type condition: function
		@param monarchState: create this action inside of that monarchState
		@param topicName: name of the topic to subscribe
		@param topicMsgType: type of the message to read
		@param condition: the function that will act as the topic reader callback and enable to save the data in a user-defined variable."""
		super(TopicReaderAction, self).__init__(monarchState, condition)
		self.topicName = topicName
		self.topicMsgType = topicMsgType
		self.sub = ''

	def execute(self):
		self.sub = rospy.Subscriber(self.topicName, self.topicMsgType, self.execute_cb)

	def execute_cb(self, data):
		value = self._condition(data, self._userdata)
		super(TopicReaderAction, self).notify_action(True)

	def stop(self):
		if self.sub != '':
			self.sub.unregister()
		return