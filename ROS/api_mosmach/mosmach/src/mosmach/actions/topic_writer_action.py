#!/usr/bin/env python

# Description: Topic writer action is a state action designed to create a ROS topic publisher 
#              for a user-defined topic. It writes the data in the defined topic.


import rospy

from mosmach.state_action import StateAction
from geometry_msgs.msg import *
from std_msgs.msg import *
from monarch_msgs.msg import *


class TopicWriterAction(StateAction):
	
	def __init__(self, monarchState, topicName, topicMsgType, message, is_dynamic=False):
		# TopicWriterAction initialization
		"""Constructor.
		@type monarchState: an object of type MonarchState
		@type topicName: string
		@type topicMsgType: an object of topic message type
		@type message: topic message or function returning a topic message
		@param monarchState: create this action inside of that monarchState
		@param topicName: name of the topic to publish
		@param topicMsgType: type of the message to send
		@param message: set the message to send  / callback function for a dynamic message
		@param is_dynamic: check if the message is dynamic, if True pass a function instead of a message."""
		super(TopicWriterAction, self).__init__(monarchState, message)
		self.message = message
		self.is_dynamic = is_dynamic

		self.pub = rospy.Publisher(topicName, topicMsgType, queue_size=10)
		rospy.sleep(0.1)
		
	def execute(self):
		# TopicWriterAction execute
		rate = rospy.Rate(1)	

		if self.is_dynamic:
			self.message = self._condition(self._userdata)

		self.pub.publish(self.message)
		rate.sleep()
		super(TopicWriterAction, self).notify_action(True)

	def stop(self):
		#if self.pub != '':
		#	self.pub.unregister()
		return
