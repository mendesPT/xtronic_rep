#!/usr/bin/env python

# Description: Change Condition was designed to enable the creation of specific change conditions
#              that tigger the change of the state from a MOnarCH behaviour state machine.


class ChangeCondition(object):
	"""A change condition that will trigger the state change"""
	def __init__(self, monarchState, condition):
		"""Constructor.
        @type monarchState: an object of type MonarchState.
        @type condition: a function
        @param monarchState: the MonarchState to which this action belongs.
        @param condition:the function that will act as a callback and will trigger the state change."""
		self._monarchState = monarchState
		self._condition = condition
		self._userdata = ''

	def notify_state(self, value):
		# notify monarch_state
		self._monarchState.notify_change_condition(self, value)

	def set_userdata(self, userdata):
		# set userdata
		self._userdata = userdata