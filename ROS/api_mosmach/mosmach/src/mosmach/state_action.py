#!/usr/bin/env python

# Description: State action was designed to enable the creation of specific robot actions
#              for the MOnarCH project.


class StateAction(object):
    """An action that can be performed within a state"""       
    def __init__(self, monarch_state, condition=''):
        """Constructor.
        @type monarch_state: an object of type MonarchState.
        @param monarchState: the MonarchState to which this action belongs"""
        self._monarchState = monarch_state
        self._condition = condition
        self._userdata = ''

    def notify_action(self, value):
        # notify monarch_state
        self._monarchState.notify_action(self)

    def set_userdata(self, userdata):
        # set userdata
        self._userdata = userdata