#!/usr/bin/env python

# Description: SAM condition is a change condition designed to create a SAM reader 
#              for a user-defined SAM slot. It receives the SAM slot data and if the
#              condition to change the state is met, the MonarchState will change.


from mosmach.change_condition import ChangeCondition
from sam_helpers.reader import SAMReader
from sam_helpers.writer import SAMWriter


class SamCondition(ChangeCondition):

    def __init__(self, monarchState, samSlotName, samAgentName, condition):
        # SamCondition initialization
        """Constructor.
        @type monarchState: MonarchState
        @type samSlotName: string
        @type samAgentName: string
        @type condition: function
        @param monarchState: the MonarchState to which this action belongs
        @param samSlotName: the samSlotName to which this action belongs
        @param samAgentName: the samAgentName to which this action belongs
        @param condition: the function that will act as the sam condition callback and trigger the MonarchState change"""
        super(SamCondition, self).__init__(monarchState, condition)
        self.samSlotName = samSlotName
        self.samAgentName = samAgentName
        self.samReader = None

    def execute(self):
        self.samReader = SAMReader(self.samSlotName, self.execute_cb, self.samAgentName)

    def execute_cb(self, data):
        value = self._condition(data, self._userdata)
        self.stop()
        super(SamCondition, self).notify_state(value)

    def stop(self):
        self.samReader.remove()
        return True


