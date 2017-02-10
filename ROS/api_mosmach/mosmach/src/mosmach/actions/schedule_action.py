#!/usr/bin/env python

# Description: Schedule action is a state action designed to wait for a pre-defined time of the day


import rospy
from mosmach.state_action import StateAction
from monarch_msgs.msg import *
from time import gmtime, strftime



class ScheduleAction(StateAction):

    def __init__(self, monarchState, setTime='09:15', data_cb='', is_dynamic=False):
        # ScheduleAction initialization
        """Constructor.
        @type monarchState: an object of type MonarchState
        @type setTime: string
        @type data_cb: function
        @type is_dynamic: boolean
        @param monarchState: the MonarchState to which this action belongs
        @param setTime: the time of the day (HH:MM) when this action will be succeeded
        @param data_cb: callback function for dynamic data returning ArmsControl messsage
        @param is_dynamic: check if the data is dynamic, if True pass a fuction instead of parameters individually."""
        super(ScheduleAction, self).__init__(monarchState, data_cb)
        rospy.sleep(1)

        self.setTime = setTime
        self.is_dynamic = is_dynamic

    def execute(self):
        # ScheduleAction execute
        rate = rospy.Rate(1)

        if self.is_dynamic:
            self.setTime = self._condition(self._userdata)

        while True:
            actual_time = strftime("%H:%M")

            if self.setTime == actual_time:
                break
            else:
                rospy.sleep(0.1)

        rate.sleep()

        super(ScheduleAction, self).notify_action(True)

    def stop(self):
        return