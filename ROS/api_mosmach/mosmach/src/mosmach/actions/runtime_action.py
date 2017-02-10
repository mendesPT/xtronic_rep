#!/usr/bin/env python

# Description: Time Condition is an action designed to change the state after
#              a user-defined runtime in seconds. When the runtime is met, the MonarchState 
#              will change. 

import rospy
import time

from mosmach.state_action import StateAction

class RunTimeAction(StateAction):

    def __init__(self, monarchState, runtime):
        # TopicCondition initialization
        """Constructor.
        @type monarchState: MonarchState
        @type runtime: integer
        @param monarchState: the MonarchState to which this action belongs
        @param runtime: runtime in seconds"""
        super(RunTimeAction, self).__init__(monarchState, runtime)
        self.run_time = runtime

    def execute(self):

        self.time_zero = time.time()

        while True:   
            self.time_now = time.time() - self.time_zero

            if self.time_now >= self.run_time:
                super(RunTimeAction, self).notify_action(True)
                break

            else:
                rospy.sleep(0.5)

    def stop(self):
        return True
