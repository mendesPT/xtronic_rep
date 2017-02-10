#!/usr/bin/env python

# Description: Move To action is a state action designed to create a simple actionlib client 
#              using the MoveBaseAction. By defining a target goal, the MOnarCH robot will 
#              move and reach that goal.


import rospy
import actionlib

from mosmach.state_action import StateAction
from mosmach.util import pose2pose_stamped
from move_base_msgs.msg import *
from smach_ros import simple_action_state


class MoveToAction(StateAction):

    def __init__(self, monarchState, x=0, y=0, theta=0, data_cb='', is_dynamic=False, result_cb=''):
        # MoveToAction initialization
        """Constructor.
        @type monarchState: an object of type MonarchState
        @type x: integer
        @type y: integer
        @type theta: integer
        @type data_cb: function
        @type is_dynamic: boolean
        @type result_cb: function
        @param monarchState: the MonarchState to which this action belongs
        @param x: the x position of goal which the robot will move to
        @param y: the y position of goal which the robot will move to
        @param theta: the theta orientation of goal which the robot will move with
        @param data_cb: callback function for dynamic goal returning a MoveBaseGoal message
        @param is_dynamic: check if the message is dynamic, if True pass a function instead of parameters individually
        @param result_cb: callback function when action is completed."""
        super(MoveToAction, self).__init__(monarchState, data_cb)
        self.x = x
        self.y = y
        self.theta = theta
        self.is_dynamic = is_dynamic
        self.result_cb = result_cb
        self.state = ''

        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
    
    def execute(self):
        # MoveToAction execute
        target_goal = pose2pose_stamped(self.x, self.y, self.theta)  

        if self.is_dynamic:
            target_goal = self._condition(self._userdata)

        self.client.wait_for_server()
        self.client.send_goal_and_wait(MoveBaseGoal(target_pose=target_goal))
        self.state = self.client.get_state()

        if not self.result_cb == '':
            self.result_cb(self._userdata, self.state)

        super(MoveToAction, self).notify_action(True)

    def stop(self):
        # send actionlib abort
        if self.client.simple_state == 2:
            return True
        else:
            self.client.cancel_goal()
            return False