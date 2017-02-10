#!/usr/bin/env python

import roslib; roslib.load_manifest('mosmach')
import rospy
import smach
import smach_ros

from mosmach.monarch_state import MonarchState
from mosmach.actions.move_arms_action import MoveArmsAction
from mosmach.actions.move_head_action import MoveHeadAction
from mosmach.actions.set_led_action import SetLedAction
from mosmach.actions.set_mouth_action import SetMouthAction
from monarch_msgs.msg import MouthLedControl
from monarch_msgs.msg import LedControl

class ResetRobot(MonarchState):
	
	def __init__(self):
		MonarchState.__init__(self, state_outcomes=['succeeded'])
		rospy.loginfo("ResetRobot")
		resetArmsAction = MoveArmsAction(self, False, 0, 0, 2)
		self.add_action(resetArmsAction)

		resetHeadAction = MoveHeadAction(self, 0, 50)
		self.add_action(resetHeadAction)

		resetLeftEyeLedState = SetLedAction(self, 0, 0, 0, 0, 0)
		self.add_action(resetLeftEyeLedState)

		resetRightEyeLedState = SetLedAction(self, 1, 0, 0, 0, 0)
		self.add_action(resetRightEyeLedState)

		resetCheeksLedState = SetLedAction(self, 2, 0, 0, 0, 0)
		self.add_action(resetCheeksLedState)

		resetBaseLedState = SetLedAction(self, 3, 0, 0, 0, 0)
		self.add_action(resetBaseLedState)

		resetBaseLeftLedState = SetLedAction(self, 4, 0, 0, 0, 0)
		self.add_action(resetBaseLeftLedState)

		resetBaseRightLedState = SetLedAction(self, 5, 0, 0, 0, 0)
		self.add_action(resetBaseRightLedState)

		mouthData = [0]*32*8
		resetMouth = SetMouthAction(self, mouthData)
		self.add_action(resetMouth)


######### Main function
def main():
  rospy.init_node("reset_robot")

  # State Machine Patrolling
  sm = smach.StateMachine(outcomes = ['succeeded','preempted','aborted'])

  with sm:
    sm.add('RESET',ResetRobot(),transitions = {'succeeded':'succeeded'})
  
  outcome = sm.execute()


if __name__ == '__main__':
  main()