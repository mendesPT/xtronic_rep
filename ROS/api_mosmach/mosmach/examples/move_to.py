#!/usr/bin/env python

import roslib; roslib.load_manifest('mosmach')
import rospy
import smach
import smach_ros

from mosmach.monarch_state import MonarchState
from mosmach.actions.move_to_action import MoveToAction

class MoveToActionState(MonarchState):

  def __init__(self):
    MonarchState.__init__(self, state_outcomes=['succeeded'])
    rospy.loginfo("NavMoveToActionState")
    
    moveToAction = MoveToAction(self, -92.35, -83.90, -0.11)
    self.add_action(moveToAction)

   
######### Main function
def main():
  rospy.init_node("move_to")

  # State Machine Patrolling
  sm = smach.StateMachine(outcomes = ['succeeded','preempted','aborted'])

  with sm:
    sm.add('MoveTo', MoveToActionState())

  #sis = smach_ros.IntrospectionServer('move_to', sm, '/SM_ROOT')
  #sis.start()
  
  outcome = sm.execute()


if __name__ == '__main__':
  main()