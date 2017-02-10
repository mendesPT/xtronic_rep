#!/usr/bin/env python

import roslib; roslib.load_manifest('mosmach')
import rospy
import smach
import smach_ros

from mosmach.monarch_state import MonarchState
from mosmach.actions.move_to_action import MoveToAction
from mosmach.change_conditions.topic_condition import TopicCondition

from monarch_msgs.msg import RfidReading


# Patrolling State Machine
class MoveToOneState(MonarchState):
	def __init__(self):
		MonarchState.__init__(self, state_outcomes=['succeeded','preempted'],input_keys=['user_goal'], output_keys=['user_goal'])
		rospy.loginfo("Init MoveTo1State")

		navMoveToOneAction = MoveToAction(self, -92.35, -83.90, -0.11)
		self.add_action(navMoveToOneAction)


class MoveToTwoState(MonarchState):
	def __init__(self):
		MonarchState.__init__(self, state_outcomes=['succeeded','preempted'], input_keys=['user_goal'], output_keys=['user_goal'])
		rospy.loginfo("Init MoveTo2State")

		navMoveToTwoAction = MoveToAction(self, -88.95, -84.80, 3.03)
		self.add_action(navMoveToTwoAction)


# Get RFID tag State Machine
class GetRfidTagState(MonarchState):
	def __init__(self):
		MonarchState.__init__(self, state_outcomes=['succeeded','preempted'], input_keys=['user_goal'], output_keys=['user_goal'])
		rospy.loginfo("Init GetRfidTagState")
		topicName = 'rfid_tag'
		conditionOutcomes = ['succeeded']
		topicCondition = TopicCondition(self, topicName, RfidReading, self.rfidCondition)
		self.add_change_condition(topicCondition, conditionOutcomes)

	def rfidCondition(self, data, userdata):
		print "TopicCondition userdata"
		rospy.loginfo('user_goal = '+str(userdata.user_goal))
		value = 'succeeded'
		return value

# Run Dock State Machine
class SendDockState(MonarchState):
	def __init__(self):
		MonarchState.__init__(self, state_outcomes=['succeeded','preempted'], input_keys=['user_goal'], output_keys=['user_goal'])
		rospy.loginfo("Init SendDockState")

		navSendToDockAction = MoveToAction(self, -90.10, -85.30, 1.45)
		self.add_action(navSendToDockAction)


# Main function
def main():
	rospy.init_node("patrolling_and_dock")

	# State Machine Patrolling   
	sm_patrolling = smach.StateMachine(outcomes = ['preempted','aborted'])
	sm_patrolling.userdata.sm_patrolling_data = 0
	with sm_patrolling:
		sm_patrolling.add('WAYPOINT1',MoveToOneState(),transitions = {'succeeded':'WAYPOINT2'}, remapping={'user_goal':'sm_patrolling_data'})
		sm_patrolling.add('WAYPOINT2',MoveToTwoState(),transitions = {'succeeded':'WAYPOINT1'}, remapping={'user_goal':'sm_patrolling_data'})


	# State Machine Get RFID
	sm_getrfid = smach.StateMachine(outcomes = ['succeeded','preempted','aborted'])
	sm_getrfid.userdata.sm_getrfid_data = 0
	with sm_getrfid:
		sm_getrfid.add('GETRFID',GetRfidTagState(),transitions={'succeeded':'succeeded','preempted':'succeeded'}, remapping={'user_goal':'sm_getrfid_data'})


	# State Machine Dock
	sm_dock = smach.StateMachine(outcomes = ['succeeded','preempted','aborted'])
	sm_dock.userdata.sm_dock_data = 0

	with sm_dock:
		sm_dock.add('DOCK',SendDockState(), remapping={'user_goal':'sm_dock_data'})


	# Concurrence State Machine: Patrolling + Get RFID	
	def termination_cm_cb(outcome_map):
		if outcome_map['GETRFID_SM'] == 'succeeded':
			return True
		if outcome_map['PATROLLING_SM'] == 'preempted':
			return True
		if outcome_map['PATROLLING_SM'] == 'aborted':
			return True
		else:
			return False

	def outcome_cm_cb(outcome_map):
		if outcome_map['GETRFID_SM'] == 'succeeded':
			return 'succeeded'
		if outcome_map['PATROLLING_SM'] == 'preempted':
			return 'preempted'
		if outcome_map['PATROLLING_SM'] == 'aborted':
			return 'aborted'
		else:
			return 'succeeded'


	cm = smach.Concurrence(outcomes = ['succeeded','preempted','aborted'],
						   default_outcome = 'succeeded',
						   child_termination_cb = termination_cm_cb,
						   outcome_cb = outcome_cm_cb)
	cm.userdata.cm_data = 0

	with cm:
		cm.add('PATROLLING_SM',sm_patrolling, remapping={'sm_patrolling_data':'cm_data'})
		cm.add('GETRFID_SM',sm_getrfid, remapping={'sm_getrfid_data':'cm_data'})

	# Main State Machine
	sm_msm = smach.StateMachine(outcomes=['succeeded','preempted','aborted'])
	sm_msm.userdata.sm_msm_data = 0

	with sm_msm:
		sm_msm.add("SIMPLEPATROLLING",cm,transitions={'succeeded':'DOCK_SM'},remapping={'cm_data':'sm_msm_data'})
		sm_msm.add("DOCK_SM",sm_dock,transitions={'succeeded':'DOCK_SM'},remapping={'sm_dock_data':'sm_msm_data'})

	#sis = smach_ros.IntrospectionServer('patrolling_and_dock', sm_msm, '/SM_ROOT')
	#sis.start()
	
	outcome = sm_msm.execute()


if __name__ == '__main__':
	main()
