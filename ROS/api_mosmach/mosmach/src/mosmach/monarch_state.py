#!/usr/bin/env python

# Description: MonarchState is an extended state from ROS smach.State designed to use
#              the ffmosmach state actions and change conditions.


import threading
import rospy
import smach
import smach_ros

class MonarchState(smach.State):
    """Class that extends the smach.State class and provides specialized funtionality for the MOnarCH system
    """
    def __init__(self, state_outcomes, input_keys=[''], output_keys=['']):
        """Constructor.
        @type state_outcomes: array of strings
        @type input_keys: array of strings
        @type output_keys: array of strings
        @param state_outcomes: list of possible outcomes for this state
        @param input_keys: list of input_keys for this state
        @param output_keys: list of output_keys for this state"""
        smach.State.__init__(self, state_outcomes, input_keys, output_keys)
        rospy.loginfo('Init MonarchState')

        self.execute_semaphore = threading.Semaphore(0)
        self.state_lock = threading.Lock()

        self.running = False        

        self.action_notify = False
        self.action_finishing_action = ''
        self.action_list = []
        self.action_thread_dict = dict()
        self.action_thread_finished = dict()
        
        self.change_condition_notify = False
        self.change_condition_dict = dict()
        self.change_condition_active_cond = ''
        self.change_condition_list = []

        self.preempt_notify = False
        self.preempt_thread_finished = True
        self.preempt_thread = ''

    def execute(self, userdata):
        """State execute method, as defined in SMACH."""

        self.state_lock.acquire()
        self.running = True

        # MonarchState execute
        rospy.loginfo('Exec MonarchState')
        
        self.action_notify = False
        self.action_finishing_action = ''
        self.action_thread_dict = dict()
        self.action_thread_finished = dict()

        self.change_condition_notify = False

        self.preempt_notify = False

        # Create preempt thread
        if self.preempt_thread_finished == True:
            self.preempt_thread = threading.Thread(target = self.notify_preempt)
            self.preempt_thread_finished = False
            self.preempt_thread.start()
            

        # Set and execute change condition userdata
        for c in self.change_condition_list:
           super(c.__class__,c).set_userdata(userdata)
           c.execute()

        # Create action threads and set userdata
        for a in self.action_list:
            super(a.__class__,a).set_userdata(userdata)
            action_thread = threading.Thread(target = a.execute)
            self.action_thread_dict[a] = action_thread
            self.action_thread_finished[a] = False
            action_thread.start()

        self.state_lock.release()

        while True:
            # wait for notification from change conditions/actions or preemption
            self.execute_semaphore.acquire()
            self.state_lock.acquire()

            if (self.preempt_notify):
                rospy.loginfo("Got news from preemption!")
                self.service_preempt()
                self.preempt_notify = False       
                

                self.running = False

                for cc in self.change_condition_list:
                    # stop all ccs
                    cc.stop()

                for a, f in self.action_thread_finished.iteritems():
                    if f is False:
                        # stop current actions
                        a.stop()

                self.preempt_thread_finished = True
                self.state_lock.release()

                self.preempt_thread.join()

                return 'preempted'


            if(self.change_condition_notify):
                rospy.loginfo("Got news from change condition!")
                outcome = self.change_condition_dict[self.change_condition_active_cond]
                self.change_condition_notify = False
                self.running = False

                # stop current actions
                for a, f in self.action_thread_finished.iteritems():
                    if f is False:
                        #action still not finished, send stop
                        a.stop()

                #for cc in self.change_condition_list:
                for cc in self.change_condition_list:
                    #change condition succeeded, stop all ccs
                    cc.stop()

                self.preempt_thread_finished = True
                self.state_lock.release()

                self.preempt_thread.join()

                return outcome


            if(self.action_notify):
                rospy.loginfo("Got news from Action!")
                self.action_thread_finished[self.action_finishing_action] = True
                self.action_notify = False

                all_finished = True
                #check if all actions finished
                for a, f in self.action_thread_finished.iteritems():
                    all_finished = all_finished & f
                
                if all_finished:  

                    for a, f in self.action_thread_finished.iteritems():
                        # stop all actions
                        a.stop()

                    # stop current change conditions
                    for cc in self.change_condition_list:
                        cc.stop()


                    self.running = False
                    self.preempt_thread_finished = True
                    self.state_lock.release()

                    self.preempt_thread.join()
                    
                    return 'succeeded'

                self.state_lock.release()

    
    def add_action(self,action):
        """Adds an action to the state, that will be done when the state is executed. This must be called before state execution.
            @type action: object derived from StateAction
            @param action: the action to be added
        """
        rospy.loginfo('MonarchState add action')
        self.action_list.append(action)


    def add_change_condition(self, change_condition, outcome):
        """Adds a change condition to be monitored while the state is executing. This must be called before state execution.
            @type change_condition: object derived from ChangeCondition
            @param change_condition: the change condition to be added
        """
        rospy.loginfo('MonarchState change condition')
        self.change_condition_dict[change_condition] = outcome
        self.change_condition_list.append(change_condition)


    def notify_change_condition(self, change_condition, value):
        """Notifies the state that a ChangeCondition has performed an evaluation.
            Should not be called directly from external code.
        """

        if(self.running):
            self.state_lock.acquire()

            self.change_condition_notify = True
            self.change_condition_active_cond = change_condition
            outcomes = self.change_condition_dict[self.change_condition_active_cond]
            self.change_condition_dict[self.change_condition_active_cond] = value
            value = None

            self.state_lock.release()
            self.execute_semaphore.release()


    def notify_action(self, action):
        """Notifies the state that a StateAction has finished.
            Should not be called directly from external code.
        """

        if(self.running):
            self.state_lock.acquire()
            
            self.action_notify = True
            self.action_finishing_action = action

            self.state_lock.release()
            self.execute_semaphore.release()



    def notify_preempt(self):
        """Notifies the state that a preempt action was requested.
            Should not be called directly from external code.
        """
        while self.preempt_thread_finished == False:
            if(self.preempt_requested()):
                self.state_lock.acquire()
                self.preempt_notify = True

                self.state_lock.release()
                self.execute_semaphore.release() 
                break
            else:
                rospy.sleep(0.1)