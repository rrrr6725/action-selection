#!/usr/bin/env python

# Importing necessary modules
import rospy
import smach
import smach_ros
from std_msgs.msg import Int32

# Definition of different states and their functionalities

# Wait State: Pauses the execution for 5 seconds
class Wait(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['next'])

    def execute(self, userdata):
        rospy.loginfo('Wait for 5 seconds')  
        rospy.sleep(5.0) 
        return 'next'

# Initialize State: Subscribes to a topic and processes received messages
class Initialize(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['done', 'exit'])
        self.sub_value = None 
        self.subscriber = rospy.Subscriber('input_value', Int32, self.callback)
        
    def callback(self, data):
        self.sub_value = data.data
        rospy.loginfo("Received: %d", self.sub_value)
        
        if self.sub_value == 1:
            self.subscriber.unregister() 
            self.subscriber = None  
            self.execute_transition = 'done'
        elif self.sub_value == 0:
            self.subscriber.unregister() 
            self.subscriber = None  
            self.execute_transition = 'exit'
        else:
            rospy.logwarn('Invalid input. Please enter 0 or 1.')

    def execute(self, userdata):
        self.execute_transition = None  
        
        while not self.execute_transition:
            rospy.sleep(0.1)
        
        return self.execute_transition

# Search State: Simulates a searching action and iterates for a certain count
class Search(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['done', 'continue'])
        self.counter = 0

    def execute(self, userdata):
        rospy.loginfo('Searching...') 
        rospy.sleep(1.0) 
        if self.counter < 3: 
            self.counter += 1
            return 'continue'
        else:
            return 'done'  

# Harvest State: Simulates a harvesting action
class Harvest(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['done'])

    def execute(self, userdata):
        rospy.loginfo('Harvesting...')  
        rospy.sleep(1.0) 
        return 'done'

# Storage State: Simulates a storage action
class Storage(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['done'])

    def execute(self, userdata):
        rospy.loginfo('Storaging...')  
        rospy.sleep(1.0)  
        return 'done'

# Prepare State: Simulates a preparation action
class Prepare(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['done'])

    def execute(self, userdata):
        rospy.loginfo('Preparing...')  
        rospy.sleep(1.0)  
        return 'done'

# Main function
def main():
    # Initialize ROS node
    rospy.init_node('smach_modes')

    # Create top-level state machine
    sm_top = smach.StateMachine(outcomes=['failure', 'success'])

    # State Machine definitions and transitions
    with sm_top:
        smach.StateMachine.add('WAIT', Wait(), transitions={'next':'INITIALIZE_MODE'})

        sm_initialize = smach.StateMachine(outcomes=['next', 'failed'])
        with sm_initialize:
            smach.StateMachine.add('INITIALIZE', Initialize(), transitions={'done': 'next', 'exit': 'failed'}) 
        smach.StateMachine.add('INITIALIZE_MODE', sm_initialize, transitions={'next': 'SEARCH_MODE', 'failed': 'failure'})
    
        sm_search = smach.StateMachine(outcomes=['next', 'succeeded'])
        with sm_search:
            smach.StateMachine.add('SEARCH', Search(), transitions={'done': 'succeeded', 'continue': 'next'})
        smach.StateMachine.add('SEARCH_MODE', sm_search, transitions={'next': 'HARVEST_MODE', 'succeeded': 'success'})

        sm_harvest = smach.StateMachine(outcomes=['next'])
        with sm_harvest:
            smach.StateMachine.add('HARVEST', Harvest(), transitions={'done': 'next'})
        smach.StateMachine.add('HARVEST_MODE', sm_harvest, transitions={'next':'STORAGE_MODE'})

        sm_storage = smach.StateMachine(outcomes=['next'])
        with sm_storage:
            smach.StateMachine.add('STORAGE', Storage(), transitions={'done': 'next'})
        smach.StateMachine.add('STORAGE_MODE', sm_storage, transitions={'next':'PREPARE_MODE'})

        sm_prepare = smach.StateMachine(outcomes=['next'])
        with sm_prepare:
            smach.StateMachine.add('PREPARE', Prepare(), transitions={'done': 'next'})
        smach.StateMachine.add('PREPARE_MODE', sm_prepare, transitions={'next':'SEARCH_MODE'})

    # Start introspection server for visualization and debugging
    sis = smach_ros.IntrospectionServer('smach_server', sm_top, '/START')
    sis.start()

    # Execute the state machine and spin
    outcome = sm_top.execute()
    rospy.spin()

    # Stop introspection server
    sis.stop()

if __name__ == '__main__':
    main()
