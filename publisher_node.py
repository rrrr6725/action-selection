#!/usr/bin/env python

import rospy
from std_msgs.msg import Int32

def publisher():
    # Initialize the ROS node with the name 'publisher_node'
    rospy.init_node('publisher_node')
    
    # Create a publisher for the 'input_value' topic with Int32 type messages and a queue size of 10
    pub = rospy.Publisher('input_value', Int32, queue_size=10)

    # Continue running the loop until the rospy is shutdown
    while not rospy.is_shutdown():
        try:
            # Get user input as an integer (0 or 1)
            value = int(input("Enter 0 or 1: "))  

            # Check if the entered value is either 0 or 1
            if value in [0, 1]:
                # Log the information and publish the value on the 'input_value' topic
                rospy.loginfo("Publishing: %d", value)
                pub.publish(value)
            else:
                # Log a warning if the input is not 0 or 1
                rospy.logwarn("Invalid input. Please enter 0 or 1.")
        except ValueError:
            # Log a warning if the input is not a valid number
            rospy.logwarn("Invalid input. Please enter a number.")

if __name__ == '__main__':
    try:
        # Call the publisher function when the script is run
        publisher()
    except rospy.ROSInterruptException:
        # Handle a ROS interrupt exception, if it occurs
        pass
