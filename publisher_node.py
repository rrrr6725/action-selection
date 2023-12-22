#!/usr/bin/env python

import rospy
from std_msgs.msg import Int32

def publisher():
    rospy.init_node('publisher_node')
    pub = rospy.Publisher('input_value', Int32, queue_size=10)

    while not rospy.is_shutdown():
        try:
            value = int(input("Enter 0 or 1: "))  
            if value in [0, 1]:
                rospy.loginfo("Publishing: %d", value)
                pub.publish(value)
            else:
                rospy.logwarn("Invalid input. Please enter 0 or 1.")
        except ValueError:
            rospy.logwarn("Invalid input. Please enter a number.")

if __name__ == '__main__':
    try:
        publisher()
    except rospy.ROSInterruptException:
        pass
