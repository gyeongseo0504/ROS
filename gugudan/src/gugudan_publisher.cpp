#!/usr/bin/env python

import rospy
from std_msgs.msg import Int32

def gugudan_publisher():
    rospy.init_node('gugudan_publisher', anonymous=True)
    gugudan_pub = rospy.Publisher('gugudan_results', Int32, queue_size=10)
    rate = rospy.Rate(1) 

    while not rospy.is_shutdown():
        for i in range(1, 10):
            result = 9 * i  
            rospy.loginfo("Publishing: 9 x %d = %d", i, result)
            gugudan_pub.publish(result)
            rate.sleep()

if __name__ == '__main__':
    try:
        gugudan_publisher()
    except rospy.ROSInterruptException:
        pass
