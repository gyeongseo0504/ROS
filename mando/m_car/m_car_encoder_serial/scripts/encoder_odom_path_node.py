#!/usr/bin/env python
import rospy
import numpy as np

from nav_msgs.msg import Path
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool
from geometry_msgs.msg import PoseStamped

path     = Path()
old_odom = Odometry()


def odom_cb(data):
    global path
    global old_odom 
    
    
    #dist = old_odom.pose.pose.position.x
    dist = (old_odom.pose.pose.position.x - data.pose.pose.position.x)**2 + (old_odom.pose.pose.position.y - data.pose.pose.position.y)**2
    dist = dist**(1/2)
    
    if(dist > 0.2):
		path.header = data.header
		pose = PoseStamped()
		pose.header = data.header
		pose.pose = data.pose.pose
		path.poses.append(pose)
		old_odom = data 
		path_pub.publish(path)
		
    
    
def reset_path_cb(data):
    
    path.poses=[]
    print("REST")
	    

rospy.init_node('encoder_path_node')

odom_sub = rospy.Subscriber('/odom/car', Odometry, odom_cb)
reset_odom_sub = rospy.Subscriber('/reset_path', Bool, reset_path_cb)
path_pub = rospy.Publisher('/car_path', Path, queue_size=10)

if __name__ == '__main__':
    rospy.spin()
