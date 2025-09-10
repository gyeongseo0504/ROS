#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from std_msgs.msg import Float32MultiArray
import math
import time

if __name__ == '__main__':
    rospy.init_node('steering_sine_test')
    pub = rospy.Publisher('/can_control_cmd', Float32MultiArray, queue_size=10)
    
    # 사인파 설정
    Hz = 1.0                 # 주파수 (2Hz)
    steering_min = 69.0
    steering_max = 111.0
    center = (steering_max + steering_min) / 2.0
    amplitude = (steering_max - steering_min) / 2.0
    rpm = 0.0                # 속도는 고정 (필요시 조정)
    
    rate = rospy.Rate(100)   # 100Hz 루프 (충분히 빠르게)
    t0 = time.time()
    
    while not rospy.is_shutdown():
        t = time.time() - t0
        # 사인파: sin(2πft)
        steering = center + amplitude * math.sin(2 * math.pi * Hz * t)
        msg = Float32MultiArray()
        msg.data = [rpm, steering]
        pub.publish(msg)
        print("\r[TEST] steering=%.2f°" % steering, end='')
        rate.sleep()

