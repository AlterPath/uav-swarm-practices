#!/usr/bin/env python3
import rospy
from std_msgs.msg import String
import time

rospy.init_node('pc2_publisher')
pub = rospy.Publisher('/pc2_data', String, queue_size=10)
rate = rospy.Rate(15)  # 0.5 Hz

counter = 0
while not rospy.is_shutdown():
    message = f"Response from CLIENT 172.20.10.11 - {counter}"
    rospy.loginfo(f"CLIENT sending: {message}")
    pub.publish(message)
    counter += 1
    rate.sleep()
