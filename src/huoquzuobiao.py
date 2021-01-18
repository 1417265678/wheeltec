#!/usr/bin/env python
# encoding: utf-8
import rospy
from nav_msgs.msg  import Odometry
def callback(msg):  
    rospy.loginfo("Received a /odom message!")  
    rospy.loginfo("weizhi: [%f, %f, %f]"%(msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z))  
def listener():  
    rospy.init_node('odom_listener')  
    rospy.Subscriber("/odom", Odometry, callback)#/cmd_vel  
    rospy.spin() 
if __name__ == '__main__':  
    listener() 