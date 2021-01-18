#!/usr/bin/env python
# encoding: utf-8

from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
import rospy
import math
import tf
from geometry_msgs.msg import PointStamped, PoseStamped
import actionlib
from move_base_msgs.msg import *
import time


arr1=[]
arr2=[]

def divide(S):
	for i in range(0,len(S)):
		if(S[i]=='.'):
			arr1.append(i)

def turn_int(S,Start,End):
	res=0
	x=int(Start+1)
	y=int(End)
	for i in range(x,y):
		res=res*10
		res=res+int(S[i])
	return res

def cal(S):
	divide(S)
	for i in range(0,len(arr1)-1):
		arr2.append(turn_int(S,arr1[i],arr1[i+1]))

def pose_callback(msg):
    with open("/home/lz/catkin_ws/src/turn_on_wheeltec_robot/point_time.txt", "r") as f:
        data=f.read()
	cal(data)
    global try_again, index, add_more_point, try_again, index, try_again, index, add_more_point, try_again, index
    if msg.status.status == 3:              #表示在终端状态下操作服务器已成功完成目标
        try_again = 1
        if add_more_point == 0:             #当计数点等于目标点，完成目标点
            print 'Reach the target point'
        if index < count:                   #目标巡航未完成，count表示目标点计数，index表示已完成的目标点计数
            pose = PoseStamped()
            pose.header.frame_id = 'map'
            pose.header.stamp = rospy.Time.now()
            pose.pose.position.x = markerArray.markers[index].pose.position.x
            pose.pose.position.y = markerArray.markers[index].pose.position.y
            #kaishi
            if index == 0:
                pose.pose.orientation.w=1
            if index != 0:
                x2=markerArray.markers[index].pose.position.x
                y2=markerArray.markers[index].pose.position.y
                x1=markerArray.markers[index-1].pose.position.x
                y1=markerArray.markers[index-1].pose.position.y
                v1=(1,0)
                v2=(x2-x1,y2-y1)
                yaw=math.acos((x2-x1)/math.sqrt((x2-x1)*(x2-x1)+(y2-y1)*(y2-y1)))
                if y2-y1< 0:
                    yaw = -1*yaw
                q=tf.transformations.quaternion_from_euler(0,0,yaw)
                pose.pose.orientation.z = q[2]
                pose.pose.orientation.w = q[3] #jieshu 
            #pose.pose.orientation.w = 1#1
            dexin=index
            print 'index < count:睡眠前index-1=',index-1,' count=',count,' 此次休眠时间为',arr2[dexin-1]
            rospy.sleep(arr2[dexin-1])
            goal_pub.publish(pose)
            index += 1
            print 'index < count:睡眠后index-1=',index-1,' count=',count
        elif index == count:                #表示目标点完成，当index等于count时，开始巡航
            print 'Complete instructions'
            index = 0;
            pose = PoseStamped()
            pose.header.frame_id = 'map'
            pose.header.stamp = rospy.Time.now()
            pose.pose.position.x = markerArray.markers[index].pose.position.x
            pose.pose.position.y = markerArray.markers[index].pose.position.y
            #kaishi
            if index == 0:
                pose.pose.orientation.w=1
            if index != 0:
                x2=markerArray.markers[index].pose.position.x
                y2=markerArray.markers[index].pose.position.y
                x1=markerArray.markers[index-1].pose.position.x
                y1=markerArray.markers[index-1].pose.position.y
                v1=(1,0)
                v2=(x2-x1,y2-y1)
                yaw=math.acos((x2-x1)/math.sqrt((x2-x1)*(x2-x1)+(y2-y1)*(y2-y1)))
                if y2-y1< 0:
                    yaw = -1*yaw
                q=tf.transformations.quaternion_from_euler(0,0,yaw)
                pose.pose.orientation.z = q[2]
                pose.pose.orientation.w = q[3] #jieshu 
            #pose.pose.orientation.w = 1#1
            dexin=index
            print 'index == count:睡眠前index+count-1=',index+count-1,' count=',count,' 此次休眠时间为',arr2[dexin+count-1]
            rospy.sleep(arr2[dexin+count-1])
            goal_pub.publish(pose)
            add_more_point = 1
            index+=1
            print 'index == count:睡眠后index=',index,' count=',count
        
    else:                                   #未抵达设定的目标点
        print 'Not reaching the target point:', msg.status.status, ' please try again !'
        if try_again == 1:
            pose = PoseStamped()
            pose.header.frame_id = 'map'
            pose.header.stamp = rospy.Time.now()
            pose.pose.position.x = markerArray.markers[index - 1].pose.position.x           #前往尚未抵达的目标点
            pose.pose.position.y = markerArray.markers[index - 1].pose.position.y
            pose.pose.orientation.w = 1#1
            goal_pub.publish(pose)
            try_again = 0
            print '1  index=',index
        elif index < len(markerArray.markers):      #若还未完成目标点
            pose = PoseStamped()
            pose.header.frame_id = 'map'
            pose.header.stamp = rospy.Time.now()
            pose.pose.position.x = markerArray.markers[index].pose.position.x               #结果为未完成目标，一直在
            pose.pose.position.y = markerArray.markers[index].pose.position.y
            pose.pose.orientation.w = 1#1
            goal_pub.publish(pose)
            index += 1
            print '2  index=',index


def press_callback(msg):           #点击一个点的回调函数
    global index, add_more_point, count, index, add_more_point, count
    marker = Marker()                       #marker 赋值
    marker.header.frame_id = 'map'
    marker.type = marker.TEXT_VIEW_FACING
    marker.action = marker.ADD
    marker.scale.x = 1
    marker.scale.y = 1
    marker.scale.z = 1
    marker.color.a = 1
    marker.color.r = 1
    marker.color.g = 0
    marker.color.b = 0
    marker.pose.orientation.w = 1#1
    marker.pose.position.x = msg.point.x
    marker.pose.position.y = msg.point.y
    marker.pose.position.z = msg.point.z
    marker.text = str(count)
    markerArray.markers.append(marker)
    id = 0
    for m in markerArray.markers:       #遍历记录点的个数
        m.id = id
        id += 1
    
    mark_pub.publish(markerArray)
    if count == 0:
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = rospy.Time.now()
        pose.pose.position.x = msg.point.x
        pose.pose.position.y = msg.point.y
        pose.pose.orientation.w = 1#1
        goal_pub.publish(pose)
        index += 1
        print 'start index=',index
    if add_more_point and count > 0:        #增加新目标点
        add_more_point = 0
        move = MoveBaseActionResult()
        move.status.status = 3              #表示在终端状态下操作服务器已成功完成目标
        move.header.stamp = rospy.Time.now()
        goal_status_pub.publish(move)

    count += 1
    print 'Add a new target point'


def send_mark():
    global markerArray, count, index, add_more_point, try_again, mark_pub, goal_pub, goal_status_pub, markerArray, count, index, add_more_point, try_again, mark_pub, goal_pub, goal_status_pub
    markerArray = MarkerArray()
    count = 0
    index = 0
    add_more_point = 0
    try_again = 1
    rospy.init_node('path_point_demo')
    mark_pub = rospy.Publisher('/path_point', MarkerArray, queue_size = 100)
    click_sub = rospy.Subscriber('/clicked_point', PointStamped, press_callback)
    goal_pub = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size = 1)
    goal_status_sub = rospy.Subscriber('/move_base/result', MoveBaseActionResult, pose_callback)
    goal_status_pub = rospy.Publisher('/move_base/result', MoveBaseActionResult, queue_size = 1)
    rospy.spin()

if __name__ == '__main__':

    send_mark()


