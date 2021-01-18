#!/usr/bin/env python
# encoding: utf-8

from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
import rospy
import math
import tf
from geometry_msgs.msg import PointStamped, PoseStamped,PoseWithCovarianceStamped,PolygonStamped,Twist #各种话题信息所需数据类型
import actionlib
from move_base_msgs.msg import *
import time  
from nav_msgs.msg import Odometry
import message_filters
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
import os
def fabusudu(msg):  #向/client_messages2话题发布速度信息函数
    fs=str(msg.twist.twist.linear.x)   #转为字符串
    fs_pub.publish(fs)    #发布
def fabuweizhi(odom_sub):   #向/client_messages话题发布位置信息函数
    fw1=odom_sub.polygon.points[0].x   #横坐标
    fw2=odom_sub.polygon.points[0].y   #纵坐标
    fw=str(fw1)+" "+str(fw2)     #转为字符串后连接
    fw_pub.publish(fw)   #发布

def control_speed(odom_sub):      #不同距离不同速度函数
  if index==1 : # or index==5:    #最后一个点返回第一个点时
    p1=odom_sub.polygon.points[0].x
    p2=markerArray.markers[index-1].pose.position.x
    p3=markerArray.markers[count-1].pose.position.x
    q1=odom_sub.polygon.points[0].y
    q2=markerArray.markers[index-1].pose.position.y
    q3=markerArray.markers[count-1].pose.position.y
    juli1=(p2-p1)*(p2-p1)+(q2-q1)*(q2-q1)    #到目标点距离
    juli2=(p3-p1)*(p3-p1)+(q3-q1)*(q3-q1)    #到起始点距离
    if juli1<=juli2 :
    	juli = juli1
    else :
    	juli = juli2
    juli_sqrt=juli ** 0.5
    if juli<=4:     #2米内
            sd=MoveBaseActionResult()   #sd初始化为MoveBaseActionResult数据类型
            sd.status.status=8             #速度状态码为8
            sd.header.stamp=rospy.Time.now()
            sd_status_pub.publish(sd)    #发布给/sd_status话题
    if juli>4 and juli<=16:    #2米到4米
            sd=MoveBaseActionResult()
            sd.status.status=10
            sd.header.stamp=rospy.Time.now()
            sd_status_pub.publish(sd)
    if juli>16 and juli<=36:    #4米到6米
            sd=MoveBaseActionResult()
            sd.status.status=11
            sd.header.stamp=rospy.Time.now()
            sd_status_pub.publish(sd) 
    if juli>36:    #6米以上
            sd=MoveBaseActionResult()
            sd.status.status=12
            sd.header.stamp=rospy.Time.now()
            sd_status_pub.publish(sd)
    ##print "距离第" ,index ,"个点" ,juli_sqrt ,"m,sd 为" ,sd.status.status   #打印下一个点，当前速度，速度状态码
    ##print markerArray.markers[index-1].pose.position.x,markerArray.markers[index-1].pose.position.y  
    ##print odom_sub.polygon.points[0].x,odom_sub.polygon.points[0].y
  if index>1 : # or index==5:    #除了最后一个点返回第一个点
    p1=odom_sub.polygon.points[0].x
    p2=markerArray.markers[index-1].pose.position.x
    p3=markerArray.markers[index-2].pose.position.x
    q1=odom_sub.polygon.points[0].y
    q2=markerArray.markers[index-1].pose.position.y
    q3=markerArray.markers[index-2].pose.position.y
    juli1=(p2-p1)*(p2-p1)+(q2-q1)*(q2-q1)
    juli2=(p3-p1)*(p3-p1)+(q3-q1)*(q3-q1)
    if juli1<=juli2 :
        juli = juli1
    else :
        juli = juli2
    juli_sqrt=juli ** 0.5
    if juli<=4:
            sd=MoveBaseActionResult()
            sd.status.status=8
            sd.header.stamp=rospy.Time.now()
            sd_status_pub.publish(sd)
    if juli>4 and juli<=16:
            sd=MoveBaseActionResult()
            sd.status.status=10
            sd.header.stamp=rospy.Time.now()
            sd_status_pub.publish(sd)
    if juli>16 and juli<=36:
            sd=MoveBaseActionResult()
            sd.status.status=11
            sd.header.stamp=rospy.Time.now()
            sd_status_pub.publish(sd)
    if juli>36:
            sd=MoveBaseActionResult()
            sd.status.status=12
            sd.header.stamp=rospy.Time.now()
            sd_status_pub.publish(sd)
    ##print "距离第" ,index ,"个点" ,juli_sqrt ,"m,sd 为" ,sd.status.status
    ##print markerArray.markers[index-1].pose.position.x,markerArray.markers[index-1].pose.position.y
    ##print odom_sub.polygon.points[0].x,odom_sub.polygon.points[0].y
  #print index
    # if index>0 :
    #       print markerArray.markers[index-1].pose.position.x

arr1=[]
arr2=[]

def divide(S):    #记录'.'的位置
	for i in range(0,len(S)):
		if(S[i]=='.'):
			arr1.append(i)

def turn_int(S,Start,End):   #字符串转化为整数
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
    with open("/home/lz/catkin_ws/src/turn_on_wheeltec_robot/point_time.txt", "r") as f:   #读取小车停车时间文件
        data=f.read()
	cal(data)
    global try_again, index, add_more_point, try_again, index, try_again, index, add_more_point, try_again, index,quanshu
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
            if index == 2:     #特殊点特殊方向
                pose.pose.orientation.w=0
                pose.pose.orientation.z=1
                pose.pose.orientation.x=0
                pose.pose.orientation.y=0
            elif index == 4:
                pose.pose.orientation.z=1
                pose.pose.orientation.x=0
                pose.pose.orientation.y=0
                pose.pose.orientation.w=0
            elif index == 5 :
                pose.pose.orientation.w=-0.707
                pose.pose.orientation.z=0.707
            elif index != 0 : #if index != 0:
                x2=markerArray.markers[index].pose.position.x
                y2=markerArray.markers[index].pose.position.y
                x1=markerArray.markers[index-1].pose.position.x
                y1=markerArray.markers[index-1].pose.position.y
                v1=(1,0)
                v2=(x2-x1,y2-y1)
                yaw=math.acos((x2-x1)/math.sqrt((x2-x1)*(x2-x1)+(y2-y1)*(y2-y1)))   #计算欧拉角
                if y2-y1< 0:
                    yaw = -1*yaw
                q=tf.transformations.quaternion_from_euler(0,0,yaw)  #欧拉角转换为四元数
                pose.pose.orientation.z = q[2]
                pose.pose.orientation.w = q[3] #jieshu 
            #pose.pose.orientation.w = 1#1
            dexin=index
            ##print 'index < count:睡眠前index-1=',index-1,' count=',count,' 此次休眠时间为',arr2[dexin-1]
            rospy.sleep(arr2[dexin-1])   #小车停止时间
            goal_pub.publish(pose)     
            if index == 1 :     
                quanshu += 1
                print 'quanshu wei ',quanshu
                if quanshu == 3 :    #多点导航两圈后返回车库
                    os.system("./back.sh")
            index += 1
            ##print 'index < count:睡眠后index-1=',index-1,' count=',count
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
                pose.pose.orientation.w=0.707
                pose.pose.orientation.z=0.707
            if index!=0:#if index != 0:
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
            ##print 'index == count:睡眠前index+count-1=',index+count-1,' count=',count,' 此次休眠时间为',arr2[dexin+count-1]
            rospy.sleep(arr2[dexin+count-1])
            goal_pub.publish(pose)
            add_more_point = 1
            index+=1
            ##print 'index == count:睡眠后index=',index,' count=',count
        
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
    marker.scale.x = 0.5
    marker.scale.y = 0.5
    marker.scale.z = 0.5
    marker.color.a = 1
    marker.color.r = 0
    marker.color.g = 1
    marker.color.b = 1
    marker.pose.orientation.w = 1#1
    marker.pose.position.x = msg.point.x
    marker.pose.position.y = msg.point.y
    marker.pose.position.z = msg.point.z
    marker.text = str(count) #'asd'
    markerArray.markers.append(marker)
    id = 0
    for m in markerArray.markers:       #遍历记录点的个数
        m.id = id  #id
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
    global markerArray, count, index, add_more_point, try_again, mark_pub, goal_pub, goal_status_pub, markerArray, count, index, add_more_point, try_again, mark_pub, goal_pub, goal_status_pub,sd_status_pub,fw_pub,juli,juli1,juli2,fs_pub,quanshu
    markerArray = MarkerArray()
    count = 0
    index = 0
    quanshu = 0
    add_more_point = 0
    try_again = 1
    rospy.init_node('path_point_demo')
    # odom_sub = message_filters.Subscriber('odom', Odometry, queue_size=1)
    # point_sub  = message_filters.Subscriber('path_point', MarkerArray, queue_size=1)
    # sync = message_filters.ApproximateTimeSynchronizer([odom_sub, point_sub],10,0.1,allow_headerless=True)
    # sync.registerCallback(control_speed)
    sd_status_pub=rospy.Publisher('sd_status',MoveBaseActionResult,queue_size = 1)   #向/sd_status话题发布速度状态码
    fw_pub=rospy.Publisher('client_messages',String,queue_size = 1)
    fs_pub=rospy.Publisher('client_messages2',String,queue_size = 1)
    #rospy.Subscriber('/odom',Odometry,control_speed)
    rospy.Subscriber('/odom',Odometry,fabusudu)          #订阅/odom话题，执行fabusudu
    rospy.Subscriber('/move_base/global_costmap/footprint',PolygonStamped,fabuweizhi)   #订阅/move_base/global_costmap/footprint话题，执行fabuweizhi
    rospy.Subscriber('/move_base/global_costmap/footprint',PolygonStamped,control_speed)
    mark_pub = rospy.Publisher('/path_point', MarkerArray, queue_size = 100)
    click_sub = rospy.Subscriber('/clicked_point', PointStamped, press_callback)
    goal_pub = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size = 1)
    goal_status_sub = rospy.Subscriber('/move_base/result', MoveBaseActionResult, pose_callback)
    goal_status_pub = rospy.Publisher('/move_base/result', MoveBaseActionResult, queue_size = 1)
    rospy.spin()

if __name__ == '__main__':

    send_mark()


