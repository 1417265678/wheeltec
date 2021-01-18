
#ifndef __WHEELTEC_ROBOT_H_
#define __WHEELTEC_ROBOT_H_
// 头文件
#include "ros/ros.h"
#include <iostream>
#include <string.h>
#include <string> 
#include <iostream>
#include <math.h>
#include <stdlib.h>    
#include <unistd.h>      
#include <sys/types.h>
#include <sys/stat.h>
#include <serial/serial.h>
#include <fcntl.h>          
#include <stdbool.h>
#include <tf/transform_broadcaster.h>
#include <std_msgs/String.h>
#include <std_msgs/Float32.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <ackermann_msgs/AckermannDriveStamped.h>
#include <sensor_msgs/Imu.h>
#include <move_base_msgs/MoveBaseActionResult.h> //***
using namespace std;
#define SEND_DATA_CHECK   1     //标志位，发送端做校验位
#define READ_DATA_CHECK   0     //标志位，接收端做校验位
#define FRAME_HEADER      0X7B  //帧头，和下位机一致
#define FRAME_TAIL  0X7D //帧尾
#define RECEIVE_DATA_SIZE		24//下位机发过来的数据的长度
#define SEND_DATA_SIZE			11//ROS发给下位机的数据的长度 考虑到时效应短尽短
#define PI 				3.1415926f//圆周率
//这个和陀螺仪设置的量程有关的 转化为度每秒是/65.5 转为弧度每秒/57.3 其子65.5看MPU6050手册，STM32底层FS_SEL=1
#define GYROSCOPE_RATIO	0.00026644f	// 1/65.5/57.30=0.00026644 陀螺仪原始数据换成弧度单位
//这个和陀螺仪设置的量程有关的 转化为度每秒是/65.5 转为弧度每秒/57.3 其子65.5看MPU6050手册，STM32底层FS_SEL=1
#define ACCEl_RATIO 	16384.0f  	// 量程±2g，重力加速度定义为1g等于9.8米每平方秒。
extern sensor_msgs::Imu Mpu6050;
//协方差
const double odom_pose_covariance[36] = {1e-3, 0, 0, 0, 0, 0, 
										0, 1e-3, 0, 0, 0, 0,
										0, 0, 1e6, 0, 0, 0,
										0, 0, 0, 1e6, 0, 0,
										0, 0, 0, 0, 1e6, 0,
										0, 0, 0, 0, 0, 1e3};
const double odom_pose_covariance2[36] = {1e-9, 0, 0, 0, 0, 0, 
										0, 1e-3, 1e-9, 0, 0, 0,
										0, 0, 1e6, 0, 0, 0,
										0, 0, 0, 1e6, 0, 0,
										0, 0, 0, 0, 1e6, 0,
										0, 0, 0, 0, 0, 1e-9};
 
const double odom_twist_covariance[36] = {1e-3, 0, 0, 0, 0, 0, 
										0, 1e-3, 0, 0, 0, 0,
										0, 0, 1e6, 0, 0, 0,
										0, 0, 0, 1e6, 0, 0,
										0, 0, 0, 0, 1e6, 0,
										0, 0, 0, 0, 0, 1e3};
const double odom_twist_covariance2[36] = {1e-9, 0, 0, 0, 0, 0, 
										0, 1e-3, 1e-9, 0, 0, 0,
										0, 0, 1e6, 0, 0, 0,
										0, 0, 0, 1e6, 0, 0,
										0, 0, 0, 0, 1e6, 0,
										0, 0, 0, 0, 0, 1e-9};
// 速度/位置结构体
typedef struct __Vel_Pos_Data_
{
	float X;
	float Y;
	float Z;
}Vel_Pos_Data;


typedef struct __MPU6050_DATA_
{
	short accele_x_data; 
	short accele_y_data; 	
	short accele_z_data; 
    short gyros_x_data; 
	short gyros_y_data; 	
	short gyros_z_data; 

}MPU6050_DATA;

//DATE：2020-5-31

typedef struct _SEND_DATA_  
{
	    uint8_t tx[SEND_DATA_SIZE];
		float X_speed;	       
		float Y_speed;           
		float Z_speed;         
		unsigned char Frame_Tail;    //1个字节  帧尾 校验位 

}SEND_DATA;
//DATE：2020-5-31

typedef struct _RECEIVE_DATA_     
{
	    uint8_t rx[RECEIVE_DATA_SIZE];
	    uint8_t Flag_Stop;
		unsigned char Frame_Header; //1个字节 帧头
		float X_speed;  
		float Y_speed;  
		float Z_speed;  
		float Power_Voltage;	
		unsigned char Frame_Tail;//1个字节  帧尾 校验位
}RECEIVE_DATA;

//DATE：2020-5-31
//类，巧妙使用构造函数初始化数据和发布话题等
class turn_on_robot
{
	public:
		turn_on_robot(); //构造函数
		~turn_on_robot(); //析构函数
		void Control();//循环控制代码
		serial::Serial Stm32_Serial; //声明串口对象 
	private:
		/* /cmd_val topic call function 回调函数声明*/
		//void Cmd_Vel_Callback(const geometry_msgs::Twist &twist_aux);
		void Cmd_Vel_Callback(const ackermann_msgs::AckermannDriveStamped &akm_ctl);
		void Status_Callback(const move_base_msgs::MoveBaseActionResult &sd_ctl);  //***
		void Cmd_Amclvel_Callback(const geometry_msgs::PoseWithCovarianceStampedConstPtr &Pose);
		/* Read/Write data from ttyUSB 串口和控制函数声明 */
		bool Get_Sensor_Data();
		short IMU_Trans(uint8_t Data_High,uint8_t Data_Low);
		float Odom_Trans(uint8_t Data_High,uint8_t Data_Low);
		/* This node Publisher topic and tf */
		void Publish_Odom();//发表里程计
		void Publish_ImuSensor();//发布IMU传感器
		void Publish_Voltage();//发布电源电压
		unsigned char Check_Sum(unsigned char Count_Number,unsigned char mode);//校验函数
       	Vel_Pos_Data Robot_Pos;//机器人的位置
        Vel_Pos_Data Robot_Vel;//机器人的速度
		int serial_baud_rate;//波特率
		int product_number;//产品名称
		string usart_port_name, robot_frame_id, smoother_cmd_vel;
		ros::NodeHandle n;//创建句柄
		ros::Time _Now, _Last_Time;//时间相关
		float Sampling_Time; //采样时间
		ros::Subscriber Cmd_Vel_Sub, Amcl_Sub,sd_status_sub;//初始化2个订阅者 *****
		ros::Publisher odom_publisher, imu_publisher, voltage_publisher;//初始化3个发布者
		tf::TransformBroadcaster odom_broadcaster;
		float Power_voltage;//电源电压
		RECEIVE_DATA Receive_Data;  //接收结构体   Receive_Data    
        SEND_DATA Send_Data;  //发送结构体  Send_Data
        MPU6050_DATA Mpu6050_Data;
};
#endif
