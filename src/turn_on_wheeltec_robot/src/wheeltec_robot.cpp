#include "wheeltec_robot.h"
#include "Quaternion_Solution.h"
sensor_msgs::Imu Mpu6050;//实例化IMU对象
/**************************************
Date: May 31, 2020
Function: 主函数，ROS初始化，通过turn_on_robot类创建Robot_control对象并自动调用构造函数初始化
***************************************/
int main(int argc, char** argv)
{
  ros::init(argc, argv, "wheeltec_robot");//ROS初始化 并设置节点名称，可修改  
  ROS_INFO("wheeltec_robot node has turned on ");//显示状态
  turn_on_robot Robot_Control; //实例化一个对象
  Robot_Control.Control();  //循环执行数据采集和发布topic等操作
  return 0;  
} 
/**************************************
Date: June 29, 2020
Function: 数据传输转换函数
***************************************/
short turn_on_robot::IMU_Trans(uint8_t Data_High,uint8_t Data_Low)
{
  short transition_16;
      transition_16 = 0;
      transition_16 |=  Data_High<<8;   
      transition_16 |=  Data_Low;
  return transition_16;     
}
float turn_on_robot::Odom_Trans(uint8_t Data_High,uint8_t Data_Low)
{
  float data_return;
  short transition_16;
      transition_16 = 0;
      transition_16 |=  Data_High<<8;  //获取数据的高8位
      transition_16 |=  Data_Low;     //获取数据的低8位
      data_return   =  (transition_16 / 1000)+(transition_16 % 1000)*0.001; //(发送端将数据放大1000倍发送，这里需要将数据单位还原)
  return data_return;
}
/**************************************
Date: June 29, 2020
Function: 订阅回调函数Callback，根据订阅的指令向串口发指令控制下位机
***************************************/
/*void turn_on_robot::Cmd_Vel_Callback(const geometry_msgs::Twist &twist_aux)
{
  short  transition;  //中间变量
  Send_Data.tx[0]=FRAME_HEADER;//帧头 固定值
  Send_Data.tx[1] = 1 ; //产品型号
  Send_Data.tx[2] = 0;  //机器人使能控制标志位
  //机器人x轴的目标线速度
  transition=0;
  transition = twist_aux.linear.x*1000; //将浮点数放大一千倍，简化传输
  Send_Data.tx[4] = transition;     //取数据的低8位
  Send_Data.tx[3] = transition>>8;  //取数据的高8位
  //机器人y轴的目标线速度
  transition=0;
  transition = twist_aux.linear.y*1000;
  Send_Data.tx[6] = transition;
  Send_Data.tx[5] = transition>>8;
  //机器人z轴的目标角速度
  transition=0;
  transition = twist_aux.angular.z*1000;
  Send_Data.tx[8] = transition;
  Send_Data.tx[7] = transition>>8;

  Send_Data.tx[9]=Check_Sum(9,SEND_DATA_CHECK);//帧尾校验位，规则参见Check_Sum函数
  Send_Data.tx[10]=FRAME_TAIL;  //数据的最后一位是帧尾（固定值）
  try
  {
  // if(Receive_Data.Flag_Stop==0) 
  Stm32_Serial.write(Send_Data.tx,sizeof (Send_Data.tx)); //向串口发数据
  ROS_INFO_STREAM("New control command");//显示受到了新的控制指令  
  }
  catch (serial::IOException& e)
  {
    ROS_ERROR_STREAM("Unable to send data through serial port"); //如果try失败,打印错误信息
  }
}*/
void turn_on_robot::Status_Callback(const move_base_msgs::MoveBaseActionResult &sd_ctl) //***速度状态码，回调函数
{  
  short transition;
  transition=0;
  transition=sd_ctl.status.status*1000;  //放大1000倍，简化传输
  Send_Data.tx[6]=transition;     //低八位
  Send_Data.tx[5]=transition>>8;   //高八位
  try
  {
  // if(Receive_Data.Flag_Stop==0) 
  Stm32_Serial.write(Send_Data.tx,sizeof (Send_Data.tx)); //向串口发数据
  //ROS_INFO_STREAM("New control command");//显示收到了新的控制指令  
  }
  catch (serial::IOException& e)   
  {
    ROS_ERROR_STREAM("Unable to send data through serial port"); //如果try失败,打印错误信息
  }
}
void turn_on_robot::Cmd_Vel_Callback(const ackermann_msgs::AckermannDriveStamped &akm_ctl)
{
  short  transition;  //中间变量
  Send_Data.tx[0]=FRAME_HEADER;//帧头 固定值
  Send_Data.tx[1] = 1 ; //产品型号
  Send_Data.tx[2] = 0;  //机器人使能控制标志位

  //机器人x轴的目标线速度
  transition=0;
  transition = akm_ctl.drive.speed*1000; //将浮点数放大一千倍，简化传输
  Send_Data.tx[4] = transition;     //取数据的低8位
  Send_Data.tx[3] = transition>>8;  //取数据的高8位

  //机器人y轴的目标线速度
  //transition=0;
  //transition = twist_aux.linear.y*1000;
  //Send_Data.tx[6] = 1;
  //Send_Data.tx[5] = transition>>8; 
  
  //机器人z轴的目标角速度
  transition=0;
  transition = akm_ctl.drive.steering_angle*1000/2;
  Send_Data.tx[8] = transition;
  Send_Data.tx[7] = transition>>8;


  Send_Data.tx[9]=Check_Sum(9,SEND_DATA_CHECK);//帧尾校验位，规则参见Check_Sum函数
  Send_Data.tx[10]=FRAME_TAIL;  //数据的最后一位是帧尾（固定值）
  try
  {
  // if(Receive_Data.Flag_Stop==0) 
  Stm32_Serial.write(Send_Data.tx,sizeof (Send_Data.tx)); //向串口发数据
  //ROS_INFO_STREAM("New control command");//显示收到了新的控制指令  
  }
  catch (serial::IOException& e)   
  {
    ROS_ERROR_STREAM("Unable to send data through serial port"); //如果try失败,打印错误信息
  }
}
/**************************************
Date: May 31, 2020
Function: Amcl相关回调函数Callback
***************************************/
void turn_on_robot::Cmd_Amclvel_Callback(const geometry_msgs::PoseWithCovarianceStampedConstPtr &Pose)
{
  geometry_msgs::Pose Amclpose;//订阅机器人的姿态信息
  Amclpose.position.x = Pose->pose.pose.position.x;
  Amclpose.position.y = Pose->pose.pose.position.y;
  Amclpose.orientation = Pose->pose.pose.orientation;
  float temp = tf::getYaw(Amclpose.orientation);
}
/**************************************
Date: May 31, 2020
Function: 发布IMU数据
***************************************/
void turn_on_robot::Publish_ImuSensor()
{
  sensor_msgs::Imu Imu_Data_Pub;//话题的消息类型sensor_msgs::Imu
  Imu_Data_Pub.header.stamp = ros::Time::now(); //当前时间
  Imu_Data_Pub.header.frame_id = "gyro_link"; 
  Imu_Data_Pub.orientation.x = Mpu6050.orientation.x; //四元数
  Imu_Data_Pub.orientation.y = Mpu6050.orientation.y; 
  Imu_Data_Pub.orientation.z = Mpu6050.orientation.z;
  Imu_Data_Pub.orientation.w = Mpu6050.orientation.w;
  Imu_Data_Pub.orientation_covariance[0] = 1e6;
  Imu_Data_Pub.orientation_covariance[4] = 1e6;
  Imu_Data_Pub.orientation_covariance[8] = 1e-6;
  Imu_Data_Pub.angular_velocity.x = Mpu6050.angular_velocity.x; //三轴角速度
  Imu_Data_Pub.angular_velocity.y = Mpu6050.angular_velocity.y;
  Imu_Data_Pub.angular_velocity.z = Mpu6050.angular_velocity.z;
  Imu_Data_Pub.angular_velocity_covariance[0] = 1e6;
  Imu_Data_Pub.angular_velocity_covariance[4] = 1e6;
  Imu_Data_Pub.angular_velocity_covariance[8] = 1e-6;
  Imu_Data_Pub.linear_acceleration.x = Mpu6050.linear_acceleration.x; //三轴线性加速度
  Imu_Data_Pub.linear_acceleration.y = Mpu6050.linear_acceleration.y; 
  Imu_Data_Pub.linear_acceleration.z = Mpu6050.linear_acceleration.z;  
  imu_publisher.publish(Imu_Data_Pub); 
}
/**************************************
Date: May 31, 2020
Function: 发布里程计相关信息
***************************************/
void turn_on_robot::Publish_Odom()
{
    geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(Robot_Pos.Z);
    nav_msgs::Odometry odom;//里程计话题消息数据类型
    odom.header.stamp = ros::Time::now();//当前时间
    odom.header.frame_id = "odom";
    odom.pose.pose.position.x = Robot_Pos.X;//位置
    odom.pose.pose.position.y = Robot_Pos.Y;
    odom.pose.pose.position.z = Robot_Pos.Z;
    odom.pose.pose.orientation = odom_quat;
    //设置速度
    odom.child_frame_id = robot_frame_id;
    odom.twist.twist.linear.x =  Robot_Vel.X;//X方向前进速度
    odom.twist.twist.linear.y =  Robot_Vel.Y;//y方向前进速度
    odom.twist.twist.angular.z = Robot_Vel.Z;  //角速度 
    //这个矩阵有两种，分机器人静止和动起来的时候用 这是扩展卡尔曼滤波的,官网提供的2个矩阵
    if(Robot_Vel.X== 0&&Robot_Vel.Y== 0&&Robot_Vel.Z== 0)//如果velocity是零，说明编码器的误差会比较小，认为编码器数据更可靠
    memcpy(&odom.pose.covariance, odom_pose_covariance2, sizeof(odom_pose_covariance2)),
    memcpy(&odom.twist.covariance, odom_twist_covariance2, sizeof(odom_twist_covariance2));
    else//如果小车velocity非零，考虑到运动中编码器可能带来的滑动误差，认为imu的数据更可靠
    memcpy(&odom.pose.covariance, odom_pose_covariance, sizeof(odom_pose_covariance)),
    memcpy(&odom.twist.covariance, odom_twist_covariance, sizeof(odom_twist_covariance));       
    odom_publisher.publish(odom);//发布这个话题 消息类型是nav_msgs::Odometry
}
/**************************************
Date: May 31, 2020
Function: 发布电压相关信息
***************************************/
void turn_on_robot::Publish_Voltage()
{
    std_msgs::Float32 voltage_msgs;//定义电源电压发布topic的数据类型std_msgs::Float32
    static float Count_Voltage_Pub=0;
    if(Count_Voltage_Pub++>10)
      {
        Count_Voltage_Pub=0;  
        voltage_msgs.data = Power_voltage;//电源供电的电压获取
        voltage_publisher.publish(voltage_msgs);//发布电源电压话题单位V
      }
}
/**************************************
Date: June 29, 2020
Function: 串口通讯校验函数，数据包除最后一个字节，其他的全部数据按位异或的结果作为帧尾
***************************************/
unsigned char turn_on_robot::Check_Sum(unsigned char Count_Number,unsigned char mode)
{
  unsigned char check_sum=0,k;
  
  if(mode==0) //接收数据
  {
   for(k=0;k<Count_Number;k++)//Count_Number是接收数组位数减1
    {
     check_sum=check_sum^Receive_Data.rx[k];//按位异或
     }
  }
  if(mode==1) //发送数据
  {
   for(k=0;k<Count_Number;k++)//Count_Number是发送数组位数减1
    {
     check_sum=check_sum^Send_Data.tx[k];//按位异或
     }
  }
  return check_sum;//返回结果
}
bool turn_on_robot::Get_Sensor_Data()
{ 
  short transition_16=0,j=0,Header_Pos=0,Tail_Pos=0;  //中间变量
  uint8_t Receive_Data_Pr[RECEIVE_DATA_SIZE]={0};
  Stm32_Serial.read(Receive_Data_Pr,sizeof (Receive_Data_Pr));//读串口数据
/*
    ROS_INFO("%x-%x-%x-%x-%x-%x-%x-%x-%x-%x-%x-%x-%x-%x-%x-%x-%x-%x-%x-%x-%x-%x-%x-%x",
    Receive_Data_Pr[0],Receive_Data_Pr[1],Receive_Data_Pr[2],Receive_Data_Pr[3],Receive_Data_Pr[4],Receive_Data_Pr[5],Receive_Data_Pr[6],Receive_Data_Pr[7],
   Receive_Data_Pr[8],Receive_Data_Pr[9],Receive_Data_Pr[10],Receive_Data_Pr[11],Receive_Data_Pr[12],Receive_Data_Pr[13],Receive_Data_Pr[14],Receive_Data_Pr[15],
    Receive_Data_Pr[16],Receive_Data_Pr[17],Receive_Data_Pr[18],Receive_Data_Pr[19],Receive_Data_Pr[20],Receive_Data_Pr[21],Receive_Data_Pr[22],Receive_Data_Pr[23]);
*/  
  for(j=0;j<24;j++)
    {
    if(Receive_Data_Pr[j]==FRAME_HEADER)
    Header_Pos=j;
    else if(Receive_Data_Pr[j]==FRAME_TAIL)
    Tail_Pos=j;    
    }
   // ROS_INFO("%x-%x",Header_Pos,Tail_Pos);
    if(Tail_Pos==(Header_Pos+23))
    {
         // ROS_INFO("1----");
      memcpy(Receive_Data.rx, Receive_Data_Pr, sizeof(Receive_Data_Pr));
    }
    else if(Header_Pos==(1+Tail_Pos))
    {
     // ROS_INFO("2----");
        for(j=0;j<24;j++)
        Receive_Data.rx[j]=Receive_Data_Pr[(j+Header_Pos)%24];
    }
    else 
    {
    // ROS_INFO("3----");
    return false;
    }    
    /*    ROS_INFO("%x-%x-%x-%x-%x-%x-%x-%x-%x-%x-%x-%x-%x-%x-%x-%x-%x-%x-%x-%x-%x-%x-%x-%x",
   Receive_Data.rx[0],Receive_Data.rx[1],Receive_Data.rx[2],Receive_Data.rx[3],Receive_Data.rx[4],Receive_Data.rx[5],Receive_Data.rx[6],Receive_Data.rx[7],
   Receive_Data.rx[8],Receive_Data.rx[9],Receive_Data.rx[10],Receive_Data.rx[11],Receive_Data.rx[12],Receive_Data.rx[13],Receive_Data.rx[14],Receive_Data.rx[15],
    Receive_Data.rx[16],Receive_Data.rx[17],Receive_Data.rx[18],Receive_Data.rx[19],Receive_Data.rx[20],Receive_Data.rx[21],Receive_Data.rx[22],Receive_Data.rx[23]); 
  */
  Receive_Data.Frame_Header= Receive_Data.rx[0]; //数据的第一位是帧头（固定值）
  Receive_Data.Frame_Tail= Receive_Data.rx[23];  //数据的最后一位是帧尾（数据校验位）

 if (Receive_Data.Frame_Header == FRAME_HEADER )//判断帧头
  {
    if (Receive_Data.Frame_Tail == FRAME_TAIL) //判断帧尾
    { 
      if (Receive_Data.rx[22] == Check_Sum(22,READ_DATA_CHECK))//校验位检测
      {
        Receive_Data.Flag_Stop=Receive_Data.rx[1];//停止位
        Robot_Vel.X = Odom_Trans(Receive_Data.rx[2],Receive_Data.rx[3]); //获取底盘X方向速度
        Robot_Vel.Y = Odom_Trans(Receive_Data.rx[4],Receive_Data.rx[5]); //获取底盘Y方向速度//Y速度仅在全向移动机器人底盘有效
        Robot_Vel.Z = Odom_Trans(Receive_Data.rx[6],Receive_Data.rx[7]); //获取底盘Z方向速度   
        
        Mpu6050_Data.accele_x_data = IMU_Trans(Receive_Data.rx[8],Receive_Data.rx[9]);//获取IMU的X轴加速度  
        Mpu6050_Data.accele_y_data = IMU_Trans(Receive_Data.rx[10],Receive_Data.rx[11]);//获取IMU的X轴加速度
        Mpu6050_Data.accele_z_data = IMU_Trans(Receive_Data.rx[12],Receive_Data.rx[13]);//获取IMU的X轴加速度
        Mpu6050_Data.gyros_x_data = IMU_Trans(Receive_Data.rx[14],Receive_Data.rx[15]);//获取IMU的X轴角速度  
        Mpu6050_Data.gyros_y_data = IMU_Trans(Receive_Data.rx[16],Receive_Data.rx[17]);//获取IMU的X轴角速度  
        Mpu6050_Data.gyros_z_data = IMU_Trans(Receive_Data.rx[18],Receive_Data.rx[19]);//获取IMU的X轴角速度  
        //线性加速度单位转化，和STM32 MPU6050初始化的时候的量程有关
        Mpu6050.linear_acceleration.x = Mpu6050_Data.accele_x_data / ACCEl_RATIO;
        Mpu6050.linear_acceleration.y = Mpu6050_Data.accele_y_data / ACCEl_RATIO;
        Mpu6050.linear_acceleration.z = Mpu6050_Data.accele_z_data / ACCEl_RATIO;
        //陀螺仪单位转化，和STM32底层有关，这里MPU6050的陀螺仪的量程是正负500
        //因为机器人一般Z轴速度不快，降低量程可以提高精度
        Mpu6050.angular_velocity.x =  Mpu6050_Data.gyros_x_data * GYROSCOPE_RATIO;
        Mpu6050.angular_velocity.y =  Mpu6050_Data.gyros_y_data * GYROSCOPE_RATIO;
        Mpu6050.angular_velocity.z =  Mpu6050_Data.gyros_z_data * GYROSCOPE_RATIO;
        //Robot_Vel.Z = Mpu6050.angular_velocity.z;
        //获取电池电压
        transition_16 = 0;
        transition_16 |=  Receive_Data.rx[20]<<8;
        transition_16 |=  Receive_Data.rx[21];  
        Power_voltage = transition_16/1000+(transition_16 % 1000)*0.001;//(发送端将数据放大1000倍发送，这里需要将数据单位还原)
        return true;
     }
    }
  } 
 return false;
}
/**************************************
Date: May 31, 2020
Function: 这是相关控制代码，代码循环执行
***************************************/
void turn_on_robot::Control()
{
  _Last_Time = ros::Time::now();
  while(ros::ok())
  {
    _Now = ros::Time::now();
    Sampling_Time = (_Now - _Last_Time).toSec();
   //Sampling_time是采样时间，虽然下位机发送的数据频率是固定的，这里计算里程增量以ROS系统的时间更加可靠精确。
    if (true == Get_Sensor_Data())  //从串口读取下位机法过来的全部数据
    {
      Robot_Pos.X+=(Robot_Vel.X * cos(Robot_Pos.Z) - Robot_Vel.Y * sin(Robot_Pos.Z)) * Sampling_Time;//计算x方向的位移
      Robot_Pos.Y+=(Robot_Vel.X * sin(Robot_Pos.Z) + Robot_Vel.Y * cos(Robot_Pos.Z)) * Sampling_Time;//计算y方向的位移， 
      Robot_Pos.Z+=Robot_Vel.Z * Sampling_Time; //角位移  
      Quaternion_Solution(Mpu6050.angular_velocity.x, Mpu6050.angular_velocity.y, Mpu6050.angular_velocity.z,\
                Mpu6050.linear_acceleration.x, Mpu6050.linear_acceleration.y, Mpu6050.linear_acceleration.z);//四元数解算
      Publish_Odom();        //发布里程计话题
      Publish_ImuSensor();  //发布话题    
      Publish_Voltage(); //发布电源电压
    }
    _Last_Time = _Now;//记录时间
    ros::spinOnce();//循环等待回调函数
    }
}
/**************************************
Date: May 31, 2020
Function: 构造函数, 只执行一次，用于初始化
***************************************/
turn_on_robot::turn_on_robot():Sampling_Time(0),Power_voltage(0)
{
  memset(&Robot_Pos, 0, sizeof(Robot_Pos));
  memset(&Robot_Vel, 0, sizeof(Robot_Vel));
  memset(&Receive_Data, 0, sizeof(Receive_Data)); //构造函数初始化
  memset(&Send_Data, 0, sizeof(Send_Data));
  memset(&Mpu6050_Data, 0, sizeof(Mpu6050_Data));
  ros::NodeHandle private_nh("~");
  //把以上的类成员参数注册到参数服务器，这样在launch文件里面即可修改
  //3个入口参数分别对应：参数服务器上的名称  参数变量名  初始值
  private_nh.param<std::string>("usart_port_name", usart_port_name, "/dev/wheeltec_controller"); //固定串口
  private_nh.param<int>("serial_baud_rate", serial_baud_rate, 115200); //和下位机底层波特率115200 不建议更高的波特率了
  private_nh.param<std::string>("smoother_cmd_vel", smoother_cmd_vel, "/smoother_cmd_vel");//平滑控制指令 
  private_nh.param<std::string>("robot_frame_id", robot_frame_id, "base_link");//ID
  private_nh.param<int>("product_number", product_number,0);//产品名称
  //发布3个话题，订阅2个话题
  voltage_publisher = n.advertise<std_msgs::Float32>("/PowerVoltage", 10);//电池电压数据发布
  odom_publisher = n.advertise<nav_msgs::Odometry>("odom", 50);//里程计数据发布
  imu_publisher  = n.advertise<sensor_msgs::Imu>("/mobile_base/sensors/imu_data", 20);//IMU数据发布
 // Cmd_Vel_Sub = n.subscribe("/cmd_vel", 100, &turn_on_robot::Cmd_Vel_Callback, this);//因为官方的平滑包只支持X和W，没有Y，所以这里不使用平滑包
  Cmd_Vel_Sub = n.subscribe("/ackermann_cmd", 100, &turn_on_robot::Cmd_Vel_Callback, this);//接收阿克曼类型的数据
  sd_status_sub=n.subscribe("/sd_status",100,&turn_on_robot::Status_Callback,this); //***订阅sd_status话题，发送给下位机
//Cmd_Vel_Sub = n.subscribe(smoother_cmd_vel, 100, &turn_on_robot::Cmd_Vel_Callback, this);//订阅smoother_cmd_vel话题并控制机器人//差速 
  Amcl_Sub = n.subscribe("/amcl_pose", 100, &turn_on_robot::Cmd_Amclvel_Callback, this);//自适应蒙特卡洛定位需要的数据
  ROS_INFO_STREAM("Data ready");//ready显示状态
  //初始化串口
  try{
         Stm32_Serial.setPort(usart_port_name);//选择哪个口，如果选择的口没有接串口外设初始化会失败
         Stm32_Serial.setBaudrate(serial_baud_rate);//设置波特率
         serial::Timeout _time = serial::Timeout::simpleTimeout(2000);//超时等待
         Stm32_Serial.setTimeout(_time);
         Stm32_Serial.open();//串口开启
    }
  catch (serial::IOException& e){
     ROS_ERROR_STREAM("wheeltec_robot can not open serial port,Please check the serial port cable! ");//如果try失败，打印错误信息
  }
  if(Stm32_Serial.isOpen()){
    ROS_INFO_STREAM("wheeltec_robot serial port opened");//开启成功
  }else{
  }
}
/**************************************
Date: May 31, 2020
Function: 析构函数，只执行一次，当对象结束其生命周期时系统会调用这个函数
***************************************/
turn_on_robot::~turn_on_robot()
{
  Send_Data.tx[0]=FRAME_HEADER;//帧头 固定值
  Send_Data.tx[1] = 0; //产品型号
  Send_Data.tx[2] = 0;  //机器人使能控制标志位
  //机器人x轴的目标线速度
  Send_Data.tx[4] = 0;     //取数据的低8位
  Send_Data.tx[3] = 0;  //取数据的高8位
  //机器人y轴的目标线速度
  Send_Data.tx[6] = 0;
  Send_Data.tx[5] = 0;  
  //机器人z轴的目标角速度
  Send_Data.tx[8] = 0;  
  Send_Data.tx[7] = 0;    

  Send_Data.tx[9]=Check_Sum(9,SEND_DATA_CHECK);//帧尾校验位，规则参见Check_Sum函数
  Send_Data.tx[10]=FRAME_TAIL;  //数据的最后一位是帧尾（固定值）
  try
  {
  // if(Receive_Data.Flag_Stop==0) 
  Stm32_Serial.write(Send_Data.tx,sizeof (Send_Data.tx)); //向串口发数据
  //ROS_INFO_STREAM("New control command");//显示受到了新的控制指令  
  }
  catch (serial::IOException& e)   
  {
    ROS_ERROR_STREAM("Unable to send data through serial port"); //如果try失败,打印错误信息
  }
  Stm32_Serial.close();//关闭串口  
  ROS_INFO_STREAM("Shutting down");//close 
}
