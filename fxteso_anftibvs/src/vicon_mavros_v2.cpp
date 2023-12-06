/***************************************************************************************************************************
 * px4_pos_estimator.cpp
 *
 * Author: Qyp
 *
 * Update Time: 2019.3.10
 *
 * 说明: mavros位置估计程序
 *      1. 订阅激光SLAM (cartorgrapher_ros节点) 发布的位置信息,从laser坐标系转换至NED坐标系
 *      2. 订阅Mocap设备 (vrpn-client-ros节点) 发布的位置信息，从mocap坐标系转换至NED坐标系
 *      3. 订阅飞控发布的位置、速度及欧拉角信息，作对比用
 *      4. 存储飞行数据，实验分析及作图使用
 *      5. 选择激光SLAM或者Mocap设备作为位置来源，发布位置及偏航角(xyz+yaw)给飞控
 *
***************************************************************************************************************************/
/***************************************************************************************************************************
* px4_pos_controller.cpp
*
* Author: Qyp
*
* Update Time: 2019.3.16
*能够在机载电脑中接收到所需要的定位信息
* Introduction:  PX4 Position Estimator using external positioning equipment
*         1. Subscribe position and yaw information from Lidar SLAM node(cartorgrapher_ros节点), transfrom from laser frame to ENU frame
*         2. Subscribe position and yaw information from Vicon node(vrpn-client-ros节点), transfrom from vicon frame to ENU frame
*         3. Send the position and yaw information to FCU using Mavros package (/mavros/mocap/pose or /mavros/vision_estimate/pose)
*         4. Subscribe position and yaw information from FCU, used for compare
***************************************************************************************************************************/


//头文件
#include <ros/ros.h>

#include <iostream>
#include <eigen3/Eigen/Dense>


#include <math_utils.h>
#include <math.h>
//#include <Frame_tf_utils.h>
//msg 头文件
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/Bool.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <geometry_msgs/Point.h>
#include <std_msgs/UInt16.h>
#include <std_msgs/Float64.h>
#include <tf2_msgs/TFMessage.h>
#include <geometry_msgs/TransformStamped.h>
#include <sensor_msgs/Range.h>
//---------------------------------------相关参数-----------------------------------------------
int flag_use_laser_or_vicon;                               //0:使用mocap数据作为定位数据 1:使用laser数据作为定位数据
//---------------------------------------vicon定位相关------------------------------------------
Eigen::Vector3d pos_drone_mocap;                          //无人机当前位置 (vicon)
Eigen::Quaterniond q_mocap;
Eigen::Vector3d Euler_mocap;                              //无人机当前姿态 (vicon)
//---------------------------------------无人机位置及速度--------------------------------------------
Eigen::Vector3d pos_drone_fcu;                           //无人机当前位置 (来自fcu)
Eigen::Vector3d vel_drone_fcu;                           //无人机上一时刻位置 (来自fcu)

Eigen::Quaterniond q_fcu;
Eigen::Vector3d Euler_fcu;                                          //无人机当前欧拉角(来自fcu)
//---------------------------------------发布相关变量--------------------------------------------能够在机载电脑中接收到所需要的定位信息
void optitrack_cb(const geometry_msgs::TransformStamped::ConstPtr& msg)
{
//    cout<<"here!!--------------------------------"<<endl;
    //位置 -- optitrack系 到 ENU系
    int optitrack_frame = 0; //Frame convention 0: Z-up -- 1: Y-up
    // Read the Drone Position from the Vrpn Package [Frame: Vicon]  (Vicon to ENU frame)
    Eigen::Vector3d pos_drone_mocap_enu(msg->transform.translation.x,msg->transform.translation.y,msg->transform.translation.z);

    pos_drone_mocap = pos_drone_mocap_enu;

    if(optitrack_frame == 0){
        // Read the Quaternion from the Vrpn Package [Frame: Vicon[ENU]]
        Eigen::Quaterniond q_mocap_enu(msg->transform.rotation.w, msg->transform.rotation.x, msg->transform.rotation.y, msg->transform.rotation.z);
        q_mocap = q_mocap_enu;
    }
    else
    {
        // Read the Quaternion from the Vrpn Package [Frame: Vicon[ENU]]
        Eigen::Quaterniond q_mocap_enu(msg->transform.rotation.w, msg->transform.rotation.x, msg->transform.rotation.z, msg->transform.rotation.y); //Y-up convention, switch the q2 & q3
        q_mocap = q_mocap_enu;
    }

    // Transform the Quaternion to Euler Angles
    Euler_mocap = quaternion_to_euler(q_mocap);

}

void pos_cb(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    // Read the Drone Position from the Mavros Package [Frame: ENU]
    Eigen::Vector3d pos_drone_fcu_enu(msg->pose.position.x,msg->pose.position.y,msg->pose.position.z);

    pos_drone_fcu = pos_drone_fcu_enu;
}

void vel_cb(const geometry_msgs::TwistStamped::ConstPtr &msg)
{
    // Read the Drone Velocity from the Mavros Package [Frame: ENU]
    Eigen::Vector3d vel_drone_fcu_enu(msg->twist.linear.x,msg->twist.linear.y,msg->twist.linear.z);

    vel_drone_fcu = vel_drone_fcu_enu;
}

// void euler_cb(const sensor_msgs::Imu::ConstPtr& msg)
// {
//     // Read the Quaternion from the Mavros Package [Frame: ENU]
//     Eigen::Quaterniond q_fcu_enu(msg->orientation.w, msg->orientation.x, msg->orientation.y, msg->orientation.z);

//     q_fcu = q_fcu_enu;

//     //Transform the Quaternion to Euler Angles
//     Euler_fcu = quaternion_to_euler(q_fcu);

//     // Transform the Quaternion from ENU to NED
//     Eigen::Quaterniond q_ned = transform_orientation_enu_to_ned( transform_orientation_baselink_to_aircraft(q_fcu_enu) );
// }


//获取当前时间 单位：秒
float get_dt(ros::Time last)
{
    ros::Time time_now = ros::Time::now();
    float currTimeSec = time_now.sec-last.sec;
    float currTimenSec = time_now.nsec / 1e9 - last.nsec / 1e9;
    return (currTimeSec + currTimenSec);
}



void printf_info()
{
  //  cout <<">>>>>>>>>>>>>>>>>>>>>>>>PX4_POS_ESTIMATOR<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<" <<endl;

    //固定的浮点显示
    cout.setf(ios::fixed);
    //setprecision(n) 设显示小数精度为n位
    cout<<setprecision(2);
    //左对齐
    cout.setf(ios::left);
    // 强制显示小数点
    cout.setf(ios::showpoint);
    // 强制显示符号
    cout.setf(ios::showpos);

    //using vicon system
    if(flag_use_laser_or_vicon == 0)
    {
       // cout <<">>>>>>>>>>>>>>>>>>>>>>>>VICON Info [ENU Frame]<<<<<<<<<<<<<<<<<<<<<<<<<" <<endl;
        //cout << "Pos_vicon [X Y Z] : " << pos_drone_mocap[0] << " [ m ] "<< pos_drone_mocap[1] <<" [ m ] "<< pos_drone_mocap[2] <<" [ m ] "<<endl;
        //cout << "Euler_vicon [Roll] : " << Euler_mocap[0] << " [rad]  " << endl;
        //cout << "Euler_vicon [Pitch] : " << Euler_mocap[1] << " [rad]  " << endl;
        //cout << "Euler_vicon [Yaw] : " << Euler_mocap[2] << " [rad]  " << endl;
        //cout << "Euler_vicon [Yaw] : " << Euler_mocap[2] * 180/M_PI<<" [deg]  "<<endl;
       
	/*
	 //
        cout << ">>>>>>>>>>>>>>>>>>>>>>>>VICON Info [NED Frame]<<<<<<<<<<<<<<<<<<<<<<<<<" << endl;
        cout << "Pos_vicon [X Y Z] : " << pos_drone_mocap[0] << " [ m ] " << -pos_drone_mocap[1] << " [ m ] " << -pos_drone_mocap[2] << " [ m ] " << endl;
        cout << "Euler_vicon [Roll] : " << Euler_mocap[0] << " [rad]  " << endl;
        cout << "Euler_vicon [Pitch] : " << -Euler_mocap[1] << " [rad]  " << endl;
        cout << "Euler_vicon [Yaw] : " << -Euler_mocap[2] << " [rad]  " << endl;
        //cout << "Euler_vicon [Yaw] : " << -Euler_mocap[2] * 180 / M_PI << " [deg]  " << endl;
	*/
    }
/*        cout <<">>>>>>>>>>>>>>>>>>>>>>>>FCU Info [ENU Frame]<<<<<<<<<<<<<<<<<<<<<<<<<<<" <<endl;
        cout << "Pos_fcu [X Y Z] : " << pos_drone_fcu[0] << " [ m ] "<< pos_drone_fcu[1] <<" [ m ] "<< pos_drone_fcu[2] <<" [ m ] "<<endl;
        cout << "Vel_fcu [X Y Z] : " << vel_drone_fcu[0] << " [m/s] "<< vel_drone_fcu[1] <<" [m/s] "<< vel_drone_fcu[2] <<" [m/s] "<<endl;
        cout << "Euler_fcu [Yaw] : " << Euler_fcu[2] <<" [rad] "<<endl;*/
}



//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>主 函 数<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
int main(int argc, char **argv)
{
    ros::init(argc, argv, "px4_pos_estimator");
    ros::NodeHandle nh("~");

    //读取参数表中的参数
    // 使用激光SLAM数据orVicon数据 0 for vicon， 1 for 激光SLAM
    nh.param<int>("flag_use_laser_or_vicon", flag_use_laser_or_vicon, 0);

    // 【订阅】optitrack估计位置
    ros::Subscriber optitrack_sub = nh.subscribe<geometry_msgs::TransformStamped>("/vicon/QUAV/QUAV", 10, optitrack_cb);

    // 【订阅】无人机当前位置 坐标系:ENU系 [室外：GPS，室内：自主定位或mocap等] 这里订阅仅作比较用
    ros::Subscriber position_sub = nh.subscribe<geometry_msgs::PoseStamped>("/mavros/local_position/pose", 10, pos_cb);

    // 【订阅】无人机当前速度 坐标系:ENU系 [室外：GPS，室内：自主定位或mocap等] 这里订阅仅作比较用
    ros::Subscriber velocity_sub = nh.subscribe<geometry_msgs::TwistStamped>("/mavros/local_position/velocity_local", 10, vel_cb);

    // 【订阅】无人机当前欧拉角 坐标系:ENU系 这里订阅仅作比较用
  //  ros::Subscriber euler_sub = nh.subscribe<sensor_msgs::Imu>("/mavros/imu/data", 10, euler_cb);

    // 【发布】无人机位置和偏航角 坐标系 ENU系
    //  本话题要发送飞控(通过mavros_extras/src/plugins/vision_pose_estimate.cpp发送), 对应Mavlink消息为VISION_POSITION_ESTIMATE(#??), 对应的飞控中的uORB消息为vehicle_vision_position.msg 及 vehicle_vision_attitude.msg
    ros::Publisher vision_pub = nh.advertise<geometry_msgs::PoseStamped>("/mavros/vision_pose/pose", 100);
	//ros::Publisher euler_pub = nh.advertise<geometry_msgs::Vector3>("/quad_attitude", 100);
    // 频率
    ros::Rate rate(50.0);

//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>Main Loop<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
    while(ros::ok())
    {
        //回调一次 更新传感器状态
        ros::spinOnce();

        //vicon
        geometry_msgs::PoseStamped vision;
        geometry_msgs::Vector3 attitude_euler;
        
        if(flag_use_laser_or_vicon == 0)
        {
          vision.pose.position.x = pos_drone_mocap[0] ;
          vision.pose.position.y = pos_drone_mocap[1] ;
          vision.pose.position.z = pos_drone_mocap[2] ;
           // vision.pose.position.x = 10 ;
            //vision.pose.position.y = 10 ;
           // vision.pose.position.z = 10 ;

            vision.pose.orientation.x = q_mocap.x();
            vision.pose.orientation.y = q_mocap.y();
            vision.pose.orientation.z = q_mocap.z();
            vision.pose.orientation.w = q_mocap.w();
            
            /*attitude_euler.x = -Euler_mocap[0];
            attitude_euler.y = Euler_mocap[1];
            attitude_euler.z = -Euler_mocap[2];
*/
        }
        
        vision.header.stamp = ros::Time::now();
        vision_pub.publish(vision);
		//euler_pub.publish(attitude_euler);
        //打印
        printf_info();
        rate.sleep();
    }

    return 0;

}
