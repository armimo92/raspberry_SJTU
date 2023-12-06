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
int flag_use_laser_or_vicon = 0;                              //0:使用mocap数据作为定位数据 1:使用laser数据作为定位数据
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
   //
// cout<<"here!!--------------------------------"<<endl;
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
/*
    cout <<">>>>>>>>>>>>>>>>>>>>>>>>PX4_POS_ESTIMATOR<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<" <<endl;

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
        cout <<">>>>>>>>>>>>>>>>>>>>>>>>VICON Target  Info [ENU Frame]<<<<<<<<<<<<<<<<<<<<<<<<<" <<endl;
        cout << "Pos_tgt [X Y Z] : " << pos_drone_mocap[0] << " [ m ] "<< pos_drone_mocap[1] <<" [ m ] "<< pos_drone_mocap[2] <<" [ m ] "<<endl;
        cout << "Euler_tgt [Yaw] : " << Euler_mocap[2] << " [rad]  " << endl; 
    }

*/
}
//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>主 函 数<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
int main(int argc, char **argv)
{
    ros::init(argc, argv, "vicon_target");
    ros::NodeHandle nh("~");

    //读取参数表中的参数
    // 【订阅】optitrack估计位置
    ros::Subscriber optitrack_sub = nh.subscribe<geometry_msgs::TransformStamped>("/vicon/TGT05/TGT05", 10, optitrack_cb);

    //tgt topic publisher
    ros::Publisher vision_tgt_pub = nh.advertise<geometry_msgs::PoseStamped>("/target_pose", 10);

    // 频率
    ros::Rate rate(50.0);

//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>Main Loop<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
    while(ros::ok())
    {
        //回调一次 更新传感器状态
        ros::spinOnce();

        //vicon
        geometry_msgs::PoseStamped vision;
        if(flag_use_laser_or_vicon == 0)
        {
            vision.pose.position.x = pos_drone_mocap[0] ;
            vision.pose.position.y = pos_drone_mocap[1] ;
            vision.pose.position.z = pos_drone_mocap[2] ;
           // vision.pose.position.x = 10 ;
            //vision.pose.position.y = 10 ;
           // vision.pose.position.z = 10 ;

            vision.pose.orientation.x = Euler_mocap[0];
            vision.pose.orientation.y = Euler_mocap[1];
            vision.pose.orientation.z = Euler_mocap[2];
            vision.pose.orientation.w = 0;

        }
        
        vision.header.stamp = ros::Time::now();
        vision_tgt_pub.publish(vision);

        //打印
        //printf_info();
        rate.sleep();
    }

    return 0;

}
