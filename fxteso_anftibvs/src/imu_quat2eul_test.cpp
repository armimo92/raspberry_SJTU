#include <math.h>
#include <cmath>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/Bool.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/RCIn.h>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry> 

Eigen::Vector4f receivedQuaternion;
Eigen::Vector3f receivedLinearAcceleration;
Eigen::Vector3f receivedAngularVelocity;
Eigen::Vector3f linVel(0,0,0);
Eigen::Vector3f ang(0,0,0);
Eigen::Quaterniond r;
Eigen::Vector3f attitude(0,0,0);
float step = 0.01;
float offset = 0;
float corrected_value;
float rango = 2*M_PI;
float nWraps;
bool flag = 0;
int rc_CH_7 = 0;
int rc_CH_10 = 0;

mavros_msgs::State current_state;

// Eigen::Vector3d quaternion_to_rpy2(const Eigen::Quaterniond &q)
// {
//         // YPR - ZYX
//         return q.toRotationMatrix().eulerAngles(2, 1, 0).reverse();
// }

Eigen::Vector3f Euler2Quaternion(const Eigen::Vector4f &q)
{
    Eigen::Vector3f output;
    //quat = [x, y, z, w]
    float sinr_cosp = 2 * (q(3) * q(0) + q(1) * q(2));
    float cosr_cosp = 1 - (2 * (q(0) * q(0) + q(1) * q(1)));
    output(0) = std::atan2(sinr_cosp, cosr_cosp);

    float sinp = std::sqrt(1 + 2 * (q(3) * q(1) - q(0) * q(2)));
    float cosp = std::sqrt(1 - 2 * (q(3) * q(1) - q(0) * q(2)));
    output(1) = 2 * std::atan2(sinp, cosp) - M_PI / 2;

    float siny_cosp = 2 * (q(3) * q(2) + q(0) * q(1));
    float cosy_cosp = 1 - 2 * (q(1) * q(1) + q(2) * q(2));
    output(2) = std::atan2(siny_cosp, cosy_cosp);

    return output;
}


void state_cb(const mavros_msgs::State::ConstPtr& msg)
{
    current_state = *msg;
}

float sign(float var)
{
    float result;
    if(var>0)
    {
        result = 1;
    }
    else if(var<0)
    {
        result = -1;
    }
    else if (var == 0)
    {
        result = 0;
    }
    return result;
}

void imuCallback(const sensor_msgs::Imu::ConstPtr& msg)
{
	receivedQuaternion(0) = msg->orientation.x;
    receivedQuaternion(1) = msg->orientation.y;
    receivedQuaternion(2) = msg->orientation.z;
    receivedQuaternion(3) = msg->orientation.w;

    receivedAngularVelocity(0) = msg->angular_velocity.x;
    receivedAngularVelocity(1) = msg->angular_velocity.y;
    receivedAngularVelocity(2) = msg->angular_velocity.z;

    receivedLinearAcceleration(0) = msg->linear_acceleration.x;
    receivedLinearAcceleration(1) = msg->linear_acceleration.y;
    receivedLinearAcceleration(2) = msg->linear_acceleration.z;
}

// void viconCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
// {
// 	receivedQuaternion(0) = msg->pose.orientation.x;
//     receivedQuaternion(1) = msg->pose.orientation.y;
//     receivedQuaternion(2) = msg->pose.orientation.z;
//     receivedQuaternion(3) = msg->pose.orientation.w;
// }
void rcInCallback(const mavros_msgs::RCIn::ConstPtr& msg){
    rc_CH_7 = msg->channels[6];
    rc_CH_10 = msg->channels[9];
}

int main(int argc, char **argv)
{   
    
    ros::init(argc, argv, "imu_quat2eul_test");
    ros::NodeHandle nh;
    ros::Rate rate(100.0);

    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>("mavros/state", 10, &state_cb);
    ros::Subscriber rc_sub = nh.subscribe("/mavros/rc/in", 100, &rcInCallback);
    
    ros::Subscriber imu_data_sub = nh.subscribe("/mavros/imu/data", 100, &imuCallback);
    //ros::Subscriber vicon_data_sub = nh.subscribe("/mavros/vision_pose/pose", 100, &viconCallback);
	ros::Publisher attitude_pub = nh.advertise<geometry_msgs::Vector3>("quad_attitude",100);
    //ros::Publisher lin_vel_pub = nh.advertise<geometry_msgs::Vector3>("linear_velocity",100);

    geometry_msgs::Vector3 attitude_var;
    //geometry_msgs::Vector3 lin_vel_var;
    
    tf2::Quaternion q;
	tf2::Quaternion q_rot;
    

    attitude_var.x = 0;
    attitude_var.y = 0;
    attitude_var.z = 0;

    // lin_vel_var.x = 0;
    // lin_vel_var.y = 0;
    // lin_vel_var.z = 0;

    attitude_pub.publish(attitude_var);
    //lin_vel_pub.publish(lin_vel_var);
    
    
    while(ros::ok())
    {  
        attitude = Euler2Quaternion(receivedQuaternion);

        corrected_value = attitude(2) + offset;

        nWraps = std::floor(corrected_value/rango);

        corrected_value = corrected_value - (nWraps*rango);

        if(corrected_value < -M_PI)
        {
            corrected_value = corrected_value + rango;
        }
        if(corrected_value > M_PI)
        {
            corrected_value = corrected_value - rango;
        }

        attitude_var.x = attitude(0)-0;
        attitude_var.y = -attitude(1)-0;
        attitude_var.z = -corrected_value;
 
        // if (rc_CH_10<1930)
        // {
        //     flag = 1;
        // }
        // else
        // {
        //     flag = 0;
        // 
        // if(flag==0)
        // {
        //     attitude(0) = 0;
        //     attitude(1) = 0;
        //     attitude(2) = 0;
        //     corrected_value = 0;
        //     attitude_var.x = attitude(0);
        //     attitude_var.y = attitude(1);
        //     attitude_var.z = attitude(2);
        // }
        // else
        // {
        //     attitude = Euler2Quaternion(receivedQuaternion);

        //     corrected_value = attitude(2) + offset;

        //     nWraps = std::floor(corrected_value/rango);

        //     corrected_value = corrected_value - (nWraps*rango);

        //     if(corrected_value < -M_PI)
        //     {
        //         corrected_value = corrected_value + rango;
        //     }

        //     if(corrected_value > M_PI)
        //     {
        //         corrected_value = corrected_value - rango;
        //     }

        //     attitude_var.x = attitude(0);
        //     attitude_var.y = -attitude(1);
        //     attitude_var.z = -corrected_value;
        // }
        
        // linVel(0) = linVel(0) + receivedLinearAcceleration(0)*0.01;
        // linVel(1) = linVel(1) + receivedLinearAcceleration(1)*0.01;
        // linVel(2) = linVel(2) + receivedLinearAcceleration(2)*0.01;

        // lin_vel_var.x = linVel(0)-0.1;
        // lin_vel_var.y = linVel(1)-0.4;
        // lin_vel_var.z = linVel(2)-9.81;

        attitude_pub.publish(attitude_var);
        //lin_vel_pub.publish(lin_vel_var);

        // std::cout<< attitude(0)<< ", " << attitude(1) << ", " << corrected_value <<std::endl;
        
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
