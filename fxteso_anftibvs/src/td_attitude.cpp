//Including ROS libraries
#include "ros/ros.h"
#include <std_msgs/Float64.h>
#include <std_msgs/Bool.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Twist.h>
//Including C++ nominal libraries
#include <iostream>
#include <math.h>
#include <vector>
//Including Eigen library
#include <eigen3/Eigen/Dense>

float step = 0.01;
bool flag = true;

Eigen::Vector3f attitude(0,0,0); 
Eigen::Vector3f attitude_est(0,0,0);
Eigen::Vector3f attitude_vel_est(0,0,0);

Eigen::Vector3f x1_hat_dot(0,0,0);
Eigen::Vector3f x2_hat_dot(0,0,0);
Eigen::Vector3f estimation_error(0,0,0);
Eigen::Vector3f G1(0,0,0);
Eigen::Vector3f G2(0,0,0);

Eigen::Vector3f position(0,0,0);
Eigen::Vector3f position_estimate(0,0,0);
Eigen::Vector3f velocity_estimate(0,0,0);

Eigen::Vector3f x1_hat_dot_pos(0,0,0);
Eigen::Vector3f x2_hat_dot_pos(0,0,0);
Eigen::Vector3f estimation_error_pos(0,0,0);
Eigen::Vector3f k1(0,0,0);
Eigen::Vector3f k2(0,0,0);

float alpha_1, alpha_2, beta_1, beta_2;

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

float sig(float a, float b) //sig = |a|^b * sign(a)
{
    float s;
    s = powf(std::abs(a), b) * sign(a);
    return s;
}

void attCallback(const geometry_msgs::Vector3::ConstPtr& att)
{
	attitude(0) = att->x;
	attitude(1) = att->y;
	attitude(2) = att->z;
}

// void posCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
// {
// 	position(0) = msg->pose.position.x;
// 	position(1) = msg->pose.position.y;
// 	position(2) = msg->pose.position.z;
// }

void flagCallback(const std_msgs::Bool::ConstPtr& msg)
{
    flag = msg->data;
}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "td_attitude");
	ros::NodeHandle nh;
	ros::Rate loop_rate(100);	

    //ROS publishers and subscribers
    ros::Subscriber quad_attitude_sub = nh.subscribe("quad_attitude",100, &attCallback);
    // ros::Subscriber quad_position_sub = nh.subscribe("/mavros/vision_pose/pose",100, &posCallback);
    ros::Subscriber flag_sub = nh.subscribe("flag_topic", 100, &flagCallback);

    ros::Publisher attitude_est_pub = nh.advertise<geometry_msgs::Twist>("attitude_estimates",100);
    ros::Publisher attitude_est_err_pub = nh.advertise<geometry_msgs::Vector3>("attitude_td_error",100);
    
    // ros::Publisher position_estimates_pub = nh.advertise<geometry_msgs::Twist>("vicon_estimates",100);
    // ros::Publisher position_est_err_pub = nh.advertise<geometry_msgs::Vector3>("position_td_error",100);

    // geometry_msgs::Twist position_est_var;
    // geometry_msgs::Vector3 position_est_err_var;
    geometry_msgs::Twist attitude_est_var;
    geometry_msgs::Vector3 attitude_est_err_var;
 
    attitude_est << attitude(0),attitude(1),attitude(2);
    attitude_vel_est << 0,0,0;
    estimation_error << 0,0,0;
    x1_hat_dot << 0,0,0;
    x2_hat_dot << 0,0,0;

    G1 << 5, 5, 5;
    G2 << 6, 6, 6;

    // position_estimate << position(0), position(1), position(2);
    // velocity_estimate << 0,0,0;
    // estimation_error_pos << 0,0,0;
    // x1_hat_dot_pos << 0,0,0;
    // x2_hat_dot_pos << 0,0,0;
    // k1 << 5, 5, 5;
    // k2 << 6, 6, 6;
  
      
    alpha_1 = 0.7;
    alpha_2 = 2*alpha_1 - 1;
    

    beta_1 = 1.2;
    beta_2 = 2*beta_1 - 1;
    
 
  
    attitude_est_var.linear.x = 0;
    attitude_est_var.linear.y = 0;
    attitude_est_var.linear.z = 0;

    attitude_est_var.angular.x = 0;
    attitude_est_var.angular.y = 0;
    attitude_est_var.angular.z = 0;

    attitude_est_err_var.x = 0;
    attitude_est_err_var.y = 0;
    attitude_est_err_var.z = 0;

    // position_est_var.linear.x = position_estimate(0);
    // position_est_var.linear.y = position_estimate(1);
    // position_est_var.linear.z = position_estimate(2);

    // position_est_var.angular.x = 0;
    // position_est_var.angular.y = 0;
    // position_est_var.angular.z = 0;

    // position_est_err_var.x = 0;
    // position_est_err_var.y = 0;
    // position_est_err_var.z = 0;

   
    attitude_est_pub.publish(attitude_est_var);
    attitude_est_err_pub.publish(attitude_est_err_var);
    
    // position_estimates_pub.publish(position_est_var);
    // position_est_err_pub.publish(position_est_err_var);

    //ros::Duration(4.5).sleep();

    while(ros::ok())
    {
        /////// Tracking Differentiator ///////

        //x2_hat_dot = Q1*sig(error,alpha2) + D1*sig(error,beta2)
        //x1_hat_dot = x2_hat + Q2*sig(error,alpha1) + D2*sig(error,beta1)
        //Q1,Q2,D1,D2 > 0 such that the polynomial s^2 + 2s + 2 is Hurwitz
        
        /*
        if (flag == 0)
        {
            for(int i = 0; i<=2; i++)
            {
                estimation_error(i) = 0;
                attitude_est(i) = attitude(i);
                attitude_vel_est(i) = 0;
                x2_hat_dot(i) = 0;
                x1_hat_dot(i) = 0;

                // estimation_error_pos(i) = 0;
                // position_estimate(i) = position(i);
                // velocity_estimate(i) = 0;
                // x2_hat_dot_pos(i) = 0;
                // x1_hat_dot_pos(i) = 0;

            }
        }
        else
        {*/
            for(int i = 0; i<=2; i++)
            {
                estimation_error(i) = attitude(i) - attitude_est(i);
    
                x2_hat_dot(i) = G2(i)*sig(estimation_error(i), alpha_2) + G2(i)*sig(estimation_error(i), beta_2);
                attitude_vel_est(i) = attitude_vel_est(i) + x2_hat_dot(i)*step;

                x1_hat_dot(i) = attitude_vel_est(i) + G1(i)*sig(estimation_error(i), alpha_1) + G1(i)*sig(estimation_error(i), beta_1);
                attitude_est(i) = attitude_est(i) + x1_hat_dot(i)*step;   

            //  estimation_error_pos(i) = position(i) - position_estimate(i);
    
            //  x2_hat_dot_pos(i) = k2(i)*sig(estimation_error_pos(i), alpha_2) + k2(i)*sig(estimation_error_pos(i), beta_2);
            //  velocity_estimate(i) = velocity_estimate(i) + x2_hat_dot_pos(i)*step;

            //  x1_hat_dot_pos(i) = velocity_estimate(i) + k1(i)*sig(estimation_error_pos(i), alpha_1) + k1(i)*sig(estimation_error_pos(i), beta_1);
            //  position_estimate(i) = position_estimate(i) + x1_hat_dot_pos(i)*step;   
             
            }
        //}    
        
        //Estimations for attitude measurements
        attitude_est_var.linear.x = attitude_est(0);
        attitude_est_var.linear.y = attitude_est(1);
        attitude_est_var.linear.z = attitude_est(2);

        //Estimations for attitude velocity measurements
        attitude_est_var.angular.x = attitude_vel_est(0);
        attitude_est_var.angular.y = attitude_vel_est(1);
        attitude_est_var.angular.z = attitude_vel_est(2);

        attitude_est_err_var.x = estimation_error(0);
        attitude_est_err_var.y = estimation_error(1);
        attitude_est_err_var.z = estimation_error(2);

        ////Estimation for position measurements VICON
        // position_est_var.linear.x = position_estimate(0);
        // position_est_var.linear.y = position_estimate(1);
        // position_est_var.linear.z = position_estimate(2);
        
        ////Estimation for velocity measurements VICON
        // position_est_var.angular.x = velocity_estimate(0);
        // position_est_var.angular.y = velocity_estimate(1);
        // position_est_var.angular.z = velocity_estimate(2);

        // position_est_err_var.x = estimation_error_pos(0);
        // position_est_err_var.y = estimation_error_pos(1);
        // position_est_err_var.z = estimation_error_pos(2);

        attitude_est_pub.publish(attitude_est_var);
        attitude_est_err_pub.publish(attitude_est_err_var);

        // position_estimates_pub.publish(position_est_var);
        // position_est_err_pub.publish(position_est_err_var);

    
        ros::spinOnce();
		loop_rate.sleep();
    }
	return 0;
}
