//Including ROS libraries
#include "ros/ros.h"
#include "sensor_msgs/CompressedImage.h"
#include "sensor_msgs/image_encodings.h"
#include <std_msgs/Float64.h>
#include <std_msgs/Bool.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Quaternion.h>
//Including C++ nominal libraries
#include <iostream>
#include <math.h>
#include <vector>
//Including Eigen library
#include <eigen3/Eigen/Dense>

float step = 0.01;

Eigen::Vector4f imFeat; //qx, qy, qz, qpsi
Eigen::Vector4f ctrl_input; //vDot_x, vDot_y, vDot_z, vDot_yaw
Eigen::Vector4f imFeat_estimate(0,0,1,0);
Eigen::Vector4f imFeat_estimate_dot(0,0,0,0);
Eigen::Vector4f ibvs_dist(0,0,0,0);
Eigen::Vector4f estimation_error(0,0,0,0);
Eigen::Vector3f scaled_dist(0,0,0);

Eigen::Vector4f x1_hat(0,0,0,0);
Eigen::Vector4f x2_hat(0,0,0,0);
Eigen::Vector4f x3_hat(0,0,0,0);

Eigen::Vector4f x1_hat_dot(0,0,0,0);
Eigen::Vector4f x2_hat_dot(0,0,0,0);
Eigen::Vector4f x3_hat_dot(0,0,0,0);

Eigen::Vector4f gamma1;
Eigen::Vector4f gamma2;
Eigen::Vector4f gamma3;
Eigen::Vector4f gamma4;

Eigen::Vector4f mu1;
Eigen::Vector4f mu2;
Eigen::Vector4f mu3;

float alpha = 0;
float alpha2 = 0;
float alpha3 = 0;

float beta = 0;
float beta2 = 0;
float beta3 = 0;

float fx = 0;
float gx_u = 0;
float quad_mass = 0.835;
float z_des = 1.2;
//float z_des = 2.5;

float yawRate_desired = 0;
bool flag_arucos = false;
bool flag=false;

Eigen::Vector3f quad_att(0,0,0);
Eigen::Vector3f dist_linear(0,0,0);

Eigen::Matrix3f Ryaw(float yaw)
{   
    Eigen::Matrix3f yaw_mat;
    yaw_mat << cos(yaw), -sin(yaw),0,
                sin(yaw), cos(yaw), 0,
                0, 0, 1;
    return yaw_mat;
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

float sig(float a, float b) //sig = |a|^b * sign(a)
{
    float s;
    s = powf(std::abs(a), b) * sign(a);
    return s;
}

void quadAttCallback(const geometry_msgs::Vector3::ConstPtr& quadAtt)
{
	quad_att(0) = quadAtt->x;
    quad_att(1) = quadAtt->y;
    quad_att(2) = quadAtt->z;
}

void imFeatCallback(const geometry_msgs::Quaternion::ConstPtr& img_features)
{
	imFeat(0) = img_features->x;
	imFeat(1) = img_features->y;
	imFeat(2) = img_features->z;
    imFeat(3) = img_features->w;
}

void ibvsCtrlCallback(const geometry_msgs::Quaternion::ConstPtr& ctrl)
{
	ctrl_input(0) = ctrl->x;
	ctrl_input(1) = ctrl->y;
	ctrl_input(2) = ctrl->z;
    ctrl_input(3) = ctrl->w;
}

void yawRateCallback(const geometry_msgs::Quaternion::ConstPtr& yr)
{
	yawRate_desired = yr->w;
}

void flagArucosCallback(const std_msgs::Bool::ConstPtr& msg)
{
	flag_arucos = msg->data;
}

void flagCallback(const std_msgs::Bool::ConstPtr& msg)
{
        flag = msg->data;
}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "fixed_eso");
	ros::NodeHandle nh;
	ros::Rate loop_rate(100);	

    //ROS publishers and subscribers
    ros::Subscriber im_feat_sub = nh.subscribe("ImFeat_vector", 100, &imFeatCallback);  
    ros::Subscriber ctrl_sub = nh.subscribe("ibvs_control_input", 100, &ibvsCtrlCallback);  
    ros::Subscriber yawRate_sub = nh.subscribe("desired_attitude", 100, &yawRateCallback);
    ros::Subscriber quad_att_sub = nh.subscribe("quad_attitude", 100, &quadAttCallback);
    ros::Subscriber flag_arucos_sub = nh.subscribe("flag_arucos",100, &flagArucosCallback);
    ros::Subscriber flag_sub = nh.subscribe("flag_topic",100, &flagCallback);

    ros::Publisher im_feat_est_pub = nh.advertise<geometry_msgs::Quaternion>("ImFeat_estimates_fxt",100);
    ros::Publisher im_feat_est_err_pub = nh.advertise<geometry_msgs::Quaternion>("estimation_error_fxt",100);
    ros::Publisher im_feat_est_dot_pub = nh.advertise<geometry_msgs::Quaternion>("ImFeat_dot_estimates_fxt",100);
    ros::Publisher dist_est_pub = nh.advertise<geometry_msgs::Quaternion>("ibvs_dist",100);
    ros::Publisher scaled_dist_pub = nh.advertise<geometry_msgs::Quaternion>("scaled_ibvs_dist",100);

    geometry_msgs::Quaternion im_feat_est_var;
    geometry_msgs::Quaternion im_feat_est_dot_var;
    geometry_msgs::Quaternion im_feat_est_err_var;
    geometry_msgs::Quaternion ibvs_dist_var;
    geometry_msgs::Quaternion scaled_ibvs_dist_var;

    imFeat_estimate << imFeat(0), imFeat(1), imFeat(2), imFeat(3);
    imFeat_estimate_dot << 0,0,0,0;
    ibvs_dist << 0, 0, 0, 0;

    estimation_error << 0, 0, 0, 0;
    x1_hat_dot << 0, 0, 0, 0;
    x2_hat_dot << 0, 0, 0, 0;
    x3_hat_dot << 0, 0, 0, 0;

    // //GAINS WITH MODEL UNCERTAINTIES
   gamma1 << 8, 8, 16, 5;
   gamma2 << 16, 16, 14, 16;
   gamma3 << 7, 7, 21, 7;
   gamma4 << 0.0001, 0.0001, 0.001, 0.001;
  
     /*
     
    gamma1 << 16, 16, 16, 12;
   gamma2 << 14, 14, 14, 14;
   gamma3 << 6, 6, 21, 3;
   gamma4 << 0.0001, 0.0001, 0.001, 0.001;
*/


    
    mu1 = gamma1;
    mu2 = gamma2;
    mu3 = gamma3;
    

    im_feat_est_var.x = imFeat_estimate(0);
    im_feat_est_var.y = imFeat_estimate(1);
    im_feat_est_var.z = imFeat_estimate(2);
    im_feat_est_var.w = imFeat_estimate(3);

    im_feat_est_dot_var.x = imFeat_estimate_dot(0);
    im_feat_est_dot_var.y = imFeat_estimate_dot(1);
    im_feat_est_dot_var.z = imFeat_estimate_dot(2);
    im_feat_est_dot_var.w = imFeat_estimate_dot(3);

    im_feat_est_err_var.x = estimation_error(0);
    im_feat_est_err_var.y = estimation_error(1);
    im_feat_est_err_var.z = estimation_error(2);
    im_feat_est_err_var.w = estimation_error(3);

    ibvs_dist_var.x = 0;
    ibvs_dist_var.y = 0;
    ibvs_dist_var.z = 0;
    ibvs_dist_var.w = 0;

    scaled_ibvs_dist_var.x = 0;
    scaled_ibvs_dist_var.y = 0;
    scaled_ibvs_dist_var.z = 0;
    scaled_ibvs_dist_var.w = 0;

    im_feat_est_pub.publish(im_feat_est_var);
    im_feat_est_dot_pub.publish(im_feat_est_dot_var);
    im_feat_est_err_pub.publish(im_feat_est_err_var);
    dist_est_pub.publish(ibvs_dist_var);
    scaled_dist_pub.publish(scaled_ibvs_dist_var);

    ros::Duration(1).sleep();

    while(ros::ok())
    {
	if (flag_arucos == 0 || flag == 0)
	{
		for (int i = 0; i<=3; i++) 
		{
			estimation_error(i) = 0;
			x3_hat_dot(i) = 0;
			x2_hat_dot(i) = 0;
			x1_hat_dot(i) = 0;
			imFeat_estimate_dot(i) = 0;
			ibvs_dist(i) = 0;
			imFeat_estimate(i) = imFeat(i);
		}
	}
	else
	{
        	for(int i = 0; i<=3; i++)
	        {
        	    estimation_error(i) = imFeat(i) - imFeat_estimate(i);

	            //Defining f(x) and g(x)u
	            if(i == 0)
	            {   
	                alpha = 0.75;
	                beta = 1.2;
	
	                alpha2 = (alpha + 1)/2;
	                beta2 = (beta + 1)/2;
    
	                alpha3 = (alpha + 2)/3;
	                beta3 = (beta + 2)/3;

	                fx =0; //imFeat_estimate_dot(1) * yawRate_desired + ctrl_input(3) * imFeat_estimate(1);
	                gx_u = -(1/z_des)*ctrl_input(0);
	            }
        	    else if (i == 1)
	            {
	       	        alpha = 0.75;
        	        beta = 1.2;

                	alpha2 = (alpha + 1)/2;
	                beta2 = (beta + 1)/2;
	    
	                alpha3 = (alpha + 2)/3;
	                beta3 = (beta + 2)/3;

	                fx = 0;//-imFeat_estimate_dot(0) * yawRate_desired - ctrl_input(3) * imFeat_estimate(0);
	                gx_u = -(1/z_des)*ctrl_input(1);
        	    }
	            else if (i == 2)
	            {
	                alpha = 0.75;
        	        beta = 1.2;

	                alpha2 = (alpha + 1)/2;
	                beta2 = (beta + 1)/2;
    
	                alpha3 = (alpha + 2)/3;
	                beta3 = (beta + 2)/3;
	                fx = 0;
	                gx_u = -(1/z_des)*ctrl_input(2);
        	    }
	            else if (i == 3)
        	    {
	                alpha = 0.75;
	                beta = 1.2;

	                alpha2 = (alpha + 1)/2;
	                beta2 = (beta + 1)/2;
    
        	        alpha3 = (alpha + 2)/3;
	                beta3 = (beta + 2)/3;
	                fx = 0;
	                gx_u = ctrl_input(3);
	            }

	            x3_hat_dot(i) = gamma3(i) * sig(estimation_error(i),alpha) + mu3(i) * sig(estimation_error(i),beta) + gamma4(i) * sign(estimation_error(i));
        	    ibvs_dist(i) = ibvs_dist(i) + x3_hat_dot(i) * step;

	            x2_hat_dot(i) = ibvs_dist(i) + fx + gx_u + gamma2(i)*sig(estimation_error(i), alpha2) + mu2(i)*sig(estimation_error(i), beta2);
        	    imFeat_estimate_dot(i) = imFeat_estimate_dot(i) + x2_hat_dot(i)*step;

	            x1_hat_dot(i) = imFeat_estimate_dot(i) + gamma1(i) * sig(estimation_error(i), alpha3) + mu1(i) * sig(estimation_error(i), beta3);
	            imFeat_estimate(i) = imFeat_estimate(i) + x1_hat_dot(i)*step;   
                  
       		}
	}

        dist_linear << ibvs_dist(0), ibvs_dist(1),ibvs_dist(2);
        scaled_dist = Ryaw(quad_att(2)).transpose() * (quad_mass * z_des * dist_linear);

        im_feat_est_var.x = imFeat_estimate(0);
        im_feat_est_var.y = imFeat_estimate(1);
        im_feat_est_var.z = imFeat_estimate(2);
        im_feat_est_var.w = imFeat_estimate(3);

        im_feat_est_dot_var.x = imFeat_estimate_dot(0);
        im_feat_est_dot_var.y = imFeat_estimate_dot(1);
        im_feat_est_dot_var.z = imFeat_estimate_dot(2);
        im_feat_est_dot_var.w = imFeat_estimate_dot(3);

        im_feat_est_err_var.x = estimation_error(0);
        im_feat_est_err_var.y = estimation_error(1);
        im_feat_est_err_var.z = estimation_error(2);
        im_feat_est_err_var.w = estimation_error(3);
        
        ibvs_dist_var.x = ibvs_dist(0);
        ibvs_dist_var.y = ibvs_dist(1);
        ibvs_dist_var.z = ibvs_dist(2);
        ibvs_dist_var.w = ibvs_dist(3);

        scaled_ibvs_dist_var.x = scaled_dist(0);
        scaled_ibvs_dist_var.y = scaled_dist(1);
        scaled_ibvs_dist_var.z = scaled_dist(2);
        scaled_ibvs_dist_var.w = quad_mass * z_des * ibvs_dist(3);;

        im_feat_est_err_pub.publish(im_feat_est_err_var);
        im_feat_est_pub.publish(im_feat_est_var);
        im_feat_est_dot_pub.publish(im_feat_est_dot_var);
        dist_est_pub.publish(ibvs_dist_var);
        scaled_dist_pub.publish(scaled_ibvs_dist_var);

        ros::spinOnce();
		loop_rate.sleep();
    }

    return 0;
}
