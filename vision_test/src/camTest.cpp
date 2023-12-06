//Including ROS libraries
#include "ros/ros.h"
#include "sensor_msgs/CompressedImage.h"
#include "sensor_msgs/image_encodings.h"
#include "cv_bridge/cv_bridge.h"
#include "image_transport/image_transport.h"
#include <std_msgs/Float64.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Quaternion.h>
//Including C++ nominal libraries
#include <iostream>
#include <math.h>
#include <cmath>
#include <vector>
//Including opencv libraries
#include <opencv2/aruco.hpp>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
//Including Eigen library
#include <eigen3/Eigen/Dense>

//Declaring global variables
cv::Mat frame;


/////////////////////ROS Subscribers//////////////////////////////////
void imageCallback(const sensor_msgs::CompressedImageConstPtr& msg)
{
	frame = cv_bridge::toCvCopy(msg, "bgr8")->image;
}
////////////////////Main program//////////////////////////////////////
int main(int argc, char *argv[])
{
	ros::init(argc, argv, "camTest");
	ros::NodeHandle nh;
        image_transport::ImageTransport it(nh);
	ros::Rate loop_rate(50);	

        ros::Subscriber sub = nh.subscribe("raspicam_node/image/compressed", 1, &imageCallback); //Real camera    

    while (ros::ok())
	{
		
		if(!frame.empty())
		{
		    std::cout << "It worked! " << std::endl;
		}
		else
		{
			std::cout << "empty " << std::endl;
		}

     		ros::spinOnce();
		loop_rate.sleep();
	}  

    return 0;
}
