#include <iostream>
#include <math.h>
#include <opencv2/aruco.hpp>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <vector>
#include "ros/ros.h"
#include <eigen3/Eigen/Dense>
#include "sensor_msgs/CompressedImage.h"
#include "sensor_msgs/image_encodings.h"
#include "cv_bridge/cv_bridge.h"
#include "image_transport/image_transport.h"
#include <std_msgs/Float64.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/Vector3.h>

int keypress = cv::waitKey(25);

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "raspicam_test");

	ros::NodeHandle nh;
	ros::Rate loop_rate(50);	
	
	image_transport::ImageTransport it(nh);
	image_transport::Publisher pub = it.advertise("camera/image", 1);
	
    
    sensor_msgs::ImagePtr msg;
	cv::Mat frame;

    //DETECT ARUCO MARKER
	//OPEN THE CAMERA
	cv::VideoCapture cap(15);

	//CHECK IF CAMERA OPENED SUCCESFULLY
	if (!cap.isOpened())
	{
		std::cout << "camera didn't open" << std::endl;
		return -1;
	}

	while(ros::ok())
	{
		
		//CAPTURE FRAME
		cap >> frame;

		//If frame is empty, break
		if (frame.empty())
		{
			break;
		}

        msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", frame).toImageMsg();
        pub.publish(msg);
	
		if (keypress == 27)
		{
			break;
		}

        ros::spinOnce();
        loop_rate.sleep();

	}

	cap.release();
	cv::destroyAllWindows();
	return 0;
}


// Run program: Ctrl + F5 or Debug > Start Without Debugging menu
// Debug program: F5 or Debug > Start Debugging menu

// Tips for Getting Started: 
//   1. Use the Solution Explorer window to add/manage files
//   2. Use the Team Explorer window to connect to source control
//   3. Use the Output window to see build output and other messages
//   4. Use the Error List window to view errors
//   5. Go to Project > Add New Item to create new code files, or Project > Add Existing Item to add existing code files to the project
//   6. In the future, to open this project again, go to File > Open > Project and select the .sln file	
