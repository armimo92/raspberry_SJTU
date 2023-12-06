#include <ros/ros.h>
#include <std_msgs/String.h> 
#include <std_msgs/Header.h>
#include <stdio.h>
#include <math.h>
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Vector3Stamped.h"
#include "std_msgs/Float32.h"
#include "geometry_msgs/Vector3.h"
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/CommandBool.h>
#include <std_msgs/Bool.h> 

 
//#include </home/mahesh/catkin_ws/src/beginner_tutorials/src/Qualisys.h>
mavros_msgs::State current_state;
void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}
//
int main(int argc, char **argv)
{
    ros::init(argc, argv, "position_setpoints");
    ros::NodeHandle nh;
    
    ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>("mavros/setpoint_position/local", 100);

    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>("/mavros/state", 100, state_cb);

    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");

    ros::Rate rate(50);
    ros::spinOnce();
   
    geometry_msgs::PoseStamped pose;
    pose.pose.position.x = 0;
    pose.pose.position.y = 0;
    pose.pose.position.z = 0.8;
   
    while(ros::ok() && !current_state.connected)
    {
       ros::spinOnce();
       rate.sleep();
       std::cout<<"Connecting"<<std::endl;
    }
    
    //send a few setpoints before starting
    for(int i = 100; ros::ok() && i > 0; --i){        
        local_pos_pub.publish(pose);
        ros::spinOnce();
        rate.sleep();
    }
    
    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";

    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

    ros::Time last_request = ros::Time::now();
     
   while(ros::ok()){

    if( current_state.mode != "OFFBOARD" && (ros::Time::now() - last_request > ros::Duration(5.0)))
    {
        if( set_mode_client.call(offb_set_mode) && offb_set_mode.response.mode_sent)
        {
            ROS_INFO("Offboard enabled");
        }
        last_request = ros::Time::now();
    } 
    else 
    {
        if( !current_state.armed && (ros::Time::now() - last_request > ros::Duration(5.0)))
        {
            if( arming_client.call(arm_cmd) && arm_cmd.response.success)
            {
                ROS_INFO("Vehicle armed");
            }
        last_request = ros::Time::now();
        }
    } 
    
    local_pos_pub.publish(pose);

    
    ros::spinOnce();
    rate.sleep();

   }
   return 0;
}
