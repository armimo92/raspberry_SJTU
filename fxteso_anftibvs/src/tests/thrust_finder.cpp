#include <ros/ros.h>
#include <std_msgs/String.h> 
#include <std_msgs/Header.h>
#include <std_msgs/Bool.h> 
#include <stdio.h>
#include <math.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/PositionTarget.h>
#include "mavros_msgs/SetMode.h"
#include <mavros_msgs/State.h>
#include "mavros_msgs/CommandBool.h"
#include <mavros_msgs/Thrust.h> 
#include <mavros_msgs/RCIn.h>

mavros_msgs::State current_state;
int counter = 1;
float thrust = 0;
float thrust_norm = 0;
int rc_CH_10 = 0;   //Channel for IBVS Activation ------> State 1 -> LOCAL_POSITION_CONTROL WITH VICON----------State 2 -> IBVS

void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}

void rcInCallback(const mavros_msgs::RCIn::ConstPtr& msg){
    rc_CH_10 = msg->channels[9];
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "thrust_finder");
    ros::NodeHandle nh;
    ros::Rate rate(100);
   
    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>("/mavros/state", 100, state_cb);
    ros::Subscriber rc_sub = nh.subscribe("/mavros/rc/in", 100, &rcInCallback);

    ros::Publisher cmdVel_pub = nh.advertise<geometry_msgs::TwistStamped>("/mavros/setpoint_attitude/cmd_vel",100);
    ros::Publisher normThrust_pub = nh.advertise<mavros_msgs::Thrust>("/mavros/setpoint_attitude/thrust",100);
    

    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");

    mavros_msgs::Thrust thrust_normalized;
    geometry_msgs::TwistStamped attVel2pix;
    
    attVel2pix.twist.angular.x = 0;

    attVel2pix.twist.angular.y = 0;
    attVel2pix.twist.angular.z = 0;

    thrust_normalized.header.seq = counter;
    thrust_normalized.header.stamp = ros::Time::now();
    thrust_normalized.header.frame_id = 1;
    thrust_normalized.thrust = 0;

    while(ros::ok() && !current_state.connected)
    {
       ros::spinOnce();
       rate.sleep();
       std::cout<<"Connecting"<<std::endl;
    }
    
    //send a few setpoints before starting
    for(int i = 100; ros::ok() && i > 0; --i){        
        cmdVel_pub.publish(attVel2pix);
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

        if (rc_CH_10 > 1930)
        {
       
            thrust = 9.5;
        }

        else
        {
	 
	  //thrust = 11;  //Mantiene la altura pero se queda muy arriba
	  thrust = 11;
	  thrust_norm = thrust/30;    //Thrust in N / Maximum thrust	   
          attVel2pix.header.seq = counter;
          attVel2pix.header.stamp = ros::Time::now();
          attVel2pix.header.frame_id = 1;

          attVel2pix.twist.angular.x = 0.0;
          attVel2pix.twist.angular.y = 0.0;
          attVel2pix.twist.angular.z = 0.0;

          thrust_normalized.header.seq = counter;
          thrust_normalized.header.stamp = ros::Time::now();
          thrust_normalized.header.frame_id = 1;
          thrust_normalized.thrust = thrust_norm;

          cmdVel_pub.publish(attVel2pix);
          normThrust_pub.publish(thrust_normalized);

            
          counter++;
        }
                
        ros::spinOnce();
        rate.sleep();

    }
    return 0;
}
