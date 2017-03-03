/**
 * @file offb_node.cpp
 * @brief offboard example node, written with mavros version 0.14.2, px4 flight
 * stack and tested in Gazebo SITL
 */

#include <math.h>
#include <ros/ros.h>
#include <geometry_msgs/TwistStamped.h>
#include <std_msgs/Float64.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <boost/thread/mutex.hpp>

mavros_msgs::State current_state;
geometry_msgs::TwistStamped cmd_twist;
float heading_rad;
boost::mutex mtx_twist;

void state_cb(const mavros_msgs::State::ConstPtr& msg)
{
    current_state = *msg;
}

void twist_cb(const geometry_msgs::TwistStamped::ConstPtr& msg)
{
    //lock resource for concurrent access
    mtx_twist.lock();
    cmd_twist.header.stamp = msg->header.stamp;
    cmd_twist.twist.angular.z = msg->twist.angular.z;
    float x = msg->twist.linear.x;
    float y = msg->twist.linear.y;
    //convert from body referenced to earth referenced
    cmd_twist.twist.linear.x = x*sin(heading_rad) + y*cos(heading_rad);
    cmd_twist.twist.linear.y = x*cos(heading_rad) - y*sin(heading_rad);
    cmd_twist.twist.linear.z = msg->twist.linear.z;
    mtx_twist.unlock();
}

void hdg_callback(const std_msgs::Float64::ConstPtr& msg)
{
  heading_rad = msg->data*M_PI/180.0;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "offb_node");
    ros::NodeHandle nh;

    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
            ("mavros/state", 10, state_cb);
    ros::Subscriber twist_sub = nh.subscribe<geometry_msgs::TwistStamped>
            ("falcon/twist", 10, twist_cb);
    ros::Subscriber heading_sub = nh.subscribe<std_msgs::Float64>
            ("mavros/global_position/compass_hdg",10,hdg_callback);
    ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::TwistStamped>
            ("mavros/setpoint_velocity/cmd_vel", 10);
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>
            ("mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
            ("mavros/set_mode");

    //the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(20.0);

    // wait for FCU connection
    while(ros::ok() && current_state.connected){
        ros::spinOnce();
        rate.sleep();
    }

    //send a few setpoints before starting
    for(int i = 100; ros::ok() && i > 0; --i){
        local_pos_pub.publish(cmd_twist);
        ros::spinOnce();
        rate.sleep();
    }

    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";

    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

    ros::Time last_request = ros::Time::now();

    while(ros::ok()){
        if( current_state.mode != "OFFBOARD" &&
            (ros::Time::now() - last_request > ros::Duration(5.0))){
            if( set_mode_client.call(offb_set_mode) &&
                offb_set_mode.response.success){
                ROS_INFO("Offboard enabled");
            }
            last_request = ros::Time::now();
        } else {
            if( !current_state.armed &&
                (ros::Time::now() - last_request > ros::Duration(5.0))){
                if( arming_client.call(arm_cmd) &&
                    arm_cmd.response.success){
                    ROS_INFO("Vehicle armed");
                }
                last_request = ros::Time::now();
            }
        }

        mtx_twist.lock();
        local_pos_pub.publish(cmd_twist);
        mtx_twist.unlock();

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
