#include <iostream>
#include "ros/ros.h"
#include "std_msgs/Float64.h"
#include "geometry_msgs/Twist.h"

float x = 0, theta_z = 0;
double velocityMultiplier, angleMultiplier;
bool firstCallback = false;

void commandCallback(const geometry_msgs::Twist::ConstPtr& msg)
{

    x = msg->linear.x*velocityMultiplier;
    theta_z = msg->angular.z*angleMultiplier;

    ROS_INFO("x : %f", x);
    ROS_INFO("theta_z : %f", theta_z);
}

int main(int argc, char **argv){
    ros::init(argc, argv, "controller_node");

    std_msgs::Float64 speedMsg;
    std_msgs::Float64 angleMsg;

    ros::NodeHandle n;

    ros::Subscriber sub = n.subscribe("/robot_command", 1000, commandCallback);

    
    ros::Publisher wheel3VelocityPub = n.advertise<std_msgs::Float64>("/wheel3_velocity_controller/command", 1000);
    ros::Publisher wheel4VelocityPub = n.advertise<std_msgs::Float64>("/wheel4_velocity_controller/command", 1000);

    ros::Publisher wheel3AnglePub = n.advertise<std_msgs::Float64>("/wheel3_angle_controller/command", 1000);
    ros::Publisher wheel4AnglePub = n.advertise<std_msgs::Float64>("/wheel4_angle_controller/command", 1000);

    ros::Rate loop_rate(10);

    if (!n.hasParam("velocity_multiplier"))
    {
        ROS_INFO("No param named 'velocity_multiplier'");
    }

    n.param("velocity_multiplier", velocityMultiplier, 1.0);
    n.param("angle_multiplier", angleMultiplier, 1.0);

    ROS_INFO("velocity_multiplier: %f", velocityMultiplier);
    ROS_INFO("angle_multiplier: %f", angleMultiplier);

    speedMsg.data = 0;
    angleMsg.data = 0;

    wheel3VelocityPub.publish(speedMsg);
    wheel4VelocityPub.publish(speedMsg);

    wheel3AnglePub.publish(angleMsg);
    wheel4AnglePub.publish(angleMsg);

    while(ros::ok()){

        ROS_INFO("x : %f", x);
        ROS_INFO("theta_z : %f", theta_z);

        speedMsg.data = x;
        angleMsg.data = theta_z;

        wheel3VelocityPub.publish(speedMsg);
        wheel4VelocityPub.publish(speedMsg);

        wheel3AnglePub.publish(angleMsg);
        wheel4AnglePub.publish(angleMsg);

        ros::spinOnce();
        loop_rate.sleep();

    }

    return 0;
}