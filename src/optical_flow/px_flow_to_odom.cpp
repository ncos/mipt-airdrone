#include <ros/ros.h>
#include <optical_flow/OpticalFlow.h>
#include <nav_msgs/Odometry.h>


ros::Publisher  odom_pub;
ros::Subscriber flow_sub;
nav_msgs::Odometry odom;

void opticalflowCallback(const optical_flow::OpticalFlow::ConstPtr& flow)
{
    ROS_INFO("X velocity: [%f]", flow->velocity_x);

    odom_pub.publish(odom);
};


int main(int argc, char **argv)
{
    ros::init(argc, argv, "px_flow_to_odom_node");
    ros::NodeHandle n;

    odom.child_frame_id = "/world";
    odom.header.

    odom_pub = n.advertise<nav_msgs::Odometry>("/vo", 1000);
    flow_sub = n.subscribe("/px4flow/opt_flow", 1000, opticalflowCallback);


    ros::spin();


    return 0;
};
