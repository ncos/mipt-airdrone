#include <stdio.h>
#include <iostream>
#include <vector>
#include <math.h>

#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/common_headers.h>
#include <geometry_msgs/Twist.h>

#include "pid_regulator.h"
#include "ransac.h"



ros::Subscriber sub;
ros::Publisher  pub_vel;

Floor_SAC floor_detector;

double base_height = 0.0;



void callback(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& floor_cloud)
{
	PointCloudN::Ptr mls_cloud = floor_detector.filter(floor_cloud);
	pcl::ModelCoefficients::Ptr coeff = floor_detector.find_plane(mls_cloud);

	geometry_msgs::Twist base_cmd;
	PID pid_vel(1, 0.5, 0.5);

	base_cmd.linear.z = pid_vel.get_output(base_height, floor_detector.position.distance_to_floor);

	pub_vel.publish(base_cmd);
};






int main (int argc, char** argv)
{
	ros::init (argc, argv, "thee_seeker_of_the_floor");

	ros::NodeHandle nh;
	std::string input_topic  = nh.resolveName("/shrinker/depth/floor_points");
	std::string output_topic = nh.resolveName("/cmd_vel_1");

	sub 	= nh.subscribe<pcl::PointCloud<pcl::PointXYZ> > (input_topic,  1, callback);
	pub_vel = nh.advertise<geometry_msgs::Twist> (output_topic, 1);

	if (!nh.getParam("base_height", base_height)) ROS_ERROR("Failed to get param 'base_height'");


	ros::spin ();

	return 0;
}

