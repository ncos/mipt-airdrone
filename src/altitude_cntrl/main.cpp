#include <stdio.h>
#include <iostream>
#include <vector>
#include <math.h>

#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/common_headers.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Twist.h>

#include "pid_regulator.h"
#include "ransac.h"



ros::Subscriber sub;
ros::Publisher  pub_vel;
ros::Publisher  pub_mrk;

Floor_SAC floor_detector;

double base_height = 0.0;
visualization_msgs::Marker height_text;



void callback(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& floor_cloud)
{
    if (floor_cloud->points.size() < 10)
    {
        height_text.text = "Height = Nan";
        pub_mrk.publish(height_text);
        return;
    }
    PointCloudN::Ptr mls_cloud = floor_detector.filter(floor_cloud);
    pcl::ModelCoefficients::Ptr coeff = floor_detector.find_plane(mls_cloud);

    geometry_msgs::Twist base_cmd;
    PID pid_vel(1, 0.5, 0.5);

    base_cmd.linear.z = pid_vel.get_output(base_height, floor_detector.position.distance_to_floor);

    char text[20];
    if (!floor_detector.position.is_Nan)
    {
        sprintf(text, "Height = %f", floor_detector.position.distance_to_floor);
    }
    else
    {
        sprintf(text, "Height = Nan");
        ROS_WARN("Lost floor");
    }

    height_text.text = text;

    pub_vel.publish(base_cmd);
    pub_mrk.publish(height_text);
};






int main (int argc, char** argv)
{
    ros::init (argc, argv, "Altitude_controller_ransac");

    ros::NodeHandle nh;
    std::string input_topic  = nh.resolveName("/shrinker/depth/floor_points");
    std::string output_topic = nh.resolveName("/cmd_vel_1");
    std::string output_topic_mrk = nh.resolveName("visualization_marker");

    sub     = nh.subscribe<pcl::PointCloud<pcl::PointXYZ> > (input_topic,  1, callback);
    pub_vel = nh.advertise<geometry_msgs::Twist> (output_topic, 1);
    pub_mrk = nh.advertise<visualization_msgs::Marker >     (output_topic_mrk, 5 );

    if (!nh.getParam("base_height", base_height)) ROS_ERROR("Failed to get param 'base_height'");


    height_text.header.frame_id = "/camera_link";
    height_text.ns = "text_ns";
    height_text.action = visualization_msgs::Marker::ADD;
    height_text.id = 100;
    height_text.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    height_text.scale.z = 0.2;
    height_text.pose.position.z = 1;
    height_text.color.r = 1.0;
    height_text.color.g = 0.0;
    height_text.color.b = 0.0;
    height_text.color.a = 0.3;
    height_text.text    = "Initializing altitude controller...";
    height_text.lifetime = ros::Duration(0.2);
    pub_mrk.publish(height_text);

    ros::spin ();

    return 0;
}

