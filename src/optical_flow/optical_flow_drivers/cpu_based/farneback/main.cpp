#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include "flow.h"


// Check the http://docs.opencv.org/trunk/modules/gpuoptflow/doc/optflow.html

int camera_id = 0;
int numLevels = 5;
double pyrScale = 0.5;
bool fastPyramids = false;
int winSize = 13;
int numIters = 10;
int polyN = 5;
double polySigma = 1.1;
int flags = 0;
double resize_scale = 0.5;


image_transport::Publisher image_pub_;


int main (int argc, char** argv)
{
	ros::init (argc, argv, "optical_flow_cpu_farn_node");

    ros::NodeHandle nh;

    image_transport::ImageTransport it_(nh);
    image_pub_ = it_.advertise("/optical_flow/camera_image", 1);
    cv_bridge::CvImage cv_ptr;
    cv_ptr.encoding = sensor_msgs::image_encodings::BGR8;


    if (!nh.getParam("camera_id",    camera_id))    ROS_ERROR("Failed to get param 'camera_id'");
    if (!nh.getParam("numLevels",    numLevels))    ROS_ERROR("Failed to get param 'numLevels'");
    if (!nh.getParam("pyrScale",     pyrScale))     ROS_ERROR("Failed to get param 'pyrScale'");
    if (!nh.getParam("fastPyramids", fastPyramids)) ROS_ERROR("Failed to get param 'fastPyramids'");
    if (!nh.getParam("winSize",      winSize))      ROS_ERROR("Failed to get param 'winSize'");
    if (!nh.getParam("numIters",     numIters))     ROS_ERROR("Failed to get param 'numIters'");
    if (!nh.getParam("polyN",        polyN))        ROS_ERROR("Failed to get param 'polyN'");
    if (!nh.getParam("polySigma",    polySigma))    ROS_ERROR("Failed to get param 'polySigma'");
    if (!nh.getParam("flags",        flags))        ROS_ERROR("Failed to get param 'flags'");
    if (!nh.getParam("resize_scale", resize_scale)) ROS_ERROR("Failed to get param 'resize_scale'");


	OpticalFlow of (camera_id);


    while (ros::ok()) {
        of.renew();

        of.cflow.copyTo(cv_ptr.image);

        image_pub_.publish(cv_ptr.toImageMsg());
        ros::spinOnce();
    }


	return 0;
}

