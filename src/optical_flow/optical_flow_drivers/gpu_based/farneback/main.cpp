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
int size_x = 640;
int size_y = 480;


image_transport::Publisher image_pub_;


int main (int argc, char** argv)
{
    ros::init (argc, argv, "optical_flow_gpu_farn_node");

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
    if (!nh.getParam("size_x",       size_x))       ROS_ERROR("Failed to get param 'size_x'");
    if (!nh.getParam("size_y",       size_y))       ROS_ERROR("Failed to get param 'size_y'");


    OpticalFlow of (camera_id, size_x, size_y);


    while (ros::ok()) {
        of.renew();

        of.getOptFlowMap(8, cv::Scalar(0, 255, 255)).copyTo(cv_ptr.image);
        //of.colorizeFlow().copyTo(cv_ptr.image);

        image_pub_.publish(cv_ptr.toImageMsg());
        ros::spinOnce();
    }


    return 0;
}

