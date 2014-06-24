#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <optical_flow/OpticalFlow.h>
#include <algorithm>
#include <cmath>
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
ros::Publisher  flow_pub_;


optical_flow::OpticalFlow mat_to_msg(const cv::Mat &R)
{
    optical_flow::OpticalFlow msg;

    // extend rigid transformation to use perspectiveTransform:
    cv::Mat H = cv::Mat(3, 3, R.type());

    H.at<double>(0, 0) = R.at<double>(0, 0);
    H.at<double>(0, 1) = R.at<double>(0, 1);
    H.at<double>(0, 2) = R.at<double>(0, 2);

    H.at<double>(1, 0) = R.at<double>(1, 0);
    H.at<double>(1, 1) = R.at<double>(1, 1);
    H.at<double>(1, 2) = R.at<double>(1, 2);

    H.at<double>(2, 0) = 0.0;
    H.at<double>(2, 1) = 0.0;
    H.at<double>(2, 2) = 1.0;


    // compute perspectiveTransform on p1
    std::vector<cv::Point2f> to_transform;
    to_transform.push_back(cv::Point2f(0, 0));
    to_transform.push_back(cv::Point2f(0, 1));

    std::vector<cv::Point2f> result;
    cv::perspectiveTransform(to_transform, result, H);

    cv::Point2f shift (result.at(0).x, result.at(0).y);
    cv::Point2f e1 = to_transform.at(1);
    cv::Point2f e2 = result.at(1) - shift; // Rotated e1

    double yaw_cos = std::min(e1.ddot(e2), 1.0);
    double yaw = acos(yaw_cos) * 180 / M_PI;

    msg.flow_x = shift.x;
    msg.flow_y = shift.y;
    msg.offset_x = shift.x;
    msg.offset_y = shift.y;
    msg.ground_distance = 0;
    msg.quality = 255;
    msg.header.frame_id = "optical_flow";
    msg.header.stamp = ros::Time::now();

    ROS_INFO("(%f, %f)", result.at(0).x, result.at(0).y);

    return msg;
};



int main (int argc, char** argv)
{
	ros::init (argc, argv, "optical_flow_cpu_farn_node");

    ros::NodeHandle nh;

    image_transport::ImageTransport it_(nh);
    image_pub_ = it_.advertise("/optical_flow/camera_image", 1);
    flow_pub_  = nh.advertise<optical_flow::OpticalFlow> ("/optical_flow/opt_flow", 1);
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



        cv::Mat R = cv::estimateRigidTransform(of.flowp1, of.flowp2, false);
        if (R.data != NULL) {
            optical_flow::OpticalFlow msg = mat_to_msg(R);
            msg.dt = of.dt;
            flow_pub_.publish(msg);
        }


        ros::spinOnce();
    }


	return 0;
}

