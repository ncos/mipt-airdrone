#include <ros/ros.h>
#include "detector.h"

ros::Publisher pub_mrk;
ros::Subscriber camera_info_sub;
int target_seq_num;
std::string base_footprint_frame;
std::string downward_camera_frame;
std::string kinect_depth_optical_frame;
boost::shared_ptr<PointEstimator> point_est;
boost::shared_ptr<DownwardCamera> dnw_cam;

int main (int argc, char** argv)
{
	ros::init (argc, argv, "marker_detector");

	ros::NodeHandle nh;

	std::string input_image_topic;
	std::string landing_marker_topic;
	std::string input_camera_info_topic;
	if (!nh.getParam("marker_detector/input_image_topic", input_image_topic)) input_image_topic = "/sensors/downward_camera/image";
	if (!nh.getParam("marker_detector/input_camera_info_topic", input_camera_info_topic)) input_camera_info_topic = "/sensors/downward_camera/camera_info";
	if (!nh.getParam("marker_detector/landing_marker_topic", landing_marker_topic)) landing_marker_topic = "landing_marker";
	if (!nh.getParam("marker_detector/base_footprint_frame", base_footprint_frame)) base_footprint_frame = "/base_footprint_";
	if (!nh.getParam("marker_detector/downward_camera_frame", downward_camera_frame)) downward_camera_frame = "/base_link_";
	if (!nh.getParam("marker_detector/kinect_depth_optical_frame", kinect_depth_optical_frame)) kinect_depth_optical_frame = "/kinect_depth_optical_frame";

    image_transport::ImageTransport it(nh);
    image_transport::Subscriber sub = it.subscribe(input_image_topic, 1, callback);

    camera_info_sub = nh.subscribe<sensor_msgs::CameraInfo> (input_camera_info_topic, 1, camera_info_callback);

    std::string output_topic_mrk = nh.resolveName(landing_marker_topic);
    pub_mrk = nh.advertise<geometry_msgs::Point32 > (output_topic_mrk, 5 );

    point_est = boost::shared_ptr<PointEstimator>  (new PointEstimator);
    dnw_cam = boost::shared_ptr<DownwardCamera>  (new DownwardCamera);

    target_seq_num = 0;
    ros::spin();

    //capture_image();
	//detect(argc, argv );

	//ros::spin ();

	return 0;
}

