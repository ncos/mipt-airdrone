#include <ros/ros.h>
#include "detector.h"

int main (int argc, char** argv)
{
	ros::init (argc, argv, "marker_detector");

	ros::NodeHandle nh;


    image_transport::ImageTransport it(nh);
    image_transport::Subscriber sub = it.subscribe("/flow_camera/flow_camera/image", 1, callback);

    cvNamedWindow("view");

    ros::spin();

    //capture_image();
	//detect(argc, argv );

	//ros::spin ();

	return 0;
}

