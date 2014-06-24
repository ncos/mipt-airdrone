#include <ros/ros.h>
#include "detector.h"





int main (int argc, char** argv)
{
	ros::init (argc, argv, "marker_detector");

	ros::NodeHandle nh;



	haar(argc, argv );

	//ros::spin ();

	return 0;
}

