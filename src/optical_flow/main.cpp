#include <ros/ros.h>
#include "flow.h"





int main (int argc, char** argv)
{
	ros::init (argc, argv, "optical_flow_node");

	ros::NodeHandle nh;



	flow(argc, argv);

	//ros::spin ();

	return 0;
}

