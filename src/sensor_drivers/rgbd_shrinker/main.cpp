#include <stdio.h>
#include <iostream>
#include <vector>
#include <math.h>
#include <algorithm>    // std::sort


#include <ros/ros.h>
#include "pcl_ros/point_cloud.h"
#include <pcl/point_types.h>
#include <pcl/common/common_headers.h>
#include <pcl/filters/passthrough.h>


using std::cout;
using std::cerr;
using std::endl;

ros::Subscriber sub;
ros::Publisher  pub_laser;

pcl::PointCloud<pcl::PointXYZ>::Ptr laser_cloud (new pcl::PointCloud<pcl::PointXYZ>);


struct Cmp_class {
  bool operator() (pcl::PointXYZ point_1, pcl::PointXYZ point_2)
  	  {
	  	  if(point_1.z == 0 || point_2.z == 0) return true;
	  	  return ((point_1.x/point_1.z) < (point_2.x/point_2.z) );
  	  }
} cmp_class;


//ros::Time start;
void callback(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& cloud)
{
    //cerr << "\tIdle_sec: " << cloud->header.stamp << "\n";
	//cerr << "\tIdle_sec: " << ros::Time::now().toSec() - start.toSec(); start = ros::Time::now();
	pcl::PassThrough<pcl::PointXYZ> pass;
	pass.setInputCloud (cloud);
	pass.setFilterFieldName ("y");      // z is to front, y is DOWN!
	pass.setFilterLimits (-0.3, -0.2);  // (-0.3, -0.2)
	pass.filter (*laser_cloud);

	for (int i = 0; i < laser_cloud->points.size(); i++)
		laser_cloud->points[i].y = 0;

	std::sort (laser_cloud->points.begin(), laser_cloud->points.end(), cmp_class);     // Sorting by angle


	pub_laser.publish(laser_cloud);
	//cerr << "\tLaser_sec: " << ros::Time::now().toSec() - start.toSec() << "\n"; start = ros::Time::now();
};



int main (int argc, char** argv)
{
	ros::init (argc, argv, "rgbd_shrinker");

	ros::NodeHandle nh;
	std::string input_topic        = nh.resolveName("/sensors/kinect/depth/points" );
	std::string output_laser_topic = nh.resolveName("/shrinker/depth/laser_points");

	sub       = nh.subscribe<pcl::PointCloud<pcl::PointXYZ> > (input_topic,  1, callback);
	pub_laser = nh.advertise<pcl::PointCloud<pcl::PointXYZ> > (output_laser_topic, 1);

	ros::spin ();
	return 0;
}



