#include <cstdio>
#include <iostream>
#include <vector>
#include <cmath>
#include <ctime>

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <visualization_msgs/Marker.h>

#include "pcl_ros/point_cloud.h"
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/common/common_headers.h>
#include <pcl/ModelCoefficients.h>

#include "passage_finder.h"

using std::cout;
using std::cerr;
using std::endl;

ros::Subscriber sub;
ros::Publisher  pub_mrk;
ros::Publisher  pub_vel;

double vel_P = 0.0, vel_I = 0.0, vel_D = 0.0;
double ang_P = 0.0, ang_I = 0.0, ang_D = 0.0;

double target_dist = 0.0;
double target_angl = 0.0;
double movement_speed = 0.0;

visualization_msgs::Marker line_list;
LocationServer loc_srv;
MotionServer   msn_srv;


// Draw in kinect coordinates
void draw_point(double x, double y, int id)
{
	    visualization_msgs::Marker marker;

	    marker.header.frame_id = "/camera_link";
	    marker.ns = "basic_shapes";
	    marker.action = visualization_msgs::Marker::ADD;
	    marker.id = id;
	    marker.type = visualization_msgs::Marker::SPHERE;

	    marker.pose.position.x =  y;
	    marker.pose.position.y = -x;
	    marker.pose.position.z = 0;

	    // Set the scale of the marker -- 1x1x1 here means 1m on a side
	    marker.scale.x = 0.1;
	    marker.scale.y = 0.1;
	    marker.scale.z = 0.1;

	    // Set the color -- be sure to set alpha to something non-zero!
	    marker.color.r = 1.0f;
	    marker.color.g = 0.0f;
	    marker.color.b = 0.0f;
	    marker.color.a = 0.6;

	    marker.lifetime = ros::Duration(0.2);

	    marker.header.stamp = ros::Time::now();
	    // Publish the marker
	    pub_mrk.publish(marker);
};


void add_e(double x, double y)
{
    geometry_msgs::Point p;
    p.y = 0;
    p.z = 0;
    p.x = 0;

    line_list.points.push_back(p);

    p.y = -x;
    p.z =  0;
    p.x =  y;

    line_list.points.push_back(p);
};


void add_l(Line_param *lp)
{
	const int length = 40;
    geometry_msgs::Point p;
    p.y = -lp->fdir_vec.kin.x * lp->distance + lp->ldir_vec.kin.x * length / 2;
    p.z =  0;
    p.x =  lp->fdir_vec.kin.y * lp->distance - lp->ldir_vec.kin.y * length / 2;
    line_list.points.push_back(p);
    p.y =  p.y - lp->ldir_vec.kin.x * length;
    p.z =  0;
    p.x =  p.x + lp->ldir_vec.kin.y * length;
    line_list.points.push_back(p);
};




void callback(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& cloud)
{
    line_list.header.stamp = ros::Time::now();
    line_list.points.clear();
    loc_srv.spin_once(cloud);

    if(loc_srv.get_ref_wall() == NULL)
    	loc_srv.track_wall(loc_srv.lm.get_best_fit(0, 2));


    add_l(loc_srv.get_ref_wall());


	if (!loc_srv.obstacle_detected())
	{

	}
	else
	{
		/*
		add_l(lp_corner);
		Passage_finder pf(lp_corner);
		for (int i = 0; i < pf.passage.size(); i++)
			draw_point(pf.passage.at(i).kin_middle.x, pf.passage.at(i).kin_middle.y, i);
		*/
	}



	msn_srv.spin_once();
	pub_vel.publish(msn_srv.base_cmd);
    pub_mrk.publish(line_list);

};



int main( int argc, char** argv )
{
  ros::init(argc, argv, "laserscan_server");
  ros::NodeHandle nh;


  if (!nh.getParam("PID_ang_P", ang_P)) ROS_ERROR("Failed to get param 'PID_ang_P'");
  if (!nh.getParam("PID_ang_I", ang_I)) ROS_ERROR("Failed to get param 'PID_ang_I'");
  if (!nh.getParam("PID_ang_D", ang_D)) ROS_ERROR("Failed to get param 'PID_ang_D'");


  if (!nh.getParam("PID_vel_P", vel_P)) ROS_ERROR("Failed to get param 'PID_vel_P'");
  if (!nh.getParam("PID_vel_I", vel_I)) ROS_ERROR("Failed to get param 'PID_vel_I'");
  if (!nh.getParam("PID_vel_D", vel_D)) ROS_ERROR("Failed to get param 'PID_vel_D'");


  if (!nh.getParam("distance_to_wall", target_dist)) ROS_ERROR("Failed to get param 'distance_to_wall'");
  if (!nh.getParam("angle_to_wall", target_angl)) ROS_ERROR("Failed to get param 'angle_to_wall'");
  if (!nh.getParam("movement_speed", movement_speed)) ROS_ERROR("Failed to get param 'movement_speed'");



  std::string input_topic      = nh.resolveName("/shrinker/depth/laser_points");
  std::string output_topic_mrk = nh.resolveName("visualization_marker");
  std::string output_topic_vel = nh.resolveName("/cmd_vel_2");

  sub     = nh.subscribe<pcl::PointCloud<pcl::PointXYZ> > (input_topic,  1, callback);
  pub_mrk = nh.advertise<visualization_msgs::Marker >     (output_topic_mrk, 5 );
  pub_vel = nh.advertise<geometry_msgs::Twist >           (output_topic_vel, 1 );

  line_list.header.frame_id = "/camera_link";
  line_list.ns = "lines_ns";
  line_list.action = visualization_msgs::Marker::ADD;
  line_list.id = 0;
  line_list.type = visualization_msgs::Marker::LINE_LIST;
  line_list.scale.x = 0.03;
  line_list.color.r = 0.0;
  line_list.color.g = 0.0;
  line_list.color.b = 1.0;
  line_list.color.a = 0.3;
  line_list.lifetime = ros::Duration(0.2);


  ros::spin ();

  return 0;
}
