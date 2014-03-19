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
	if(lp == NULL) return;

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





bool explore_new_wall()
{
	Passage_finder pf(*loc_srv.get_ref_wall());
	for (int i = 0; i < pf.passage.size(); i++)
		draw_point(pf.passage.at(i).kin_middle.x, pf.passage.at(i).kin_middle.y, i);

	return !(pf.passage.size() == 0);
};


void move_along(double vel, double angl, double dist)
{
	msn_srv.ref_ang  = angl;
	msn_srv.ref_dist = dist;
	msn_srv.move_parallel(vel);
};


bool unexplored_wall = true;
bool left_wall_with_door = false;
bool in_front_of_passage = false;
bool ready_to_enter = false;


void callback(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& cloud)
{
    line_list.header.stamp = ros::Time::now();
    line_list.points.clear();
    loc_srv.spin_once(cloud);

    msn_srv.set_ref_wall(loc_srv.get_ref_wall());



    if (unexplored_wall) {
    	msn_srv.set_angles_current();
    	msn_srv.ref_dist -= 0.3;

    	if (explore_new_wall() == true) {
    		left_wall_with_door = true;
        	unexplored_wall = false;
    	}

    	if (!left_wall_with_door)
    		msn_srv.ref_ang = target_angl;

        if(fabs(loc_srv.get_ref_wall()->angle - msn_srv.ref_ang) < 10 ) {
        	unexplored_wall = false;
        }
    }

    if (left_wall_with_door && !in_front_of_passage) {
    	Passage_finder pf(*loc_srv.get_ref_wall());
    	for (int i = 0; i < pf.passage.size(); i++) {
    		draw_point(pf.passage.at(i).kin_middle.x, pf.passage.at(i).kin_middle.y, i*3 + 0);
    		draw_point(pf.passage.at(i).kin_left.x,   pf.passage.at(i).kin_left.y,   i*3 + 1);
    		draw_point(pf.passage.at(i).kin_rght.x,   pf.passage.at(i).kin_rght.y,   i*3 + 2);

    	}

    	if (pf.passage.size() == 0) ROS_WARN("No passage here!");
    	else {
    		msn_srv.ref_ang = loc_srv.get_ref_wall()->angle + pf.passage.at(0).rght_ang + 15;

    		double err_shift = loc_srv.get_ref_wall()->ldir_vec.kin.x * pf.passage.at(0).kin_middle.x +
    						   loc_srv.get_ref_wall()->ldir_vec.kin.y * pf.passage.at(0).kin_middle.y;

    		msn_srv.move_parallel(-vel_P * movement_speed * err_shift);
    		msn_srv.ref_dist = 1.2;
    		if (fabs(err_shift) < 0.1) {
    			in_front_of_passage = true;
    			ready_to_enter = true;
    			msn_srv.ref_ang = 55;
    		}
    	}
    }


    if(in_front_of_passage)
    {
        if(fabs(loc_srv.get_ref_wall()->angle - msn_srv.ref_ang) < 5 ) {
        	if(!loc_srv.obstacle_detected_rght() && ready_to_enter) ROS_ERROR("CANT SEE WALL ON THE RIGHT");
        }


    	if( loc_srv.obstacle_detected_rght() && ready_to_enter) {
    		ROS_INFO("in_front_of_passage");
    		loc_srv.track_wall(loc_srv.get_crn_wall_rght());
    	    msn_srv.set_ref_wall(loc_srv.get_ref_wall());
    		msn_srv.ref_dist = loc_srv.get_ref_wall()->distance;
    		msn_srv.ref_ang = 20;
    		ready_to_enter = false;
    	}


    	if(!ready_to_enter) {
			move_along(movement_speed, 20, msn_srv.ref_dist);


			if(loc_srv.obstacle_detected_rght()) {
				ROS_ERROR("ON THE RIGHT");

				left_wall_with_door = false;
				in_front_of_passage = false;
				unexplored_wall = false;
			}
    	}
    }



    if (!unexplored_wall && !left_wall_with_door) {
        if(loc_srv.obstacle_detected_left()) {
        	loc_srv.track_wall(loc_srv.get_crn_wall_left());
        	unexplored_wall = true;
        }
        msn_srv.ref_ang = target_angl;

        if(fabs(loc_srv.get_ref_wall()->angle - msn_srv.ref_ang) < 10 ) {
        	move_along(movement_speed, target_angl, target_dist);
        }
    }



    if(loc_srv.obstacle_detected_left()) ROS_INFO("Wall on the left");
    if(loc_srv.obstacle_detected_rght()) ROS_INFO("Wall on the right");



    add_l(loc_srv.get_ref_wall());
    add_l(loc_srv.get_crn_wall_left());
    add_l(loc_srv.get_crn_wall_rght());



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


  msn_srv.set_pid_ang(ang_P, ang_I, ang_D);
  msn_srv.set_pid_vel(vel_P, vel_I, vel_D);

  //ROS_INFO("ROS_INFO");
  //ROS_WARN("ROS_WARN");
  //ROS_ERROR("ROS_ERROR");
  ros::spin ();

  return 0;
}
