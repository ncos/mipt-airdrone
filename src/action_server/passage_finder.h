#ifndef PASSAGEFINDER_H
#define PASSAGEFINDER_H

#include <ros/ros.h>
#include <pcl/point_cloud.h>
#include <geometry_msgs/Twist.h>

#include <iostream>
#include <cstdio>
#include <vector>
#include <cmath>
#include <ctime>

#include "RANSAC.h"
#include "pid_regulator.h"


struct Passage
{
	double width;
	pcl::PointXY kin_middle;
	pcl::PointXY kin_left;
	pcl::PointXY kin_rght;
	double mid_ang, left_ang, rght_ang;

	Passage ()
	{
		width = 0.0;
		kin_middle.x = 0.0;
		kin_middle.y = 0.0;
	}
};


struct stMemory
{
	double angle;
	double distance;
};


class Passage_finder // Da Passage findr! (looks for holes in walls)
{
public:
	std::vector<Passage> passage;
	Passage_finder (Line_param &line);
private:
	void add_passage(int id1, int id2, Line_param &line);
	void check_boundary(Line_param &line);
};




class LocationServer
{
public:
	Line_map lm;
	bool lost_ref_wall;
private:
	double yaw;
	stMemory stm, ltm;
	Line_param *ref_wall, *corner_wall_left, *corner_wall_rght;

public:
	LocationServer () {lost_ref_wall = true; ref_wall = NULL; yaw = 0; stm.angle = 0; stm.distance = 0;}
	double get_yaw() 		{return this->yaw; }
	void   set_zero_yaw() 	{this->yaw = 0.0;  }
	void   track_wall			(Line_param *wall);
	Line_param  *get_ref_wall() {return this->ref_wall; }
	Line_param  *get_crn_wall_left() {return this->corner_wall_left; }
	Line_param  *get_crn_wall_rght() {return this->corner_wall_rght; }
	void   spin_once(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& cloud);
	bool   obstacle_detected_left ();
	bool   obstacle_detected_rght ();
};




class MotionServer
{
private:
	PID pid_ang;
	PID pid_vel;
	Line_param *ref_wall;
public:
	double ref_dist, ref_ang;
	geometry_msgs::Twist base_cmd;

public:
	void set_pid_vel  (double P, double I, double D) {this->pid_vel.set_PID(P, I, D); }
	void set_pid_ang  (double P, double I, double D) {this->pid_ang.set_PID(P, I, D); }
	void set_ref_wall (Line_param *wall);
	void set_angles_current ();
	bool rotate(double angle);
	bool move_parallel(double vel);
	bool move_perpendicular(double shift);
	void spin_once();

private:
	void set_target_angle (double angle) {this->ref_ang  = angle; }
	void set_target_dist  (double dist ) {this->ref_dist = dist;  }
};




#endif
