#ifndef PASSAGEFINDER_H
#define PASSAGEFINDER_H

#include <pcl/point_cloud.h>
#include <geometry_msgs/Twist.h>

#include <iostream>
#include <cstdio>
#include <vector>
#include <cmath>
#include <ctime>

#include "RANSAC.h"



struct Passage
{
	double width;
	pcl::PointXY kin_middle;

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
private:
	double yaw;
	stMemory stm, ltm;
	Line_param *ref_wall;
	bool lost_ref_wall;
public:
	LocationServer () {lost_ref_wall = true; ref_wall = NULL; yaw = 0;}
	double get_yaw() 		{return this->yaw; }
	void   set_zero_yaw() 	{this->yaw = 0.0;  }
	void   track_wall			(Line_param *wall);
	Line_param  *get_ref_wall() {return lost_ref_wall ? NULL : this->ref_wall; }
	void   spin_once(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& cloud);
	bool   obstacle_detected ();
};




class MotionServer
{
private:
	PID pid_ang;
	PID pid_vel;
	Line_param *ref_wall;
	double ref_dist, ref_ang;
public:
	geometry_msgs::Twist base_cmd;

public:
	void set_pid_vel  (double P, double I, double D) {this->pid_vel.set_PID(P, I, D); }
	void set_pid_ang  (double P, double I, double D) {this->pid_ang.set_PID(P, I, D); }
	void set_ref_wall (Line_param *wall) {this->ref_wall = wall;  }
	void set_target_angle (double angle) {this->ref_ang  = angle; }
	void set_target_dist  (double dist ) {this->ref_dist = dist;  }
	bool rotate(double angle);
	bool move_parallel(double vel);
	bool move_perpendicular(double shift);
	void spin_once();
};




#endif
