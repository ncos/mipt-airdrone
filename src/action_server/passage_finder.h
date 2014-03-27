#ifndef PASSAGEFINDER_H
#define PASSAGEFINDER_H

#include <ros/ros.h>
#include <pcl/point_cloud.h>
#include <geometry_msgs/Twist.h>
#include <boost/thread/mutex.hpp>


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
	stMemory () : angle(0),
				  distance(0)
	{}
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
	stMemory stm;
	Line_param *ref_wall, *corner_wall_left, *corner_wall_rght;
	boost::shared_ptr<boost::mutex> mutex;

public:
	LocationServer (boost::shared_ptr<boost::mutex> _mutex) : mutex(_mutex),
															  corner_wall_left(NULL),
															  corner_wall_rght(NULL),
															  lost_ref_wall(true),
															  ref_wall(NULL),
															  yaw(0)
															  {};

	double get_yaw() 		{return this->yaw; }
	void   set_zero_yaw() 	{this->yaw = 0.0;  }
	void   track_wall			(Line_param *wall);
	Line_param  *get_ref_wall() {return this->ref_wall; }
	Line_param  *get_crn_wall_left() {return this->corner_wall_left; }
	Line_param  *get_crn_wall_rght() {return this->corner_wall_rght; }
	void   spin_once(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& cloud);
	bool   obstacle_detected_left ();
	bool   obstacle_detected_rght ();
	void   lock() {this->mutex->lock(); }
	void   unlock() {this->mutex->unlock(); }
};




class MotionServer
{
private:
	PID pid_ang;
	PID pid_vel;
	Line_param *ref_wall;
	boost::shared_ptr<boost::mutex> mutex;

public:
	double ref_dist, ref_ang;
	geometry_msgs::Twist base_cmd;

public:
	MotionServer (boost::shared_ptr<boost::mutex> _mutex) : mutex(_mutex),
															ref_wall(NULL),
															ref_dist(0),
															ref_ang(0)
															{}

	void set_pid_vel  (double P, double I, double D) {this->pid_vel.set_PID(P, I, D); }
	void set_pid_ang  (double P, double I, double D) {this->pid_ang.set_PID(P, I, D); }
	void set_ref_wall (Line_param *wall);
	void set_angles_current ();
	bool rotate(double angle);
	bool move_parallel(double vel);
	bool move_perpendicular(double shift);
	void spin_once();
	void lock() {this->mutex->lock(); }
	void unlock() {this->mutex->unlock(); }

private:
	void set_target_angle (double angle) {this->ref_ang  = angle; }
	void set_target_dist  (double dist ) {this->ref_dist = dist;  }
};




#endif
