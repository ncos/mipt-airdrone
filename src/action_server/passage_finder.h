#ifndef PASSAGEFINDER_H
#define PASSAGEFINDER_H

#include <ros/ros.h>
#include <pcl/point_cloud.h>
#include <geometry_msgs/Twist.h>
#include <boost/thread/mutex.hpp>
#include <geometry_msgs/PoseStamped.h>


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
	bool is_nan;
	double mid_ang, left_ang, rght_ang;

	Passage () : mid_ang(0), left_ang(0), rght_ang(0)
	{
		width = NAN;
		kin_middle.x = NAN;
		kin_middle.y = NAN;
		kin_left.x   = NAN;
		kin_left.y   = NAN;
		kin_rght.x   = NAN;
		kin_rght.y   = NAN;
		is_nan = true;
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
	Passage_finder(Line_param &line);
private:
	void add_passage(int id1, int id2, Line_param &line);
	void check_boundary(Line_param &line);
};



class Advanced_Passage_finder // Da ADVANCED Passage findr! (looks for holes in walls and is better than Passage findr)
{
public:
    std::vector<Passage> passages;
    void renew(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr &cloud);
    bool passage_on_line(Line_param &line, Passage &passage);
    Line_param *get_best_line(Passage &passage, Line_map &linemap);

private:
    void add_passage(double point1x, double point1y, double point2x, double point2y);
    double sqrange(pcl::PointXYZ p1, pcl::PointXYZ p2);
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
	bool tracking_on;

public:
	double ref_dist, ref_ang;
	geometry_msgs::Twist base_cmd, buf_cmd;

public:
	MotionServer (boost::shared_ptr<boost::mutex> _mutex) : mutex(_mutex),
															ref_wall(NULL),
															ref_dist(0),
															ref_ang(0)
															{}

	void set_pid_vel  (double P, double I, double D) {this->pid_vel.set_PID(P, I, D); }
	void set_pid_ang  (double P, double I, double D) {this->pid_ang.set_PID(P, I, D); }
	void set_ref_wall (Line_param *wall);
	void untrack ();
	void set_angles_current ();
	bool rotate(double angle);
	bool move_parallel(double vel);
	bool move_perpendicular(double shift);
	void spin_once();
	void clear_cmd();
	void lock() {this->mutex->lock(); }
	void unlock() {this->mutex->unlock(); }

private:
	void set_target_angle (double angle) {this->ref_ang  = angle; }
	void set_target_dist  (double dist ) {this->ref_dist = dist;  }
};


class MappingServer
{
private:
    boost::shared_ptr<boost::mutex> mutex;
    ros::Subscriber sub;

public:
    pcl::PointXY position;


public:
    MappingServer (ros::NodeHandle _nh, std::string inp_topic);

    pcl::PointXY get_positon();

    void lock() {this->mutex->lock(); }
    void unlock() {this->mutex->unlock(); }

private:
    void callback (const geometry_msgs::PoseStamped pos_msg);
};


#endif
