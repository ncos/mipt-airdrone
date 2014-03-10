#ifndef PASSAGEFINDER_H
#define PASSAGEFINDER_H

#include <pcl/point_cloud.h>


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
private:
	double yaw;
	Line_map lm;
	stMemory stm, ltm;
	Line_param *ref_wall;
public:
	LocationServer();
	double get_yaw() 		{return this->yaw; }
	void   set_zero_yaw() 	{this->yaw = 0.0;  }
	void   set_ref_wall			(Line_param &lp_stick);
	void   set_ref_wall_best_fit(double angle, double distance);
	Line_param  *get_ref_wall() {return this->ref_wall;  }
	void   refresh();
	~LocationServer();
};


class MotionServer
{
private:
	PID pid_ang;
	PID pid_vel;

public:
	MotionServer();
	void set_pid_vel(double P, double I, double D) {this->pid_vel.set_PID(P, I, D); }
	void set_pid_ang(double P, double I, double D) {this->pid_vel.set_PID(P, I, D); }
	bool rotate(double angle);



};




#endif
