#ifndef RANSAC_H
#define RANSAC_H

#include <stdio.h>
#include <iostream>
#include <vector>
#include <cmath>

#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/common/angles.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>


#define PI 3.14159265

using std::cout;
using std::cerr;
using std::endl;





class PID
{
	double P, I, D;
	double prev_err;
	bool not_started;
	double t_diff;
	double integral;

public:
	PID(double _P = 0.0, double _I = 0.0, double _D = 0.0)
	{
		P = _P; I = _I; D = _D;
		not_started = true;
		prev_err = 0;
		integral = 0;
		t_diff = 0.0333333; // 1/30 sec
	}

	void set_PID(double _P, double _I, double _D)
	{
		P = _P; I = _I; D = _D;
	}

	double get_output(double target_val, double current_val)
	{
		double err = target_val - current_val;

		if(not_started)
		{
			not_started = false;
			prev_err = err;
			return P * err;
		}
		integral += err;

		double P_val = P * err;
		double I_val = I * integral * t_diff;
		double D_val = D * (err - prev_err) / t_diff;

		prev_err = err;
		return P_val + I_val + D_val;
	}
};


class Line_param
{
public:
	bool found;
	double angle; // with the ldir_vec
	double distance, A, B, C;
	bool on_left;
	std::vector<pcl::PointXY> kin_inliers;
	struct LDirection
	{
		struct Kin
		{
			double x, y;
		} kin;

		struct Cmd
		{
			double x, y;
		} cmd;

	} ldir_vec; // direction to the left across the line

	struct FDirection
	{
		struct Kin
		{
			double x, y;
		} kin;

		struct Cmd
		{
			double x, y;
		} cmd;

	} fdir_vec; // points to the line


	Line_param ()
	{
		found = false;

	    this->ldir_vec.kin.x = 0;
	    this->ldir_vec.kin.y = 0;

	    this->ldir_vec.cmd.x = 0;
	    this->ldir_vec.cmd.y = 0;

	    this->fdir_vec.kin.x = 0;
	    this->fdir_vec.kin.y = 0;

	    this->fdir_vec.cmd.x = 0;
	    this->fdir_vec.cmd.y = 0;
	}


	void renew (pcl::ModelCoefficients::Ptr coefficients,
			    pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud,
			    pcl::PointIndices::Ptr inliers_idx)
	{
		kin_inliers.clear();
	    normalize(coefficients);
	    this->ldir_vec.kin.x =   coefficients->values[3];
	    this->ldir_vec.kin.y =   coefficients->values[5];

	    this->ldir_vec.cmd.x =   coefficients->values[5];
	    this->ldir_vec.cmd.y = - coefficients->values[3];

	    this->fdir_vec.kin.x = - coefficients->values[5];
	    this->fdir_vec.kin.y =   coefficients->values[3];

	    this->fdir_vec.cmd.x =   coefficients->values[3];
	    this->fdir_vec.cmd.y =   coefficients->values[5];

	    this->A = this->fdir_vec.kin.x;
	    this->B = this->fdir_vec.kin.y;
	    this->C = -(A*coefficients->values[0] + B*coefficients->values[2]);

	    this->on_left = (coefficients->values[0] < 0) ? true : false;

	    if (this->ldir_vec.kin.x == 0)
	    {
	    	if(this->ldir_vec.kin.y < 0) this->angle = -90; // 5 = forward, 3 - to the right
	    	else this->angle = 90;
	    }
	    else this->angle = atan(this->ldir_vec.kin.y / this->ldir_vec.kin.x)*180.0/PI;

	    this->distance = fabs( coefficients->values[2]*coefficients->values[3]
	                         - coefficients->values[0]*coefficients->values[5] );

	    for (int i = 0; i < inliers_idx->indices.size(); i++)
	    {
	    	pcl::PointXY point;
	    	point.x = cloud->points.at(inliers_idx->indices.at(i)).x;
	    	point.y = cloud->points.at(inliers_idx->indices.at(i)).z;
	    	this->kin_inliers.push_back(point);
	    }
	};


	void normalize(pcl::ModelCoefficients::Ptr coefficients)
	{
		double x_axis = coefficients->values[3]; // x is to the right
		double y_axis = coefficients->values[5]; // y is forward !FOR KINECT!

		if (x_axis < 0)
		{
			x_axis = - x_axis;
			y_axis = - y_axis;
		}

		coefficients->values[3] = x_axis;
		coefficients->values[5] = y_axis;
	};



};


class Line_map
{
private:
	pcl::ExtractIndices<pcl::PointXYZ> extract;
	pcl::SACSegmentation<pcl::PointXYZ> seg;
	pcl::ModelCoefficients::Ptr coefficients;
	pcl::PointIndices::Ptr inliers;
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_f;

public:
	std::vector<Line_param> lines;
	double eps, derr;

	Line_map () : eps(20), derr (0.5)
	{
		this->coefficients = pcl::ModelCoefficients::Ptr (new pcl::ModelCoefficients ());
		this->inliers 	   = pcl::PointIndices::Ptr 	 (new pcl::PointIndices ());
		// Optional

		this->seg.setOptimizeCoefficients (true);
		this->seg.setModelType (pcl::SACMODEL_LINE);
		this->seg.setMethodType (pcl::SAC_RANSAC);
		this->seg.setDistanceThreshold (0.003);

		this->cloud_f = pcl::PointCloud<pcl::PointXYZ>::Ptr (new pcl::PointCloud<pcl::PointXYZ>);
	}

	void renew (pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud);
	Line_param *get_best_fit (double angle, double distance);
	Line_param *get_closest  (double angle);
};




#endif

