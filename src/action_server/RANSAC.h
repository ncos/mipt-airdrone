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

#define MIN_POINTS_IN_LINE 150

using std::cout;
using std::cerr;
using std::endl;





class Line_param
{
public:
	bool found;
	double angle; // with the ldir_vec
	double distance, A, B, C;
	bool on_left;
	int quality; // How good we can see this line
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

	    this->on_left = (this->A ==0) ? true : (-this->C/this->A < 0);



	    if (this->ldir_vec.kin.x == 0) {
	    	if(this->ldir_vec.kin.y < 0) this->angle = -90; // 5 = forward, 3 - to the right
	    	else this->angle = 90;
	    }
	    else this->angle = atan(this->ldir_vec.kin.y / fabs(this->ldir_vec.kin.x))*180.0/M_PI;
	    if (this->angle > 0 && this->ldir_vec.kin.x < 0) this->angle += 90;
	    if (this->angle < 0 && this->ldir_vec.kin.x < 0) this->angle -= 90;

	    this->distance = fabs( this->C );

	    for (int i = 0; i < inliers_idx->indices.size(); ++i)
	    {
	    	pcl::PointXY point;
	    	point.x = cloud->points.at(inliers_idx->indices.at(i)).x;
	    	point.y = cloud->points.at(inliers_idx->indices.at(i)).z;
	    	this->kin_inliers.push_back(point);
	    }

	    this->quality = this->kin_inliers.size();
	};

	double distance_to_point(double x, double y)
	{
	    return fabs(this->A * x + this->B * y + this->C);
	};

private:
	void normalize(pcl::ModelCoefficients::Ptr coefficients)
	{
		// line direction coordinates in kinect frame:
		double lx = coefficients->values[3]; // x is to the right
		double ly = coefficients->values[5]; // y is forward !FOR KINECT!

		// point on the line in kinect frame:
		double rx = coefficients->values[0]; // x is to the right
		double ry = coefficients->values[2]; // y is forward !FOR KINECT!

		if (rx*ly - ry*lx > 0)
		{
			lx = - lx;
			ly = - ly;
		}

		coefficients->values[3] = lx;
		coefficients->values[5] = ly;
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




#endif // RANSAC_H

