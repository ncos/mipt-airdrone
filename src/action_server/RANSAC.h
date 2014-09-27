#ifndef RANSAC_H
#define RANSAC_H

#include <stdio.h>
#include <iostream>
#include <vector>
#include <cmath>

#include <ros/ros.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/common/angles.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>


// Message files
#include <ransac_slam/LineMap.h>


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

    void renew (pcl::PointXYZ &dfdir, pcl::PointXYZ &ldir)
    {
        this->ldir_vec.kin.x =   ldir.x;
        this->ldir_vec.kin.y =   ldir.z;

        this->ldir_vec.cmd.x =   ldir.z;
        this->ldir_vec.cmd.y = - ldir.x;

        this->fdir_vec.kin.x = - ldir.z;
        this->fdir_vec.kin.y =   ldir.x;

        this->fdir_vec.cmd.x =   ldir.x;
        this->fdir_vec.cmd.y =   ldir.z;

        this->A = this->fdir_vec.kin.x;
        this->B = this->fdir_vec.kin.y;
        this->C = -(A*dfdir.x + B*dfdir.z);

        this->on_left = (this->A ==0) ? true : (-this->C/this->A < 0);


        if (this->ldir_vec.kin.x == 0) {
            if(this->ldir_vec.kin.y < 0) this->angle = -90; // 5 = forward, 3 - to the right
            else this->angle = 90;
        }
        else this->angle = atan(this->ldir_vec.kin.y / fabs(this->ldir_vec.kin.x))*180.0/M_PI;
        if (this->angle > 0 && this->ldir_vec.kin.x < 0) this->angle += 90;
        if (this->angle < 0 && this->ldir_vec.kin.x < 0) this->angle -= 90;

        this->distance = fabs( this->C );
    };


	double distance_to_point(double x, double y)
	{
	    return fabs(this->A * x + this->B * y + this->C);
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

	Line_map () : eps(30), derr (1.0)
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
    void renew (ransac_slam::LineMap::ConstPtr lines_msg);
	Line_param *get_best_fit (double angle, double distance);
	Line_param *get_closest  (double angle);
};




#endif // RANSAC_H

