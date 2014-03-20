#ifndef RANSAC_H
#define RANSAC_H

// C++ libraries
#include <stdio.h>
#include <iostream>
#include <vector>
#include <cmath>


// PCL libraries
#include <pcl/point_types.h>
#include <pcl/features/normal_3d.h>
#include <pcl/common/common_headers.h>

#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/surface/mls.h>

#include <pcl/ModelCoefficients.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/sample_consensus/sac_model_plane.h>





typedef pcl::PointXYZRGB  PointT;
typedef pcl::PointXYZ 	  PointS;
typedef pcl::Normal 	  Normal;
typedef pcl::PointNormal  PointN;

typedef pcl::PointCloud<PointT> PointCloudT;
typedef pcl::PointCloud<PointS> PointCloudS;
typedef pcl::PointCloud<Normal> NormalCloud;
typedef pcl::PointCloud<PointN> PointCloudN;

typedef unsigned long long int uint64;

#define PI 3.14159265





class Position
{
public:
	bool is_Nan;
	Eigen::Vector3f normal_to_floor;
	double distance_to_floor;
	struct {float roll; float pitch;} rotation;

	Position ();
	void renew (float &A, float &B, float &C, float &D, uint64 num_of_points);
};




class Floor_SAC
{
private:
	pcl::PointIndices::		Ptr inliers;
	pcl::ModelCoefficients::Ptr coefficients;
	PointCloudN::			Ptr mls_points;


	pcl::search::Search<PointS>::Ptr tree; // Search (used by other filters)
	pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointNormal> mls; // Smoothing
	pcl::SACSegmentation<PointN> seg; // Segmentation


public:
	Position position;

public:
	Floor_SAC ();

	PointCloudN::Ptr filter (PointCloudS::ConstPtr simple_cloud);
	pcl::ModelCoefficients::Ptr  find_plane (PointCloudN::Ptr mls_cloud);
	PointCloudT::Ptr get_colored_cloud (PointCloudN::Ptr mls_cloud, pcl::PointIndices::Ptr inliers);

};




















#endif
