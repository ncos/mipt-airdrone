#ifndef POSITION_H
#define POSITION_H


#include <stdio.h>
#include <iostream>
#include <vector>
#include <cassert>

#include <pcl/point_types.h>
#include <pcl/common/common_headers.h>



typedef pcl::PointXYZRGB  PointT;
typedef pcl::PointXYZ 	  PointS;
typedef pcl::Normal 	  Normal;
typedef pcl::PointNormal  PointN;

typedef pcl::PointCloud<PointT> PointCloudT;
typedef pcl::PointCloud<PointS> PointCloudS;
typedef pcl::PointCloud<Normal> NormalCloud;
typedef pcl::PointCloud<PointN> PointCloudN;

typedef unsigned long long int uint64;

using std::cout;
using std::cerr;
using std::endl;




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


#endif
