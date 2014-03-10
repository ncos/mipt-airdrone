#ifndef PCV_H
#define PCV_H

// C++ libraries
#include <stdio.h>
#include <iostream>
#include <vector>
#include <cassert>


// PCL libraries
#include <pcl/point_types.h>
#include <boost/thread/thread.hpp>
#include <pcl/common/common_headers.h>

#include <pcl/search/search.h>
#include <pcl/search/kdtree.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/filter.h>
#include <pcl/sample_consensus/sac_model_plane.h>

#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>

#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/surface/mls.h>

// Local libraries
#include <position.h>

typedef pcl::PointXYZRGB  PointT;
typedef pcl::PointXYZ 	  PointS;
typedef pcl::Normal 	  Normal;
typedef pcl::PointNormal  PointN;

typedef pcl::PointCloud<PointT> PointCloudT;
typedef pcl::PointCloud<PointS> PointCloudS;
typedef pcl::PointCloud<Normal> NormalCloud;
typedef pcl::PointCloud<PointN> PointCloudN;


using std::cout;
using std::cerr;
using std::endl;







class KinectIN
{
private:
	// Local variables
	//PointCloudS::			Ptr downsampled_cloud;
	//PointCloudT::			Ptr segmented_cloud;
	pcl::PointIndices::		Ptr inliers;
	pcl::ModelCoefficients::Ptr coefficients;
	PointCloudN::			Ptr mls_points;


	// Filter objects
	pcl::search::Search<PointS>::Ptr tree; // Search (used by other filters)
	pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointNormal> mls; // Smoothing
	pcl::SACSegmentation<PointN> seg; // Segmentation


public:
	// Other objects
	//Splitter split_cloud;
	Position position;

public:
	KinectIN ();
	~KinectIN () {}; // Safely exit) //

	PointCloudN::Ptr filter (PointCloudS::ConstPtr simple_cloud);
	pcl::ModelCoefficients::Ptr  find_plane (PointCloudN::Ptr mls_cloud);
	PointCloudT::Ptr get_colored_cloud (PointCloudN::Ptr mls_cloud, pcl::PointIndices::Ptr inliers);

};




















#endif
