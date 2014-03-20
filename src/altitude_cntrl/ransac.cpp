
#include "ransac.h"




Floor_SAC::Floor_SAC ()
{
	this->inliers 			= pcl::PointIndices::Ptr 		(new pcl::PointIndices);
    this->coefficients 		= pcl::ModelCoefficients::Ptr 	(new pcl::ModelCoefficients);
    this->mls_points  		= PointCloudN::Ptr 				(new PointCloudN);

    this->tree = boost::shared_ptr<pcl::search::Search<PointS> > (new pcl::search::KdTree<PointS>);
};


PointCloudN::Ptr Floor_SAC::filter (PointCloudS::ConstPtr floor_cloud)
{
	this->mls.setComputeNormals (false);
	this->mls.setInputCloud (floor_cloud);
	this->mls.setPolynomialFit (true);
	this->mls.setSearchMethod (tree);
	this->mls.setSearchRadius (0.2);
	this->mls.process (*mls_points);

	return this->mls_points;
};


pcl::ModelCoefficients::Ptr Floor_SAC::find_plane (PointCloudN::Ptr filtered_cloud)
{
	seg.setOptimizeCoefficients (true);
	seg.setModelType (pcl::SACMODEL_PERPENDICULAR_PLANE);
	seg.setMethodType (pcl::SAC_PROSAC);
	seg.setDistanceThreshold (0.005);
	seg.setEpsAngle	(pcl::deg2rad (30.0));
	seg.setAxis(this->position.normal_to_floor);
	seg.setMaxIterations(50);
	seg.setInputCloud (filtered_cloud);
	seg.segment (*inliers, *coefficients);

	this->position.renew(	coefficients->values[0],
							coefficients->values[1],
							coefficients->values[2],
							coefficients->values[3],
							inliers->indices.size ()	);
	return this->coefficients;
};


Position::Position ()
{
	this->is_Nan = true;
	this->distance_to_floor  = 0;
	this->normal_to_floor(0) = 0;  // Component to the front
	this->normal_to_floor(1) = 1;  // Axis up
	this->normal_to_floor(2) = 0;  // Axis right
};


void Position::renew (float &A, float &B, float &C, float &D, uint64 num_of_points)
{
	if (num_of_points < 200) {this->is_Nan = true; return;}
	this->is_Nan = false;

	this->normal_to_floor(0) = A;  // Component to the front
	this->normal_to_floor(1) = B;  // Axis up
	this->normal_to_floor(2) = C;  // Axis right

	// (0, 1, 0) - vector normal to floor
	double roll_cos  = B/sqrt(B*B + C*C);
	double pitch_cos = B/sqrt(B*B + A*A);

	this->rotation.pitch = (PI-acos(pitch_cos)) * 180.0/PI;
	this->rotation.roll  = (PI-acos(roll_cos )) * 180.0/PI;

	this->distance_to_floor = D;
};










