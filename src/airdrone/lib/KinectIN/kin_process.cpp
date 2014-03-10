
#include <kin.h>


#include <pcl/search/search.h>
#include <pcl/search/kdtree.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/filter.h>
#include <pcl/sample_consensus/sac_model_plane.h>




KinectIN::KinectIN ()
{
	this->inliers 			= pcl::PointIndices::Ptr 		(new pcl::PointIndices);
    this->coefficients 		= pcl::ModelCoefficients::Ptr 	(new pcl::ModelCoefficients);
    this->mls_points  		= PointCloudN::Ptr 				(new PointCloudN);

    this->tree = boost::shared_ptr<pcl::search::Search<PointS> > (new pcl::search::KdTree<PointS>);

}


PointCloudN::Ptr KinectIN::filter (PointCloudS::ConstPtr floor_cloud)
{
	this->mls.setComputeNormals (false);
	this->mls.setInputCloud (floor_cloud);
	this->mls.setPolynomialFit (true);
	this->mls.setSearchMethod (tree);
	this->mls.setSearchRadius (0.2);
	this->mls.process (*mls_points);

	return this->mls_points;
}


pcl::ModelCoefficients::Ptr KinectIN::find_plane (PointCloudN::Ptr filtered_cloud)
{
	seg.setOptimizeCoefficients (true);
	//seg.setModelType (pcl::SACMODEL_PLANE);
	seg.setModelType (pcl::SACMODEL_PERPENDICULAR_PLANE);

	seg.setMethodType (pcl::SAC_PROSAC);
	seg.setDistanceThreshold (0.005);
	seg.setEpsAngle	(pcl::deg2rad (30.0));

	// vector with three components: look for the (left/right wall; floor; front wall)
	seg.setAxis(this->position.normal_to_floor);
	seg.setMaxIterations(50);
	//seg.setProbability(100.0);

	seg.setInputCloud (filtered_cloud);

	seg.segment (*inliers, *coefficients);
	//while(inliers->indices.size () < 400)
		//seg.segment (*inliers, *coefficients);


	this->position.renew(	coefficients->values[0],
							coefficients->values[1],
							coefficients->values[2],
							coefficients->values[3],
							inliers->indices.size ()	);

	//std::cerr << "Model inliers: " << inliers->indices.size () << std::endl;

	return coefficients;
}



/*
SAC_RANSAC - RANdom SAmple Consensus
SAC_LMEDS - Least Median of Squares
SAC_MSAC - M-Estimator SAmple Consensus
SAC_RRANSAC - Randomized RANSAC
SAC_RMSAC - Randomized MSAC
SAC_MLESAC - Maximum LikeLihood Estimation SAmple Consensus
SAC_PROSAC - PROgressive SAmple Consensus
*/
