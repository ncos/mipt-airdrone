#include "RANSAC.h"



void Line_map::renew (pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud)
{
	bool lines_cleared = false;
	(*cloud_f).clear();
	(*cloud_f) += *cloud;

	int nr_points = (int) cloud_f->points.size ();

	while (cloud_f->points.size () > 0.06 * nr_points)
	{
		// Segment the largest planar component from the remaining cloud
	    seg.setInputCloud (cloud_f);
	    seg.segment (*inliers, *coefficients);
	    if (inliers->indices.size () == 0) break;
	    else if (!lines_cleared) {lines.clear(); lines_cleared = true; }

	    Line_param lp;
	    lp.found = true;
	    lp.renew(coefficients, cloud, inliers);

	    lines.push_back(lp);

	    // Create the filtering object
	    extract.setInputCloud (cloud_f);
	    extract.setIndices (inliers);
	    extract.setNegative (true);
	    extract.filter (*cloud_f);
	}
};



Line_param *Line_map::get_best_fit (double angle, double distance)
{
	int id = 0;
	double error = 10000;
	for (int i = 0; i < lines.size(); i++)
	{
		double newerror = 	fabs(lines.at(i).angle - angle) +
							fabs(lines.at(i).distance - distance);
		if (newerror < error) { id = i; error = newerror; }
	}
	if (fabs(lines.at(id).angle    - angle   ) > eps ) lines.at(id).found = false;
	if (fabs(lines.at(id).distance - distance) > derr) lines.at(id).found = false;
	return &lines.at(id);
};



Line_param *Line_map::get_closest (double angle)
{
	int id = 0;
	double distance = 10000;
	for (int i = 0; i < lines.size(); i++)
	{
		if (fabs(lines.at(i).angle - angle) < eps)
		{
			double newdistance = 	lines.at(i).distance;
			if (newdistance < distance) { id = i; distance = newdistance; }
		}
	}
	if (fabs(lines.at(id).angle    - angle   ) > eps ) lines.at(id).found = false;
	return &lines.at(id);
};



