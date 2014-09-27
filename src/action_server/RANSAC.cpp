#include "RANSAC.h"




void Line_map::renew (ransac_slam::LineMap::ConstPtr lines_msg)
{
    this->lines.clear();
    for (int i = 0; i < lines_msg->number; ++i) {
        pcl::PointXYZ dfdir = pcl::PointXYZ(lines_msg->dfdirs.at(i).x, lines_msg->dfdirs.at(i).y, lines_msg->dfdirs.at(i).z);
        pcl::PointXYZ ldir  = pcl::PointXYZ(lines_msg->ldirs.at(i).x,  lines_msg->ldirs.at(i).y,  lines_msg->ldirs.at(i).z);

        Line_param lp;
        lp.found = true;
        lp.renew(dfdir, ldir);
        this->lines.push_back(lp);
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
	if (fabs(lines.at(id).angle    - angle   ) > eps ) return NULL;
	return &lines.at(id);
};



