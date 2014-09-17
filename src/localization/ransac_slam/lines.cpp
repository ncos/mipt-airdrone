#include "lines.h"


// *****************************************
//              Line param
// *****************************************
Line_param::Line_param () {
    this->found = false;
    this->ldir_vec = pcl::PointXYZ(NAN, NAN, NAN);
    this->fdir_vec = pcl::PointXYZ(NAN, NAN, NAN);
    this->r_vec    = pcl::PointXYZ(NAN, NAN, NAN);
    this->found    = false;
    this->angle    = NAN;
    this->distance = NAN;
    this->quality  = NAN; // How good we can see this line
    this->frame    = "";
};


void Line_param::renew (pcl::ModelCoefficients::Ptr coefficients,
                        pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud,
                        pcl::PointIndices::Ptr inliers_idx)
{
    kin_inliers.clear();
    normalize(coefficients);
    this->frame = cloud->header.frame_id;
    pcl::PointXYZ r = pcl::PointXYZ(coefficients->values[0], coefficients->values[1], coefficients->values[2]);
    pcl::PointXYZ l = pcl::PointXYZ(coefficients->values[3], coefficients->values[4], coefficients->values[5]);
    pcl::PointXYZ f;
    // f = r - l(r, l)
    l = VectorMath::to_e(l);
    f.x = r.x - l.x * VectorMath::dot(r, l);
    f.y = r.y - l.y * VectorMath::dot(r, l);
    f.z = r.z - l.z * VectorMath::dot(r, l);
    f = VectorMath::to_e(f);

    this->r_vec    = r;
    this->ldir_vec = l;
    this->fdir_vec = f;

    // TODO: this implementation only works for flat lines
    if (this->ldir_vec.x == 0) {
        if(this->ldir_vec.z < 0) this->angle = -90; // 5 = forward, 3 - to the right
        else this->angle = 90;
    }
    else this->angle = atan(this->ldir_vec.z / fabs(this->ldir_vec.x))*180.0/M_PI;
    if (this->angle > 0 && this->ldir_vec.x < 0) this->angle += 90;
    if (this->angle < 0 && this->ldir_vec.x < 0) this->angle -= 90;

    this->distance = this->distance_to_point(pcl::PointXYZ(0, 0, 0));

    for (int i = 0; i < inliers_idx->indices.size(); ++i) {
        pcl::PointXYZ point;
        point.x = cloud->points.at(inliers_idx->indices.at(i)).x;
        point.y = cloud->points.at(inliers_idx->indices.at(i)).y;
        point.z = cloud->points.at(inliers_idx->indices.at(i)).z;
        this->kin_inliers.push_back(point);
    }

    this->quality = this->kin_inliers.size();
};


double Line_param::distance_to_point(pcl::PointXYZ p1) {
    pcl::PointXYZ M0M1;
    M0M1.x = this->r_vec.x - p1.x;
    M0M1.y = this->r_vec.y - p1.y;
    M0M1.z = this->r_vec.z - p1.z;
    pcl::PointXYZ d_vec = VectorMath::cross(M0M1, this->ldir_vec);
    return VectorMath::len(d_vec);
};


void Line_param::normalize(pcl::ModelCoefficients::Ptr coefficients)
{
    // line direction coordinates in kinect frame:
    double lx = coefficients->values[3]; // x is to the right
    double ly = coefficients->values[5]; // y is forward !FOR KINECT!

    // point on the line in kinect frame:
    double rx = coefficients->values[0]; // x is to the right
    double ry = coefficients->values[2]; // y is forward !FOR KINECT!

    if (rx*ly - ry*lx > 0) {
        lx = - lx;
        ly = - ly;
    }

    coefficients->values[3] = lx;
    coefficients->values[5] = ly;
};


// *****************************************
//              Line map
// *****************************************
Line_map::Line_map () : eps(20), derr (0.5)
{
    this->coefficients = pcl::ModelCoefficients::Ptr (new pcl::ModelCoefficients ());
    this->inliers      = pcl::PointIndices::Ptr      (new pcl::PointIndices ());
    // Optional

    this->seg.setOptimizeCoefficients (true);
    this->seg.setModelType (pcl::SACMODEL_LINE);
    this->seg.setMethodType (pcl::SAC_RANSAC);
    this->seg.setDistanceThreshold (0.003);

    this->cloud_f = pcl::PointCloud<pcl::PointXYZ>::Ptr (new pcl::PointCloud<pcl::PointXYZ>);
};


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
	    if (inliers->indices.size () <= min_points_in_line) break;
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
	if (fabs(lines.at(id).angle - angle) > eps ) return NULL;
	return &lines.at(id);
};



// *****************************************
//              Location server
// *****************************************
void LocationServer::track_wall(Line_param *wall)
{
    if (wall == NULL) {
        ROS_ERROR("LocationServer::track_wall argument is NULL");
        return;
    }

    if (this->ref_wall == NULL) {
        this->yaw = wall->angle;
    }

    this->ref_wall = wall;
    this->stm.angle = wall->angle;
    this->stm.distance = wall->distance;
    this->lost_ref_wall = false;

    this->corner_wall_left = lm.get_closest(this->stm.angle + 90);
    this->corner_wall_rght = lm.get_closest(this->stm.angle - 90);
};


void LocationServer::spin_once(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& cloud)
{
    if (cloud->points.size() < 10) {
        ROS_WARN("The cloud is empty. Location server is unable to provide pose estimation. Skipping...");
        if (this->ref_wall != NULL) {
            this->ref_wall->found = false;
        }
        return;
    };

    if (this->ref_wall == NULL) {
        ROS_WARN("No ref_wall. Using random!");
    };

    this->lm.renew(cloud);
    this->ref_wall = this->lm.get_best_fit(this->stm.angle, this->stm.distance);

    if (!this->ref_wall->found) { this->lost_ref_wall = true; ROS_WARN("Lost ref_wall"); }
    else this->lost_ref_wall = false;
    double d_angle = this->stm.angle - this->ref_wall->angle;
    this->stm.angle = this->ref_wall->angle;
    this->stm.distance = this->ref_wall->distance;

    this->yaw -= d_angle;

    this->corner_wall_left = lm.get_closest(this->stm.angle + 90);
    this->corner_wall_rght = lm.get_closest(this->stm.angle - 90);
};


bool LocationServer::obstacle_detected_left ()
{
    if (this->corner_wall_left == NULL) return false;
    return true;
};


bool LocationServer::obstacle_detected_rght ()
{
    if (this->corner_wall_rght == NULL) return false;
    return true;
};




