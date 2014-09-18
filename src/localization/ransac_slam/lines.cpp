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
Line_map::Line_map ()
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

Line_param Line_map::get_best_fit (double angle, double distance)
{
    std::vector<Line_param> l;
    for (int i = 0; i < this->lines.size(); ++i) {
        l.push_back(lines[i]);
    }
    lineCmp line_cmp_class(angle, distance);
    std::sort (l.begin(), l.end(), line_cmp_class);
    if (l.size() == 0) return NULL;
    return l[0];
};

Line_param Line_map::get_best_fit_a (double angle)
{
    std::vector<Line_param> l;
    this->sort_lines_a(angle, l);
    if (l.size() == 0) return NULL;
    return l[0];
};

Line_param Line_map::get_best_fit_d (double distance)
{
    std::vector<Line_param> l;
    this->sort_lines_d(distance, l);
    if (l.size() == 0) return NULL;
    return l[0];
};

void Line_map::sort_lines_a (double angle, std::vector<Line_param> &l)
{
    for (int i = 0; i < this->lines.size(); ++i) {
        l.push_back(lines[i]);
    }
    lineCmp line_cmp_class(angle, NAN);
    std::sort (l.begin(), l.end(), line_cmp_class);

};

void Line_map::sort_lines_d (double distance, std::vector<Line_param> &l)
{
    for (int i = 0; i < this->lines.size(); ++i) {
        l.push_back(lines[i]);
    }
    lineCmp line_cmp_class(NAN, distance);
    std::sort (l.begin(), l.end(), line_cmp_class);

};

// *****************************************
//              Line metrics
// *****************************************
double LineMetrics::get_error (Line_param &l1, Line_param &l2)
{
    return LineMetrics::get_error(l1.angle - l2.angle, l1.distance - l2.distance);
};

double LineMetrics::get_error (Line_param &l, double angle, double distance)
{
    return LineMetrics::get_error(l.angle - angle, l.distance - distance);
};

double LineMetrics::get_error (double delta1, double delta2)
{
    double d1 = delta1, d2 = delta2;
    if (isnan(delta1)) d1 = 0;
    if (isnan(delta2)) d2 = 0;
    return sqrt(d1 * d1 + d2 * d2);
};

// *****************************************
//              Brute force matcher
// *****************************************
BruteForceMatcher
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
    if (cloud->points.size() < min_points_in_cloud) {
        ROS_WARN("The cloud is too small (%lu points). Location server is unable \
                                        to provide reliable pose estimation", cloud->points.size());
        return;
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




