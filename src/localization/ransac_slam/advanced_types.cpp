#include "advanced_types.h"


// *****************************************
//              Vector Math
// *****************************************
pcl::PointXYZ VectorMath::cross(pcl::PointXYZ p1, pcl::PointXYZ p2) {
    pcl::PointXYZ result;
    result.x = p1.y * p2.z - p1.z * p2.y;
    result.y = p1.z * p2.x - p1.x * p2.z;
    result.z = p1.x * p2.y - p1.y * p2.x;
    return result;
};

double VectorMath::dot(pcl::PointXYZ p1, pcl::PointXYZ p2) {
    return p1.x * p2.x + p1.y * p2.y + p1.z * p2.z;
};

double VectorMath::len(pcl::PointXYZ p1) {
    return sqrt(dot(p1, p1));
};

pcl::PointXYZ VectorMath::to_e(pcl::PointXYZ p1) {
    pcl::PointXYZ result;
    result.x = p1.x / len(p1);
    result.y = p1.y / len(p1);
    result.z = p1.z / len(p1);
    return result;
};



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

