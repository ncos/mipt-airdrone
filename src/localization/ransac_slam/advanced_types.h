#ifndef ADVANCED_TYPES_H
#define ADVANCED_TYPES_H

#include <vector>
#include <cmath>

#include <ros/ros.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/filters/extract_indices.h>


typedef unsigned long long int uint64;


class Passage
{
public:
    double width;
    std::string frame;
    pcl::PointXYZ middle;
    pcl::PointXYZ left;
    pcl::PointXYZ rght;
    bool is_nan;
    double mid_ang, left_ang, rght_ang;

    Passage () : mid_ang(NAN), left_ang(NAN), rght_ang(NAN), width(NAN), is_nan(true),
            middle(NAN, NAN, NAN), left(NAN, NAN, NAN), rght(NAN, NAN, NAN), frame("")
    {}
};


class VectorMath
{
public:
    static pcl::PointXYZ cross(pcl::PointXYZ p1, pcl::PointXYZ p2);
    static double dot(pcl::PointXYZ p1, pcl::PointXYZ p2);
    static double len(pcl::PointXYZ p1);
    static pcl::PointXYZ to_e(pcl::PointXYZ p1);
};


class Line_param
{
public:
    bool found;
    double distance;
    int quality; // How good we can see this line
    std::string frame;
    uint64 global_id;
    std::vector<pcl::PointXYZ> kin_inliers;
    pcl::PointXYZ ldir_vec; // direction to the left across the line
    pcl::PointXYZ fdir_vec; // points to the line
    pcl::PointXYZ r_vec;    // the point lying the line
    double angle; // ldir angle with the 'x' frame axis

    Line_param ();

    void renew (pcl::ModelCoefficients::Ptr coefficients,
                pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud,
                pcl::PointIndices::Ptr inliers_idx);

    double distance_to_point(pcl::PointXYZ p1);

private:
    void normalize(pcl::ModelCoefficients::Ptr coefficients);
};


class LineMetrics
{
public:
    static double get_error (Line_param &l1, Line_param &l2);
    static double get_error (Line_param &l, double angle, double distance);
    static double get_error (double delta1, double delta2);
};




#endif
