#ifndef LINES_H
#define LINES_H

#include <vector>
#include <cmath>

#include <boost/assign/list_of.hpp> // for 'list_of()'

#include <ros/ros.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/angles.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <tf/tf.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>

extern int min_points_in_line;
extern int min_points_in_cloud;
extern std::string fixed_frame;
extern std::string base_frame;

class VectorMath
{
public:
    static pcl::PointXYZ cross(pcl::PointXYZ p1, pcl::PointXYZ p2) {
        pcl::PointXYZ result;
        result.x = p1.y * p2.z - p1.z * p2.y;
        result.y = p1.z * p2.x - p1.x * p2.z;
        result.z = p1.x * p2.y - p1.y * p2.x;
        return result;
    };

    static double dot(pcl::PointXYZ p1, pcl::PointXYZ p2) {
        return p1.x * p2.x + p1.y * p2.y + p1.z * p2.z;
    };

    static double len(pcl::PointXYZ p1) {
        return sqrt(dot(p1, p1));
    }

    static pcl::PointXYZ to_e(pcl::PointXYZ p1) {
        pcl::PointXYZ result;
        result.x = p1.x / len(p1);
        result.y = p1.y / len(p1);
        result.z = p1.z / len(p1);
        return result;
    }
};


class Line_param
{
public:
	bool found;
	double distance;
	int quality; // How good we can see this line
	std::string frame;
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


class Line_map
{
private:
	pcl::ExtractIndices<pcl::PointXYZ> extract;
	pcl::SACSegmentation<pcl::PointXYZ> seg;
	pcl::ModelCoefficients::Ptr coefficients;
	pcl::PointIndices::Ptr inliers;
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_f;

	struct lineCmp
	{
	    double angle, distance;
	    lineCmp(double a, double d) {
	        this->angle = a;
	        this->distance = d;
	    }
	    bool operator() (Line_param l1, Line_param l2) {
	        double err1 = LineMetrics::get_error(l1, this->angle, this->distance);
            double err2 = LineMetrics::get_error(l2, this->angle, this->distance);
            return err1 < err2;
	    }
	};

public:
	std::vector<Line_param> lines;

	Line_map ();

	void renew (pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud);
	Line_param get_best_fit (double angle, double distance);
	Line_param get_best_fit_a (double angle);
    Line_param get_best_fit_d (double distance);
    void sort_lines_a (double angle,    std::vector<Line_param> &l);
    void sort_lines_d (double distance, std::vector<Line_param> &l);

private:

};



class BruteForceMatcher
{
public:
    struct Pair
    {
        Line_param l1, l2;
        double err;
        Pair (Line_param &l1_, Line_param &l2_) {
            l1 = l1_;
            l2 = l2_;
            err = LineMetrics::get_error(l1, l2);
        }
    };

public:
    // returns the common error of the match
    static double match(std::vector<Line_param> l1, std::vector<Line_param> l2,
                 std::vector<BruteForceMatcher::Pair> &matched, std::vector<Line_param> &unmatched);

private:
    static unsigned int get_best_fit(Line_param &line, std::vector<Line_param> &lines);

};


class LocationServer
{
public:
    Line_map lm;
    double range_from_sonar;

public:
    LocationServer (): range_from_sonar(0)
                                       {};
    nav_msgs::Odometry spin_once(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& cloud, tf::Transform fixed_to_base, tf::Transform base_to_cloud);

};





#endif // LINES_H

