#ifndef LINES_H
#define LINES_H

#include <vector>
#include <cmath>

#include <ros/ros.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/angles.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>


extern int min_points_in_line;

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


class Line_map
{
private:
	pcl::ExtractIndices<pcl::PointXYZ> extract;
	pcl::SACSegmentation<pcl::PointXYZ> seg;
	pcl::ModelCoefficients::Ptr coefficients;
	pcl::PointIndices::Ptr inliers;
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_f;

public:
	std::vector<Line_param> lines;
	double eps, derr;

	Line_map ();

	void renew (pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud);
	Line_param *get_best_fit (double angle, double distance);
	Line_param *get_closest  (double angle);
};


struct stMemory
{
    double angle;
    double distance;
    stMemory () : angle(0), distance(0)
    {}
};


class LocationServer
{
public:
    Line_map lm;
    bool lost_ref_wall;
private:
    double yaw;
    stMemory stm;
    Line_param *ref_wall, *corner_wall_left, *corner_wall_rght;

public:
    LocationServer () : corner_wall_left(NULL), corner_wall_rght(NULL),
                        lost_ref_wall(true), ref_wall(NULL), yaw(0) {};
    double get_yaw()        {return this->yaw; }
    void   set_zero_yaw()   {this->yaw = 0.0;  }
    void   track_wall           (Line_param *wall);
    Line_param  *get_ref_wall() {return this->ref_wall; }
    Line_param  *get_crn_wall_left() {return this->corner_wall_left; }
    Line_param  *get_crn_wall_rght() {return this->corner_wall_rght; }
    void   spin_once(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& cloud);
    bool   obstacle_detected_left ();
    bool   obstacle_detected_rght ();
};





#endif // LINES_H

