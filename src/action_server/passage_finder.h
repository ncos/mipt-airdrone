#ifndef PASSAGEFINDER_H
#define PASSAGEFINDER_H

#include <ros/ros.h>
#include <pcl/point_cloud.h>
#include <geometry_msgs/Twist.h>
#include <boost/thread/mutex.hpp>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_listener.h>

// Message files
#include <ransac_slam/LineMap.h>

#include <vector>
#include <cmath>

#include "RANSAC.h"
#include "pid_regulator.h"
#include "da_vinci.h"  // Draw stuff in Rviz

// Set in main.cpp:
extern std::string fixed_frame;
extern std::string base_footprint_frame;
extern std::string base_stabilized_frame;
extern std::string pointcloud_frame;


enum PassageType {
    ortogonal, parrallel, single_wall, undefined, non_valid
};


class VectorMath
{
public:
    static pcl::PointXYZ cross(pcl::PointXYZ p1, pcl::PointXYZ p2);
    static double dot(pcl::PointXYZ p1, pcl::PointXYZ p2);
    static double len(pcl::PointXYZ p1);
    static pcl::PointXYZ to_e(pcl::PointXYZ p1);
};


struct Passage
{
	double width;
	pcl::PointXYZ kin_middle;
	pcl::PointXYZ kin_left;
	pcl::PointXYZ kin_rght;
    pcl::PointXYZ cmd_middle;
    pcl::PointXYZ cmd_left;
    pcl::PointXYZ cmd_rght;
	bool is_nan;
	double mid_ang, left_ang, rght_ang;

	Passage () : mid_ang(NAN), left_ang(NAN), rght_ang(NAN), width(NAN), is_nan(true),
	        kin_middle(NAN, NAN, NAN), kin_left(NAN, NAN, NAN), kin_rght(NAN, NAN, NAN),
	        cmd_middle(NAN, NAN, NAN), cmd_left(NAN, NAN, NAN), cmd_rght(NAN, NAN, NAN)
	{}
};


struct stMemory
{
	double angle;
	double distance;
	stMemory () : angle(0),
				  distance(0)
	{}
};


class Advanced_Passage_finder // Da ADVANCED Passage findr! (looks for holes in walls and is better than Passage findr)
{
public:
    std::vector<Passage> passages;
    void renew(const ransac_slam::LineMap::ConstPtr& lines_msg);
    bool passage_on_line(Line_param &line, Passage &passage);
    Line_param *get_best_line(pcl::PointXYZ &point, Line_map &linemap);
    Line_param *get_best_opposite_line(pcl::PointXYZ pass_point_kin, pcl::PointXYZ pass_point_kin_op, Line_map &linemap);

private:
    void add_passage(std::string frame, pcl::PointXYZ point1, pcl::PointXYZ point2);
    double sqrange(pcl::PointXYZ p1, pcl::PointXYZ p2);
    pcl::PointXYZ get_closest_left(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr &cloud, int point_id);
    pcl::PointXYZ get_closest_rght(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr &cloud, int point_id);
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
	boost::shared_ptr<boost::mutex> mutex;

public:
	LocationServer (boost::shared_ptr<boost::mutex> _mutex) : mutex(_mutex),
															  corner_wall_left(NULL),
															  corner_wall_rght(NULL),
															  lost_ref_wall(true),
															  ref_wall(NULL),
															  yaw(0)
															  {};

	double get_yaw() 		{return this->yaw; }
	void   set_zero_yaw() 	{this->yaw = 0.0;  }
	void   track_wall			(Line_param *wall);
	Line_param  *get_ref_wall() {return this->ref_wall; }
	Line_param  *get_crn_wall_left() {return this->corner_wall_left; }
	Line_param  *get_crn_wall_rght() {return this->corner_wall_rght; }
    void   spin_once(const ransac_slam::LineMap::ConstPtr& lines);
	bool   obstacle_detected_left ();
	bool   obstacle_detected_rght ();
	void   lock() {this->mutex->lock(); }
	void   unlock() {this->mutex->unlock(); }
};


class MappingServer
{
private:
    tf::StampedTransform prev_transform;
    boost::shared_ptr<boost::mutex> mutex;
    tf::TransformListener tf_listener;
    bool init_flag; // Need to fix bug with start position in gazebo
    int rotation_cnt;
    std::vector<pcl::PointXYZ> visited_points;

public:
    std::vector<pcl::PointXYZ> tracked_points;
    std::vector<pcl::PointXYZ> landing_points;
    pcl::PointXYZ aver_land_pad;
    pcl::PointXYZ move_target;

public:
    MappingServer ();
    pcl::PointXYZ get_global_positon();
    double get_global_angle();
    double diff(double a, double b);
    pcl::PointXYZ diff(pcl::PointXYZ a, pcl::PointXYZ b);
    int track (pcl::PointXYZ p);
    pcl::PointXYZ do_transform(const pcl::PointXYZ point);
    int add_land_pad (pcl::PointXYZ p);
    void spin_once ();
    void clear_land_pad();
    void add_move_target_track (pcl::PointXYZ p);
    pcl::PointXYZ rotate(const pcl::PointXYZ vec, double angle);


private:
    void lock() {this->mutex->lock(); }
    void unlock() {this->mutex->unlock(); }
    void add_visited ();
    void resize_land_pads ();
};


class MotionServer
{
private:
    PID pid_ang;
    PID pid_vel;
    Line_param *ref_wall;
    boost::shared_ptr<boost::mutex> mutex;
    bool tracking_on;
    pcl::PointXYZ move_target;
    double move_vel;
    double move_phi;
    double move_rot_vel;

    // Utility variables
    boost::shared_ptr<MappingServer> map_srv;
    tf::TransformListener tf_listener;
    double move_epsilon;
    double move_rot_epsilon;
    double prev_angl;
    pcl::PointXYZ prev_pos;
    double height_epsilon;
    double prev_height;

public:
    double ref_dist, ref_ang;
    geometry_msgs::Twist base_cmd, buf_cmd;
    bool move_done;
    bool rot_done;
    double target_height;
    double height;
    double vert_vel;
    bool height_done;
    bool on_floor;
    ros::Publisher pub_mrk;
    visualization_msgs::Marker height_text;

public:
    MotionServer (boost::shared_ptr<boost::mutex> _mutex, boost::shared_ptr<MappingServer> _map_srv) :
        mutex(_mutex), map_srv (_map_srv), ref_wall(NULL), ref_dist(0), ref_ang(0), tracking_on(true),
        move_target (pcl::PointXYZ(0, 0, 0)), move_vel (0), move_phi (0),
        move_rot_vel (0), move_epsilon (0.1), move_rot_epsilon (0), prev_angl (0),
        prev_pos (pcl::PointXYZ (0, 0, 0)), move_done (true), rot_done (true), height_done (true),
        vert_vel (0), height_epsilon (0.1), on_floor (true), height (0)
    {
        ros::NodeHandle nh;
        std::string output_topic_mrk = nh.resolveName("visualization_marker");

        pub_mrk = nh.advertise<visualization_msgs::Marker >(output_topic_mrk, 5 );

        if (!nh.getParam("base_height", target_height)) ROS_ERROR("Failed to get param 'base_height'");

        height_text.header.frame_id = "/kinect_link";
        height_text.ns = "text_ns";
        height_text.action = visualization_msgs::Marker::ADD;
        height_text.id = 100;
        height_text.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
        height_text.scale.z = 0.2;
        height_text.pose.position.z = 1;
        height_text.color.r = 1.0;
        height_text.color.g = 0.0;
        height_text.color.b = 0.0;
        height_text.color.a = 0.3;
        height_text.text    = "Initializing altitude controller...";
        height_text.lifetime = ros::Duration(0.2);

        tf::StampedTransform transform;
        try {this->tf_listener.waitForTransform(base_footprint_frame, base_stabilized_frame, ros::Time(0), ros::Duration(10.0) ); }
        catch (tf::TransformException &ex) {ROS_ERROR("Action Server Node: (wait) Unable to transform: %s", ex.what()); }

        try {this->tf_listener.lookupTransform(base_footprint_frame, base_stabilized_frame, ros::Time(0), transform); }
        catch (tf::TransformException &ex) {ROS_ERROR("Action Server Node: (lookup) Unable to transform: %s", ex.what()); }

        this->target_height = this->prev_height = transform.getOrigin().z();
    }
    ~MotionServer ();
    void set_pid_vel  (double P, double I, double D) {this->pid_vel.set_PID(P, I, D); }
    void set_pid_ang  (double P, double I, double D) {this->pid_ang.set_PID(P, I, D); }
    void set_ref_wall (Line_param *wall);
    void track ();
    void untrack ();
    void set_angles_current ();
    bool rotate(double angle);
    bool move_parallel(double vel);
    bool move_perpendicular(double shift);
    void spin_once();
    void clear_cmd();
    void lock() {this->mutex->lock(); }
    void unlock() {this->mutex->unlock(); }
    void set_target_angle (double angle) {this->ref_ang  = angle; }
    void set_target_dist  (double dist ) {this->ref_dist = dist;  }
    void move(pcl::PointXYZ target, double vel, double phi, double rot_vel);
    void move_step();
    void set_height(double height);
    void altitude_step();
};


struct Passage_type
{
    int type;
    bool pass_exist;
    bool closest_exist;
    bool opposite_exist;
    bool middle_exist;
    boost::shared_ptr<boost::mutex> mutex;

    Passage_type () : type(non_valid), pass_exist (false), closest_exist(false),
                      opposite_exist(false), middle_exist(false) {}

    int recognize (boost::shared_ptr<Advanced_Passage_finder> apf, Line_map lm, bool on_left_side);
};

#endif // PASSAGEFINDER_H
