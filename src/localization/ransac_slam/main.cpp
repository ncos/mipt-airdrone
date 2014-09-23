#include <vector>
#include <math.h>
#include <algorithm> // std::sort

#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/common_headers.h>
#include <pcl/filters/passthrough.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud.h>
#include <message_filters/subscriber.h>
#include <laser_geometry/laser_geometry.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <tf/message_filter.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <sensor_msgs/Range.h>
#include <nav_msgs/Odometry.h>

#include "lines.h"
#include "da_vinci.h" // Draw in Rviz



// Parameters from param.yaml
int min_points_in_cloud;
int min_points_in_line;
std::string input_cloud_topic;
std::string output_cloud_topic;
int shrink_order;
std::string output_cloud_shrinked_topic;
std::string visualization_topic; // Rviz markers
bool publish_clouds;
std::string fixed_frame;
std::string base_frame;
std::string kinect_depth_optical_frame;
std::string output_frame;
std::string output_odom_topic;
bool publish_tf;
bool use_sonar_data;
std::string input_sonar_topic;

class CloudProcessor
{
private:
    ros::NodeHandle n_;
    pcl::PointCloud<pcl::PointXYZ>::Ptr laser_cloud;
    pcl::PointCloud<pcl::PointXYZ>::Ptr laser_cloud_shrinked;
    ros::Subscriber cloud_sub, sonar_sub;
    ros::Publisher pub_laser_cloud, pub_laser_cloud_shrinked;
    ros::Publisher pub_odom_out;
    tf::TransformListener tf_listener;
    tf::TransformBroadcaster tf_broadcaster;
    LocationServer loc_srv;
    DaVinci davinci;
    bool all_trasforms_are_initialized;

public:
    CloudProcessor(ros::NodeHandle n) : n_(n), davinci(n, visualization_topic),
                                        laser_cloud(new pcl::PointCloud<pcl::PointXYZ>),
                                        laser_cloud_shrinked(new pcl::PointCloud<pcl::PointXYZ>) {
        this->all_trasforms_are_initialized = false;
        this->cloud_sub = n.subscribe<pcl::PointCloud<pcl::PointXYZ> > (input_cloud_topic, 1, &CloudProcessor::rgbdCallback, this);
        this->sonar_sub = n.subscribe<sensor_msgs::Range> (input_sonar_topic, 1, &CloudProcessor::sonarCallback, this);
        this->pub_laser_cloud = n.advertise<pcl::PointCloud<pcl::PointXYZ> > (output_cloud_topic, 1);
        this->pub_laser_cloud_shrinked = n.advertise<pcl::PointCloud<pcl::PointXYZ> > (output_cloud_shrinked_topic, 1);
        this->pub_odom_out = n.advertise<nav_msgs::Odometry> (output_odom_topic, 1);
        if (use_sonar_data == false) this->sonar_sub.shutdown();

        try {
            this->tf_listener.waitForTransform(base_frame, fixed_frame, ros::Time(0), ros::Duration(10.0) );
            this->tf_listener.waitForTransform(fixed_frame, kinect_depth_optical_frame, ros::Time(0), ros::Duration(10.0) );
        }
        catch (tf::TransformException &ex) {
            ROS_ERROR("%s", ex.what());
            ros::Duration(1.0).sleep();
        }
    }

private:
    struct Cmp_class
    {
        bool operator() (pcl::PointXYZ point_1, pcl::PointXYZ point_2) {
            if(point_1.z == 0 || point_2.z == 0) return true;
            return ((point_1.x / point_1.z) < (point_2.x / point_2.z) );
        }
    } cloud_cmp_class;

    void update_shrinked_cloud(unsigned int order) {
        this->laser_cloud_shrinked->clear();
        if (this->laser_cloud->points.size() == 0) return;
        if (order == 0) return;

        for (int i = 0; i < this->laser_cloud->points.size(); ++i) {
            if (i % order == 0) {
                this->laser_cloud_shrinked->push_back(this->laser_cloud->points.at(i));
            }
        }
        this->laser_cloud_shrinked->header = this->laser_cloud->header;
    }

    void rgbdCallback (const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& cloud) {
        if(cloud->points.size() < min_points_in_cloud) {
            ROS_ERROR("rgbdCallback: The input cloud size is %lu points! \
                      (this is too little to provide an adequate position estimation)", cloud->points.size());
        }

        pcl::PassThrough<pcl::PointXYZ> pass;
        pass.setInputCloud (cloud);
        pass.setFilterFieldName ("y");      // z is to front, y is DOWN!
        pass.setFilterLimits (-0.3, -0.2);  // (-0.3, -0.2)
        pass.filter (*this->laser_cloud);
        this->laser_cloud->header = cloud->header;

        for (int i = 0; i < this->laser_cloud->points.size(); i++) {
            this->laser_cloud->points[i].y = 0;
        }

        std::sort (this->laser_cloud->points.begin(), this->laser_cloud->points.end(), this->cloud_cmp_class); // Sorting by angle
        if(this->laser_cloud->points.size() < min_points_in_cloud) {
            ROS_ERROR("rgbdCallback: The cloud size after processing is %lu points! \
                      (this is too little to provide an adequate position estimation)", this->laser_cloud->points.size());
        }
        this->update_shrinked_cloud(shrink_order);

        tf::StampedTransform fixed_to_base, base_to_cloud;
        try {
            this->tf_listener.lookupTransform(fixed_frame, base_frame, ros::Time(0), fixed_to_base);
            this->tf_listener.lookupTransform(base_frame, this->laser_cloud->header.frame_id, ros::Time(0), base_to_cloud);
        }
        catch (tf::TransformException &ex) {
            ROS_ERROR("%s", ex.what());
            ros::Duration(1.0).sleep();
        }

        nav_msgs::Odometry map_to_cloud = this->loc_srv.spin_once(this->laser_cloud, fixed_to_base, base_to_cloud);

        //this->davinci.draw_line(pcl_conversions::fromPCL(laser_cloud->header), loc_srv.get_crn_wall_left(), 10, BLUE);

        if (publish_clouds) {
            this->pub_laser_cloud.publish(this->laser_cloud);
            this->pub_laser_cloud_shrinked.publish(this->laser_cloud_shrinked);
        }

        this->pub_odom_out.publish(map_to_cloud);
    }


    void sonarCallback (const sensor_msgs::Range range_msg) {
        this->loc_srv.range_from_sonar = range_msg.range;
    }
};








int main(int argc, char** argv)
{
    ros::init(argc, argv, "ransac_slam");
    ros::NodeHandle nh;

    if (!nh.getParam("ransac_slam/min_points_in_line", min_points_in_line)) min_points_in_line = 50;
    if (!nh.getParam("ransac_slam/min_points_in_cloud", min_points_in_cloud)) min_points_in_cloud = 10;
    if (!nh.getParam("ransac_slam/input_cloud_topic", input_cloud_topic)) input_cloud_topic = "/sensors/kinect/depth/points";
    if (!nh.getParam("ransac_slam/output_cloud_topic", output_cloud_topic)) output_cloud_topic = "/ransac_slam/flat_cloud";
    if (!nh.getParam("ransac_slam/shrink_order", shrink_order)) shrink_order = 50;
    if (!nh.getParam("ransac_slam/output_cloud_shrinked_topic", output_cloud_shrinked_topic)) output_cloud_shrinked_topic = "/ransac_slam/shrinked_flat_cloud";
    if (!nh.getParam("ransac_slam/visualization_topic", visualization_topic)) visualization_topic = "/ransac_slam/markers";
    if (!nh.getParam("ransac_slam/publish_clouds", publish_clouds)) publish_clouds = true;
    if (!nh.getParam("ransac_slam/fixed_frame", fixed_frame)) fixed_frame = "/odom";
    if (!nh.getParam("ransac_slam/base_frame", base_frame)) base_frame = "/base_stabilized";
    if (!nh.getParam("ransac_slam/kinect_depth_optical_frame", kinect_depth_optical_frame)) kinect_depth_optical_frame = "/kinect_depth_optical_frame";
    if (!nh.getParam("ransac_slam/output_frame", output_frame)) output_frame = "/ransac_slam/tf_output";
    if (!nh.getParam("ransac_slam/publish_tf", publish_tf)) publish_tf = true;
    if (!nh.getParam("ransac_slam/use_sonar_data", use_sonar_data)) use_sonar_data = true;
    if (!nh.getParam("ransac_slam/input_sonar_topic", input_sonar_topic)) input_sonar_topic = "/sonar_height";
    if (!nh.getParam("ransac_slam/output_odom_topic", output_odom_topic)) output_odom_topic = "/ransac_slam/odom_out";


    CloudProcessor cp(nh);

    ros::spin();
    return 0;
}
