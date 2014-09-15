#include <vector>
#include <math.h>
#include <algorithm>    // std::sort

#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/common_headers.h>
#include <pcl/filters/passthrough.h>
#include <tf/transform_listener.h>
#include <sensor_msgs/PointCloud.h>
#include <tf/message_filter.h>
#include <message_filters/subscriber.h>
#include <laser_geometry/laser_geometry.h>

#include "lines.h"
#include "da_vinci.h" // Draw in Rviz


std::string cloud_topic;
std::string visualization_topic; // Rviz markers
unsigned long long int min_points_in_cloud;

class LaserScanProcessor
{
private:
    ros::NodeHandle n_;
    laser_geometry::LaserProjection projector_;
    tf::TransformListener listener_;
    message_filters::Subscriber<sensor_msgs::LaserScan> laser_sub_;
    tf::MessageFilter<sensor_msgs::LaserScan> laser_notifier_;
    ros::Publisher scan_pub_;
    LocationServer loc_srv;
    DaVinci davinci;

public:
    LaserScanProcessor(ros::NodeHandle n) : n_(n), laser_sub_(n,"/sensors/kinect/scan", 10), davinci(n, visualization_topic),
                                            laser_notifier_(laser_sub_, listener_, "base_link", 10) {
        this->laser_notifier_.registerCallback(boost::bind(&LaserScanProcessor::scanCallback, this, _1));
        this->laser_notifier_.setTolerance(ros::Duration(0.01));
        this->scan_pub_ = n_.advertise<sensor_msgs::PointCloud>("/my_cloud", 1);
    }

private:
    void scanCallback (const sensor_msgs::LaserScan::ConstPtr& scan_in) {
        std::string input_frame = scan_in->header.frame_id;
        sensor_msgs::PointCloud cloud;
        try {
            projector_.transformLaserScanToPointCloud("base_link", *scan_in, cloud, listener_);
        }
        catch (tf::TransformException& e) {
            std::cout << e.what();
            return;
        }

        this->davinci.draw_line(input_frame, loc_srv.get_crn_wall_left(), 1, BLUE);
        this->davinci.draw_line(input_frame, loc_srv.get_ref_wall(),      0, GREEN);
        this->davinci.draw_line(input_frame, loc_srv.get_crn_wall_rght(), 2, BLUE);

        // Do something with cloud.

        scan_pub_.publish(cloud);
    }
};


class CloudProcessor
{
private:
    ros::NodeHandle n_;
    pcl::PointCloud<pcl::PointXYZ>::Ptr laser_cloud;
    ros::Subscriber cloud_sub;
    tf::TransformListener listener_;
    LocationServer loc_srv;
    DaVinci davinci;

public:
    CloudProcessor(ros::NodeHandle n) : n_(n), davinci(n, visualization_topic),
                                        laser_cloud(new pcl::PointCloud<pcl::PointXYZ>) {
        this->cloud_sub = n.subscribe<pcl::PointCloud<pcl::PointXYZ> > (cloud_topic, 1, &CloudProcessor::rgbdCallback, this);
    }

private:
    struct Cmp_class
    {
        bool operator() (pcl::PointXYZ point_1, pcl::PointXYZ point_2) {
            if(point_1.z == 0 || point_2.z == 0) return true;
            return ((point_1.x/point_1.z) < (point_2.x/point_2.z) );
        }
    } cmp_class;

    void rgbdCallback (const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& cloud) {
        if(cloud->points.size() < min_points_in_cloud) {
            ROS_ERROR("rgbdCallback: The input cloud size is %lu points! \
                      (this is too little to provide an adequate position estimation)", cloud->points.size());
        }

        std::string input_frame = cloud->header.frame_id;
        pcl::PassThrough<pcl::PointXYZ> pass;
        pass.setInputCloud (cloud);
        pass.setFilterFieldName ("y");      // z is to front, y is DOWN!
        pass.setFilterLimits (-0.3, -0.2);  // (-0.3, -0.2)
        pass.filter (*this->laser_cloud);

        for (int i = 0; i < this->laser_cloud->points.size(); i++) {
            this->laser_cloud->points[i].y = 0;
        }

        std::sort (this->laser_cloud->points.begin(), this->laser_cloud->points.end(), this->cmp_class); // Sorting by angle
        if(this->laser_cloud->points.size() < min_points_in_cloud) {
            ROS_ERROR("rgbdCallback: The cloud size after processing is %lu points! \
                      (this is too little to provide an adequate position estimation)", this->laser_cloud->points.size());
        }

        this->loc_srv.spin_once(this->laser_cloud);

        ROS_INFO("angle: %f\t range: %f", loc_srv.get_ref_wall()->angle, loc_srv.get_ref_wall()->distance);

        //input_frame = "kinect_link";
        this->davinci.draw_line(input_frame, loc_srv.get_crn_wall_left(), 10, BLUE);
        this->davinci.draw_line(input_frame, loc_srv.get_ref_wall(),      11, GREEN);
        this->davinci.draw_line(input_frame, loc_srv.get_crn_wall_rght(), 12, BLUE);
    }
};








int main(int argc, char** argv)
{
    ros::init(argc, argv, "ransac_slam");
    ros::NodeHandle nh;

    cloud_topic = "/sensors/kinect/depth/points";
    visualization_topic = "visualization/rviz_markers";
    min_points_in_cloud = 10;

    CloudProcessor cp(nh);

    ros::spin();
    return 0;
}
