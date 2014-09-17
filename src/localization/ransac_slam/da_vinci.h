#ifndef DAVINCI_H
#define DAVINCI_H

#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Twist.h>

#include "lines.h"


// Draw in kinect coordinates
enum POINT_COLOR {RED, GREEN, BLUE, GOLD, CYAN};
class DaVinci
{
    ros::NodeHandle nh;
    ros::Publisher  publisher;
    std::string topic_name;
    unsigned long uid;

public:
    DaVinci(ros::NodeHandle nh_, std::string topic_name);
    void draw_point(std::string frame, pcl::PointXYZ point, int id, POINT_COLOR color);
    void draw_point(std::string frame, pcl::PointXYZ point, double size, int id, POINT_COLOR color);
    void draw_point_inf(std::string frame, pcl::PointXYZ point);
    void draw_line   (std::string frame, Line_param *lp, int id, POINT_COLOR color);
    void draw_line   (std_msgs::Header header, Line_param *lp, int id, POINT_COLOR color);
    void draw_e_vec  (std::string frame, pcl::PointXYZ point, int id, POINT_COLOR color);
    void draw_vec    (std::string frame, pcl::PointXYZ point, int id, POINT_COLOR color);
    void clear_inf(std::string frame);
    ~DaVinci();
private:
    visualization_msgs::Marker::_color_type choose_color(POINT_COLOR color);
};



#endif // DAVINCI_H

