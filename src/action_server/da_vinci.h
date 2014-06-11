#ifndef DAVINCI_H
#define DAVINCI_H

#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Twist.h>

#include "RANSAC.h"


// Draw in kinect coordinates
enum POINT_COLOR {RED, GREEN, BLUE, GOLD};
class DaVinci
{
    ros::NodeHandle nh;
    ros::Publisher  publisher;
    std::string topic_name;
    std::string draw_link;
    unsigned long uid;

public:
    DaVinci(ros::NodeHandle nh_);
    void draw_point(double x, double y, int id, POINT_COLOR color);
    void draw_point_inf(double x, double y);
    void draw_line (Line_param *lp, int id, POINT_COLOR color);
    void draw_e_vec  (double x, double y, int id, POINT_COLOR color);
    void draw_vec    (double x, double y, int id, POINT_COLOR color);
    void draw_vec_cmd(geometry_msgs::Twist base_cmd, int id, POINT_COLOR color);
    void draw_pos_cmd(double x, double y, int id, POINT_COLOR color);
    void clear_inf();
    ~DaVinci();
private:
    visualization_msgs::Marker::_color_type choose_color(POINT_COLOR color);
};


extern boost::shared_ptr<DaVinci>  davinci;


#endif // DAVINCI_H

