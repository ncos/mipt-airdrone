#include "da_vinci.h"





DaVinci::DaVinci(ros::NodeHandle nh_)
{
    this->nh = nh_;
    this->topic_name = this->nh.resolveName("visualization_marker");
    this->publisher = this->nh.advertise<visualization_msgs::Marker> (this->topic_name, 5);

    this->draw_link = "/camera_link";
    this->uid = 0;
};


void DaVinci::draw_point(double x, double y, double size, int id, POINT_COLOR color)
{
    if (isnan(x) || isnan(y)) return;

    visualization_msgs::Marker marker;

    marker.header.frame_id = this->draw_link;
    marker.ns = "point_ns";
    marker.action = visualization_msgs::Marker::ADD;
    marker.id = id;
    marker.type = visualization_msgs::Marker::SPHERE;

    marker.pose.position.x =  y;
    marker.pose.position.y = -x;
    marker.pose.position.z = 0;

    // Set the scale of the marker -- 1x1x1 here means 1m on a side
    marker.scale.x = size;
    marker.scale.y = size;
    marker.scale.z = size;

    // Set the color -- be sure to set alpha to something non-zero!
    marker.color = this->choose_color(color);

    marker.color.a = 0.6;
    marker.lifetime = ros::Duration(0.1);
    marker.header.stamp = ros::Time::now();
    // Publish the marker
    this->publisher.publish(marker);
};


void DaVinci::draw_point(double x, double y, int id, POINT_COLOR color)
{
    this->draw_point(x, y, 0.1, id, color);
};


void DaVinci::draw_point_inf(double x, double y)
{
    ROS_WARN("DaVinci::draw_point_inf is not properly tested");
    if (isnan(x) || isnan(y)) return;

    visualization_msgs::Marker marker;

    marker.header.frame_id = this->draw_link;
    marker.ns = "point_inf_ns";
    marker.action = visualization_msgs::Marker::ADD;
    marker.id = this->uid;
    this->uid ++;
    marker.type = visualization_msgs::Marker::SPHERE;

    marker.pose.position.x =  y;
    marker.pose.position.y = -x;
    marker.pose.position.z = 0;

    // Set the scale of the marker -- 1x1x1 here means 1m on a side
    marker.scale.x = 0.05;
    marker.scale.y = 0.05;
    marker.scale.z = 0.05;

    // Set the color -- be sure to set alpha to something non-zero!
    marker.color = this->choose_color(GOLD);

    marker.color.a = 0.6;
    marker.lifetime = ros::Duration();
    marker.header.stamp = ros::Time::now();
    this->publisher.publish(marker);
};


void DaVinci::draw_line (Line_param *lp, int id, POINT_COLOR color)
{
    if(lp == NULL) return;

    visualization_msgs::Marker marker;
    marker.header.frame_id = this->draw_link;
    marker.ns = "line_ns";
    marker.action = visualization_msgs::Marker::ADD;
    marker.id = id;
    marker.type = visualization_msgs::Marker::LINE_LIST;
    marker.scale.x = 0.02;

    const int length = 40;
    geometry_msgs::Point p;
    p.y = -lp->fdir_vec.kin.x * lp->distance + lp->ldir_vec.kin.x * length / 2;
    p.z =  0;
    p.x =  lp->fdir_vec.kin.y * lp->distance - lp->ldir_vec.kin.y * length / 2;
    marker.points.push_back(p);
    p.y =  p.y - lp->ldir_vec.kin.x * length;
    p.z =  0;
    p.x =  p.x + lp->ldir_vec.kin.y * length;
    marker.points.push_back(p);

    marker.color = this->choose_color(color);

    marker.lifetime = ros::Duration(0.1);
    marker.header.stamp = ros::Time::now();
    this->publisher.publish(marker);
};


void DaVinci::draw_e_vec(double x, double y, int id, POINT_COLOR color)
{
    const double len = 1.0;
    if (isnan(x) || isnan(y)) return;
    double vlen = sqrt(x * x + y * y);

    visualization_msgs::Marker marker;
    marker.header.frame_id = this->draw_link;
    marker.ns = "vec_ns";
    marker.action = visualization_msgs::Marker::ADD;
    marker.id = id;
    marker.type = visualization_msgs::Marker::ARROW;
    marker.scale.x = 0.01;
    marker.scale.y = 0.04;
    marker.scale.z = 0.05;

    geometry_msgs::Point p;
    p.y = 0;
    p.z = 0;
    p.x = 0;
    marker.points.push_back(p);
    p.y = -len * x / vlen;
    p.z =  0;
    p.x =  len * y / vlen;
    marker.points.push_back(p);

    marker.color = this->choose_color(color);

    marker.lifetime = ros::Duration(0.1);
    marker.header.stamp = ros::Time::now();
    this->publisher.publish(marker);
};


void DaVinci::draw_vec(double x, double y, int id, POINT_COLOR color)
{
    if (isnan(x) || isnan(y)) return;

    visualization_msgs::Marker marker;
    marker.header.frame_id = this->draw_link;
    marker.ns = "vec_ns";
    marker.action = visualization_msgs::Marker::ADD;
    marker.id = id;
    marker.type = visualization_msgs::Marker::ARROW;
    marker.scale.x = 0.01;
    marker.scale.y = 0.04;
    marker.scale.z = 0.05;

    geometry_msgs::Point p;
    p.y = 0;
    p.z = 0;
    p.x = 0;
    marker.points.push_back(p);
    p.y = -x;
    p.z =  0;
    p.x =  y;
    marker.points.push_back(p);

    marker.color = this->choose_color(color);

    marker.lifetime = ros::Duration(0.1);
    marker.header.stamp = ros::Time::now();
    this->publisher.publish(marker);
};


void DaVinci::draw_vec_cmd(geometry_msgs::Twist base_cmd, int id, POINT_COLOR color)
{
    this->draw_vec(-base_cmd.linear.y, base_cmd.linear.x, id, color);
};


void DaVinci::draw_vec_e_cmd(geometry_msgs::Twist base_cmd, int id, POINT_COLOR color)
{
    this->draw_e_vec(-base_cmd.linear.y, base_cmd.linear.x, id, color);
};


void DaVinci::draw_point_cmd(double x, double y, int id, POINT_COLOR color)
{
    this->draw_point(-y, x, id, color);
};


void DaVinci::draw_point_cmd(double x, double y, double size, int id, POINT_COLOR color)
{
    this->draw_point(-y, x, size, id, color);
};


void DaVinci::clear_inf()
{
    visualization_msgs::Marker marker;

    for (; this->uid >= 0; --this->uid) {
        marker.header.frame_id = this->draw_link;
        marker.ns = "point_inf_ns";
        marker.action = visualization_msgs::Marker::DELETE;
        marker.id = this->uid;
        marker.type = visualization_msgs::Marker::SPHERE;
        marker.lifetime = ros::Duration();
        marker.header.stamp = ros::Time::now();
        this->publisher.publish(marker);
    }
};


DaVinci::~DaVinci()
{
    this->clear_inf();
};


visualization_msgs::Marker::_color_type DaVinci::choose_color(POINT_COLOR color)
{
    visualization_msgs::Marker::_color_type col;

    switch (color)
    {
    case RED:
        col.r = 255;
        col.g =   0;
        col.b =   0;
        break;
    case GREEN:
        col.r =   0;
        col.g = 255;
        col.b =   0;
        break;
    case BLUE:
        col.r =   0;
        col.g =   0;
        col.b = 255;
        break;
    case CYAN:
        col.r =   0;
        col.g = 255;
        col.b = 255;
        break;
    case GOLD:
        col.r = 218;
        col.g = 165;
        col.b =  32;
        break;
    default:
        col.r = 255;
        col.g = 255;
        col.b = 255;
    }
    col.r = col.r / 255;
    col.g = col.g / 255;
    col.b = col.b / 255;
    col.a = 0.3;

    return col;
};

