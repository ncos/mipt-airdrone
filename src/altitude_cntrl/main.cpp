#include <stdio.h>
#include <iostream>
#include <vector>
#include <math.h>

#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/common_headers.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Range.h>

#include "pid_regulator.h"
#include "ransac.h"



ros::Subscriber sub;
ros::Publisher  pub_vel;
ros::Publisher  pub_mrk;

double target_height = 0.0;
unsigned int fear_threshold = 10;
double max_takeoff_time = 3.0;

visualization_msgs::Marker height_text;


struct Fear_trigger
{
    bool decided_to_land;
    bool decided_to_takeoff;
    bool hovering;
    unsigned int uncertainty_cnt;
    double takeoff_timer, land_timer;
    Fear_trigger()
    {
        decided_to_land = false;
        decided_to_takeoff = false;
        hovering = false;
        uncertainty_cnt = 0;
    }

    Fear_trigger& operator++ ()
    {
        if (this->uncertainty_cnt < 100)
             this->uncertainty_cnt ++;
        if (uncertainty_cnt > fear_threshold && hovering)
        {
            ROS_ERROR("Arducopter decided to land due to altitude estimation errors!");
            this->decide_to_land();
        }
        return *this;
    }
    Fear_trigger& operator-- ()
    {
        if (this->uncertainty_cnt > 0)
            this->uncertainty_cnt --;
        return *this;
    }
    void decide_to_takeoff()
    {
        this->decided_to_takeoff = true;
        this->decided_to_land    = false;
        this->hovering           = false;
        this->takeoff_timer = ros::WallTime::now().toSec();
    }
    void decide_to_land()
    {
        this->decided_to_takeoff = false;
        this->decided_to_land    = true;
        this->hovering           = false;
        this->land_timer = ros::WallTime::now().toSec();
    }
    void decide_to_hover()
    {
        this->decided_to_takeoff = false;
        this->decided_to_land    = false;
        this->hovering           = true;
        this->uncertainty_cnt    = 0;
    }
    double get_takeoff_time()
    {
        return (ros::WallTime::now().toSec() - this->takeoff_timer);
    }
    double get_land_time()
    {
        return (ros::WallTime::now().toSec() - this->land_timer);
    }
    double stop_takeoff()
    {
        this->decide_to_hover();
        return get_takeoff_time();
    }
    double stop_land()
    {
        this->decide_to_hover();
        return get_land_time();
    }
    void print()
    {
        ROS_INFO("FT Counter val: %d/%d", this->uncertainty_cnt, fear_threshold);
    }
} fear_trigger;




void callback(const sensor_msgs::Range floor_msg)
{
    geometry_msgs::Twist base_cmd;
    PID pid_vel(1, 0.5, 0.5);


    if ((floor_msg.range < floor_msg.max_range) ||
        (floor_msg.range > floor_msg.min_range)) {
        --fear_trigger;
        char text[20];
        sprintf(text, "Height = %f", floor_msg.range);
        height_text.text = text;
        pub_mrk.publish(height_text);
    }
    else {
        height_text.text = "Height = Nan";
        pub_mrk.publish(height_text);
        ++fear_trigger;
    }






    base_cmd.linear.z = pid_vel.get_output(target_height, floor_msg.range);

    if (fear_trigger.decided_to_land) {
        ROS_INFO("Airdrone is landing (%2.1f s. passed)", fear_trigger.get_land_time());
        base_cmd.linear.z = -0.5;
        if(fear_trigger.get_land_time() > max_takeoff_time) { // TODO: Check the height here (if we happen to have a reliable altimeter)
            ROS_ERROR("Time is out. Goodbye, cruel world...");
            ros::shutdown(); // I hope this will kill the entire roslaunch (roslaunch should be configured for that)
        }
    }
    if (fear_trigger.decided_to_takeoff) {
        ROS_INFO("Airdrone is taking off blind (%2.1f s. passed)", fear_trigger.get_takeoff_time());
        base_cmd.linear.z = 0.5;
        if(fear_trigger.get_takeoff_time() > max_takeoff_time) {
            ROS_ERROR("Time is out, trying to land...");
            fear_trigger.decide_to_land();
        }
    }
    if (fear_trigger.uncertainty_cnt == 0 && fear_trigger.decided_to_takeoff) {
        fear_trigger.decide_to_hover();
        ROS_INFO("Airdrone haz reliable altitude. Continuing in normal mode");
    }

    pub_vel.publish(base_cmd);
};






int main (int argc, char** argv)
{
    ros::init (argc, argv, "Altitude_controller_ransac");

    ros::NodeHandle nh;
    std::string input_topic  = nh.resolveName("/sonar_height");
    std::string output_topic = nh.resolveName("/cmd_vel_1");
    std::string output_topic_mrk = nh.resolveName("visualization_marker");

    sub     = nh.subscribe<sensor_msgs::Range> (input_topic,  1, callback);
    pub_vel = nh.advertise<geometry_msgs::Twist> (output_topic, 1);
    pub_mrk = nh.advertise<visualization_msgs::Marker >     (output_topic_mrk, 5 );

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
    pub_mrk.publish(height_text);


    fear_trigger.decide_to_takeoff();
    ros::spin ();

    return 0;
};

