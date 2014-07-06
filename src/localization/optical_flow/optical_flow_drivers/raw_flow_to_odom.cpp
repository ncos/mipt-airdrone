#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <optical_flow/OpticalFlow.h>
#include <nav_msgs/Odometry.h>

#include <boost/assign/list_of.hpp> // for 'list_of()'


// Check the http://wiki.ros.org/navigation/Tutorials/RobotSetup/Odom

ros::Publisher  odom_pub;
ros::Subscriber flow_sub;
boost::shared_ptr<tf::TransformBroadcaster> odom_broadcaster;
ros::Time current_time, last_time;

#define MAX_DELTA_TIME 0.1

double x = 0.0;
double y = 0.0;
double z = 0.0;
double th = 0.0;

double vx = 0.0;
double vy = 0.0;
double vth = 0.0;


void opticalflowCallback(const optical_flow::OpticalFlow::ConstPtr& flow)
{
    // Filling out the velocities
    current_time = flow->header.stamp;
    double dt = (current_time - last_time).toSec();
    if (fabs(dt) > MAX_DELTA_TIME) {
        last_time = current_time;
        return; // This is for the first iteration when 'last_time' is undefined
    }

    vx = flow->velocity_x;
    vy = flow->velocity_y;
    z  = flow->ground_distance;

    double flow_accuracy = 1/(flow->quality + 0.001); // 0.001 is for not dividing by zero (0.004 - 1000)
    double delta_x = (vx * cos(th) - vy * sin(th)) * dt;
    double delta_y = (vx * sin(th) + vy * cos(th)) * dt;
    double delta_th = vth * dt;

    // FIXME:
    delta_x = flow->offset_x / 50.0;
    delta_y = flow->offset_y / 50.0;

    x += delta_x;
    y += delta_y;
    th += delta_th;

    //since all odometry is 6DOF we'll need a quaternion created from yaw
    geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(th);

    //first, we'll publish the transform over tf
    geometry_msgs::TransformStamped odom_trans;
    odom_trans.header.stamp = current_time;
    odom_trans.header.frame_id = "odom";
    odom_trans.child_frame_id  = "base_link";

    odom_trans.transform.translation.x = x;
    odom_trans.transform.translation.y = y;
    odom_trans.transform.translation.z = z;
    odom_trans.transform.rotation = odom_quat;

    //send the transform
    odom_broadcaster->sendTransform(odom_trans);


    //next, we'll publish the odometry message over ROS
    nav_msgs::Odometry odom;
    odom.header.stamp = current_time;
    odom.header.frame_id = "odom";

    //set the position
    odom.pose.pose.position.x = x;
    odom.pose.pose.position.y = y;
    odom.pose.pose.position.z = z;
    odom.pose.pose.orientation = odom_quat;

    //set the velocity
    odom.child_frame_id = "base_link";
    odom.twist.twist.linear.x = vx;
    odom.twist.twist.linear.y = vy;
    odom.twist.twist.angular.z = vth;

    odom.pose.covariance =  boost::assign::list_of(1e-3)  (0) (0)  (0)  (0)  (0)
                                                  (0) (1e-3)  (0)  (0)  (0)  (0)
                                                  (0)   (0)  (1e6) (0)  (0)  (0)
                                                  (0)   (0)   (0) (1e6) (0)  (0)
                                                  (0)   (0)   (0)  (0) (1e6) (0)
                                                  (0)   (0)   (0)  (0)  (0)  (1e3);

    odom.twist.covariance = boost::assign::list_of(1e-3)  (0) (0)  (0)  (0)  (0)
                                                  (0) (1e-3)  (0)  (0)  (0)  (0)
                                                  (0)   (0)  (1e6) (0)  (0)  (0)
                                                  (0)   (0)   (0) (1e6) (0)  (0)
                                                  (0)   (0)   (0)  (0) (1e6) (0)
                                                  (0)   (0)   (0)  (0)  (0)  (1e3);
    //publish the message
    odom_pub.publish(odom);
    last_time = current_time;

    ROS_INFO("dt: %1.6f, (%3.3f, %3.3f, %3.3f) ~ %5.3f", dt, x, y, z, flow_accuracy);
};


int main(int argc, char **argv)
{
    ros::init(argc, argv, "flow_to_odom_node");
    ros::NodeHandle n;


    odom_pub = n.advertise<nav_msgs::Odometry>("/to_ekf/vo", 50);
    flow_sub = n.subscribe("/optical_flow/opt_flow", 50, opticalflowCallback);
    odom_broadcaster = boost::shared_ptr<tf::TransformBroadcaster> (new tf::TransformBroadcaster);

    current_time = ros::Time::now();
    last_time    = ros::Time::now();

    ros::spin();


    return 0;
};
