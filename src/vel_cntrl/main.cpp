#include <stdio.h>
#include <iostream>
#include <vector>
#include <math.h>

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <visualization_msgs/Marker.h>
#include <std_srvs/Empty.h>
#include <vel_cntrl/RC.h>

#define TEST

using std::cout;
using std::cerr;
using std::endl;

ros::Subscriber sub_vel_1;
ros::Subscriber sub_vel_2;
ros::Subscriber sub_vel_3;
ros::Publisher  pub_vel;
ros::Publisher  pub_rc;
ros::Publisher  pub_mrk;

geometry_msgs::Twist vel_1;
geometry_msgs::Twist vel_2;
geometry_msgs::Twist vel_3;

visualization_msgs::Marker height_text;

double vel_to_pwr = 0.0;
vel_cntrl::RC rc_msg;
double rc_acc[8];



#define ROLL 0
#define PITCH 1
#define YAW 3
#define THROTTLE 2



void callback_1(const geometry_msgs::Twist vel)
{
    vel_1.linear.x = vel.linear.x;
    vel_1.linear.y = vel.linear.y;
    vel_1.linear.z = vel.linear.z;

    vel_1.angular.x = vel.angular.x;
    vel_1.angular.y = vel.angular.y;
    vel_1.angular.z = vel.angular.z;
};

void callback_2(const geometry_msgs::Twist vel)
{
    vel_2.linear.x = vel.linear.x;
    vel_2.linear.y = vel.linear.y;
    vel_2.linear.z = vel.linear.z;

    vel_2.angular.x = vel.angular.x;
    vel_2.angular.y = vel.angular.y;
    vel_2.angular.z = vel.angular.z;
};

void callback_3(const geometry_msgs::Twist vel)
{
    vel_3.linear.x = vel.linear.x;
    vel_3.linear.y = vel.linear.y;
    vel_3.linear.z = vel.linear.z;

    vel_3.angular.x = vel.angular.x;
    vel_3.angular.y = vel.angular.y;
    vel_3.angular.z = vel.angular.z;
};






void set_manual_control()
{
    for(int i = 0; i < 8; i++) rc_msg.channel.elems[i] = 0;
};



void set_all_min()
{
    rc_msg.channel.elems[THROTTLE] = 1000;
};






void vel_to_RC(geometry_msgs::Twist vel)
{
    rc_acc[THROTTLE] += vel_to_pwr * vel.linear.z;
    if(rc_acc[THROTTLE] > 2000) rc_acc[THROTTLE] = 2000;
    if(rc_acc[THROTTLE] < 1000) rc_acc[THROTTLE] = 1000;

    for(int i = 0; i < 8; i++)
        rc_msg.channel.elems[i] = rc_acc[i];
};


int main( int argc, char** argv )
{
  ros::init(argc, argv, "velocity_server");
  ros::NodeHandle nh;
  ros::Rate loop_rate(60);


  std::string input_topic_vel_1 = nh.resolveName("/cmd_vel_1");
  std::string input_topic_vel_2 = nh.resolveName("/cmd_vel_2");
  std::string input_topic_vel_3 = nh.resolveName("/cmd_vel_3");
  std::string output_topic_vel  = nh.resolveName("/cmd_vel"  );
  std::string output_topic_rc   = nh.resolveName("/send_rc"  );
  std::string output_topic_mrk = nh.resolveName("visualization_marker");


  sub_vel_1   = nh.subscribe<geometry_msgs::Twist > (input_topic_vel_1,  1, callback_1);
  sub_vel_2   = nh.subscribe<geometry_msgs::Twist > (input_topic_vel_2,  1, callback_2);
  sub_vel_3   = nh.subscribe<geometry_msgs::Twist > (input_topic_vel_3,  1, callback_3);

  pub_vel     = nh.advertise<geometry_msgs::Twist >           (output_topic_vel, 1 );
  pub_rc      = nh.advertise<vel_cntrl::RC        >           (output_topic_rc,  1 );
  pub_mrk     = nh.advertise<visualization_msgs::Marker>      (output_topic_mrk, 5 );


  if (!nh.getParam("vel_to_pwr", vel_to_pwr)) ROS_ERROR("Failed to get param 'vel_to_pwr'");

  height_text.header.frame_id = "/camera_link";
  height_text.ns = "text_ns";
  height_text.action = visualization_msgs::Marker::ADD;
  height_text.id = 101;
  height_text.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
  height_text.scale.z = 0.1;
  height_text.pose.position.z = 0.4;
  height_text.color.r = 0.0;
  height_text.color.g = 1.0;
  height_text.color.b = 0.0;
  height_text.color.a = 0.3;
  height_text.text    = "THROTTLE = [Nan]\nROLL = [Nan]\nPITCH = [Nan]\nYAW = [Nan]\n";
  pub_mrk.publish(height_text);


#ifndef TEST
  ros::service::waitForService("/arm");
#endif

  height_text.lifetime = ros::Duration(0.2);
  while (ros::ok())
  {
      geometry_msgs::Twist vel_acc;
      vel_acc.linear.x = vel_1.linear.x + vel_2.linear.x + vel_3.linear.x;
      vel_acc.linear.y = vel_1.linear.y + vel_2.linear.y + vel_3.linear.y;
      vel_acc.linear.z = vel_1.linear.z + vel_2.linear.z + vel_3.linear.z;

      vel_acc.angular.x = vel_1.angular.x + vel_2.angular.x + vel_3.angular.x;
      vel_acc.angular.y = vel_1.angular.y + vel_2.angular.y + vel_3.angular.y;
      vel_acc.angular.z = vel_1.angular.z + vel_2.angular.z + vel_3.angular.z;

      pub_vel.publish(vel_acc);


      vel_to_RC(vel_acc);
      pub_rc.publish(rc_msg);


      char text[32];
      sprintf(text, "THROTTLE = [%4.0f]\nROLL = [%4.0f]\nPITCH = [%4.0f]\nYAW = [%4.0f]\n", rc_acc[THROTTLE], rc_acc[ROLL], rc_acc[PITCH], rc_acc[YAW]);
      height_text.text = text;
      pub_mrk.publish(height_text);

      ros::spinOnce();
      loop_rate.sleep();
  }

  return 0;
}












