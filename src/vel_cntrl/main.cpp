#include <stdio.h>
#include <iostream>
#include <vector>
#include <math.h>

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <vel_cntrl/RC.h>




using std::cout;
using std::cerr;
using std::endl;

ros::Subscriber sub_vel_1;
ros::Subscriber sub_vel_2;
ros::Subscriber sub_vel_3;
ros::Publisher  pub_vel;
ros::Publisher  pub_rc;

geometry_msgs::Twist vel_1;
geometry_msgs::Twist vel_2;
geometry_msgs::Twist vel_3;

double vel_to_pwr = 0.0;



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



int main( int argc, char** argv )
{
  ros::init(argc, argv, "velocity_server");
  ros::NodeHandle nh;
  ros::Rate loop_rate(60);


  std::string input_topic_vel_1 = nh.resolveName("/cmd_vel_1");
  std::string input_topic_vel_2 = nh.resolveName("/cmd_vel_2");
  std::string input_topic_vel_3 = nh.resolveName("/cmd_vel_3");
  std::string output_topic_vel  = nh.resolveName("/cmd_vel");
  std::string output_topic_rc   = nh.resolveName("/send_rc");


  sub_vel_1 = nh.subscribe<geometry_msgs::Twist > (input_topic_vel_1,  1, callback_1);
  sub_vel_2 = nh.subscribe<geometry_msgs::Twist > (input_topic_vel_2,  1, callback_2);
  sub_vel_3 = nh.subscribe<geometry_msgs::Twist > (input_topic_vel_3,  1, callback_3);


  pub_vel   = nh.advertise<geometry_msgs::Twist >           (output_topic_vel, 1 );
  pub_rc    = nh.advertise<vel_cntrl::RC        >           (output_topic_rc,  1 );


  if (!nh.getParam("vel_to_pwr", vel_to_pwr)) ROS_ERROR("Failed to get param 'vel_to_pwr'");



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

	  vel_cntrl::RC rc_msg;

	  rc_msg.channel.elems[0] = 2000;
	  rc_msg.channel.elems[1] = 2000;
	  rc_msg.channel.elems[2] = 2000;

	  pub_rc.publish(rc_msg);

	  ros::spinOnce();
	  loop_rate.sleep();
  }

  return 0;
}












