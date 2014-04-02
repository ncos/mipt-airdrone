#include <stdio.h>
#include <iostream>
#include <vector>
#include <math.h>

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <std_srvs/Empty.h>
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
	rc_msg.channel.elems[2] = 1000;
};


void arm()
{
	std_srvs::Empty empty;

	if (!ros::service::call("/arm", empty))
	{
		ROS_ERROR("Unable to call /arm service");
	}
};

void disarm()
{
	std_srvs::Empty empty;

	if (!ros::service::call("/disarm", empty))
	{
		ROS_ERROR("Unable to call /disarm service");
	}
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
  std::string output_topic_vel  = nh.resolveName("/cmd_vel");
  std::string output_topic_rc   = nh.resolveName("/send_rc");


  sub_vel_1 = nh.subscribe<geometry_msgs::Twist > (input_topic_vel_1,  1, callback_1);
  sub_vel_2 = nh.subscribe<geometry_msgs::Twist > (input_topic_vel_2,  1, callback_2);
  sub_vel_3 = nh.subscribe<geometry_msgs::Twist > (input_topic_vel_3,  1, callback_3);


  pub_vel   = nh.advertise<geometry_msgs::Twist >           (output_topic_vel, 1 );
  pub_rc    = nh.advertise<vel_cntrl::RC        >           (output_topic_rc,  1 );


  if (!nh.getParam("vel_to_pwr", vel_to_pwr)) ROS_ERROR("Failed to get param 'vel_to_pwr'");
  set_all_min();

  ros::service::waitForService("/disarm");
  ros::service::waitForService("/arm");
  arm();

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

	  ros::spinOnce();
	  loop_rate.sleep();
  }

  return 0;
}












