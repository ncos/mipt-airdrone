#include <stdio.h>
#include <iostream>
#include <vector>
#include <math.h>

#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/common_headers.h>
#include <geometry_msgs/Twist.h>

#include <kin.h>

typedef pcl::PointXYZRGB PointT;
typedef pcl::PointCloud<PointT> PointCloudT;


using std::cout;
using std::cerr;
using std::endl;

ros::Subscriber sub;
ros::Publisher  pub_vel;

KinectIN kin;


class PID
{
	double P, I, D;
	double prev_err;
	bool not_started;
	double t_diff;
	double integral;

public:
	PID(double _P, double _I, double _D)
	{
		P = _P; I = _I; D = _D;
		not_started = true;
		prev_err = 0;
		integral = 0;
		t_diff = 0.0333333; // 1/30 sec
	}

	void set_PID(double _P, double _I, double _D)
	{
		P = _P; I = _I; D = _D;
	}

	double get_output(double target_val, double current_val)
	{
		double err = target_val - current_val;

		if(not_started)
		{
			not_started = false;
			prev_err = err;
			return P * err;
		}
		integral += err;

		double P_val = P * err;
		double I_val = I * integral * t_diff;
		double D_val = D * (err - prev_err) / t_diff;

		prev_err = err;
		return P_val + I_val + D_val;
	}
};



void callback(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& floor_cloud)
{
	PointCloudN::Ptr mls_cloud = kin.filter(floor_cloud);
	pcl::ModelCoefficients::Ptr coeff = kin.find_plane(mls_cloud);
	//kin.position.distance_to_floor

	geometry_msgs::Twist base_cmd;
	PID pid_vel(1, 0.5, 0.5);

	base_cmd.linear.z = pid_vel.get_output(0.5, kin.position.distance_to_floor);
	//cerr << base_cmd.linear.z << "     " << kin.position.distance_to_floor << endl;

	pub_vel.publish(base_cmd);
};






int main (int argc, char** argv)
{
	ros::init (argc, argv, "thee_seeker_of_the_floor");

	ros::NodeHandle nh;
	std::string input_topic  = nh.resolveName("/shrinker/depth/floor_points");
	std::string output_topic = nh.resolveName("/cmd_vel_1");

	sub 	= nh.subscribe<pcl::PointCloud<pcl::PointXYZ> > (input_topic,  1, callback);
	pub_vel = nh.advertise<geometry_msgs::Twist> (output_topic, 1);


	ros::spin ();

	return 0;
}

