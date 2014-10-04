#include <vector>
#include <cmath>

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Vector3.h>
#include <visualization_msgs/Marker.h>
#include <std_srvs/Empty.h>
#include <vel_cntrl/RC.h>


ros::Subscriber sub_vel_cntrl;
ros::Subscriber sub_vel_kbrd;
ros::Publisher  pub_vel;
ros::Publisher  pub_rc;
ros::Publisher  pub_mrk;


std::string input_vel_cntrl_topic;
std::string input_vel_kbrd_topic;
std::string output_vel_topic;
std::string output_rc_topic;
std::string visualization_topic;
double vel_to_pwr;
double angle_of_kinect;
bool keyboard_control;


geometry_msgs::Twist vel_cntrl_;
geometry_msgs::Twist vel_kbrd_;
double rc_acc[8];
vel_cntrl::RC rc_msg;



#define ROLL 0
#define PITCH 1
#define YAW 3
#define THROTTLE 2


geometry_msgs::Vector3 rotate_z(const geometry_msgs::Vector3 linear_vel, double angle)
{
    geometry_msgs::Vector3 rotated;
    rotated.z = linear_vel.z;
    rotated.x = linear_vel.x * cos(angle * M_PI/180.0) - linear_vel.y * sin(angle * M_PI/180.0);
    rotated.y = linear_vel.x * sin(angle * M_PI/180.0) + linear_vel.y * cos(angle * M_PI/180.0);

    return rotated;
}




void callback_cntrl(const geometry_msgs::Twist vel)
{
    vel_cntrl_.linear = rotate_z(vel.linear, angle_of_kinect);
    vel_cntrl_.angular.x = vel.angular.x;
    vel_cntrl_.angular.y = vel.angular.y;
    vel_cntrl_.angular.z = vel.angular.z;
};

void callback_kbrd(const geometry_msgs::Twist vel)
{
    vel_kbrd_.linear = rotate_z(vel.linear, angle_of_kinect);
    vel_kbrd_.angular.x = vel.angular.x;
    vel_kbrd_.angular.y = vel.angular.y;
    vel_kbrd_.angular.z = vel.angular.z;
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


void state_to_rviz(bool inf = false)
{
    visualization_msgs::Marker height_text;
    height_text.header.frame_id = "/kinect_link";
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
    height_text.lifetime = ros::Duration(1000);
    height_text.text    = "THROTTLE = [Nan]\nROLL = [Nan]\nPITCH = [Nan]\nYAW = [Nan]\n";
    if (!inf) {
        char text[128];
        height_text.lifetime = ros::Duration(0.2);
        sprintf(text, "THROTTLE = [%4.0f]\nROLL = [%4.0f]\nPITCH = [%4.0f]\nYAW = [%4.0f]\n", rc_acc[THROTTLE], rc_acc[ROLL], rc_acc[PITCH], rc_acc[YAW]);
        height_text.text = text;
    }
    pub_mrk.publish(height_text);
}




int main( int argc, char** argv )
{
    ros::init(argc, argv, "velocity_server");
    ros::NodeHandle nh;
    ros::Rate loop_rate(60);

    if (!nh.getParam("velocity_server/input_vel_cntrl_topic", input_vel_cntrl_topic)) input_vel_cntrl_topic = "/cmd_vel_cntrl";
    if (!nh.getParam("velocity_server/input_vel_kbrd_topic",  input_vel_kbrd_topic)) input_vel_kbrd_topic = "/cmd_vel_keyboard";
    if (!nh.getParam("velocity_server/output_vel_topic", output_vel_topic)) output_vel_topic = "/cmd_vel";
    if (!nh.getParam("velocity_server/output_rc_topic", output_rc_topic)) output_rc_topic = "/apm/send_rc";
    if (!nh.getParam("velocity_server/visualization_topic", visualization_topic)) visualization_topic = "/velocity_server/markers";
    if (!nh.getParam("velocity_server/vel_to_pwr", vel_to_pwr)) vel_to_pwr = 0.0;
    if (!nh.getParam("velocity_server/angle_of_kinect", angle_of_kinect)) angle_of_kinect = 0.0;
    if (!nh.getParam("velocity_server/keyboard_control", keyboard_control)) keyboard_control = false;


    sub_vel_cntrl = nh.subscribe<geometry_msgs::Twist > (input_vel_cntrl_topic, 1, callback_cntrl);
    sub_vel_kbrd  = nh.subscribe<geometry_msgs::Twist > (input_vel_kbrd_topic,  1, callback_kbrd);
    pub_vel     = nh.advertise<geometry_msgs::Twist >           (output_vel_topic, 1 );
    pub_rc      = nh.advertise<vel_cntrl::RC        >           (output_rc_topic,  1 );
    pub_mrk     = nh.advertise<visualization_msgs::Marker>      (visualization_topic, 5 );

    state_to_rviz(true);
    //ros::service::waitForService("/arm");

    while (ros::ok())
    {
        geometry_msgs::Twist output_vel;
        if (keyboard_control == true) {
            output_vel.linear.x = vel_kbrd_.linear.x;
            output_vel.linear.y = vel_kbrd_.linear.y;
            output_vel.linear.z = vel_kbrd_.linear.z;

            output_vel.angular.x = vel_kbrd_.angular.x;
            output_vel.angular.y = vel_kbrd_.angular.y;
            output_vel.angular.z = vel_kbrd_.angular.z;
        }
        else {
            output_vel.linear.x = vel_cntrl_.linear.x;
            output_vel.linear.y = vel_cntrl_.linear.y;
            output_vel.linear.z = vel_cntrl_.linear.z;

            output_vel.angular.x = vel_cntrl_.angular.x;
            output_vel.angular.y = vel_cntrl_.angular.y;
            output_vel.angular.z = vel_cntrl_.angular.z;
        }

        vel_to_RC(output_vel);


        pub_vel.publish(output_vel);
        pub_rc.publish(rc_msg);
        state_to_rviz();

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
};












