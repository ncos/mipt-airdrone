#include <vector>
#include <cmath>

#include <ros/ros.h>
#include <tf/transform_listener.h>
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
std::string input_velocity_frame;
std::string output_velocity_frame;

double vel_to_pwr;
bool in_simulator;
bool keyboard_control;


geometry_msgs::Twist vel_cntrl_;
geometry_msgs::Twist vel_kbrd_;
double rc_acc[8];
vel_cntrl::RC rc_msg;

#define ROLL 0
#define PITCH 1
#define YAW 3
#define THROTTLE 2


class TransformManager
{
private:
    ros::NodeHandle nh;
    tf::TransformListener tf_listener;

public:
    TransformManager(ros::NodeHandle nh_) {
        this->nh = nh_;
        try { this->tf_listener.waitForTransform(input_velocity_frame, output_velocity_frame, ros::Time(0), ros::Duration(10.0) ); }
        catch (tf::TransformException &ex) { ROS_ERROR("[velocity_server]: (wait) %s", ex.what()); }
    }

    geometry_msgs::Vector3 kin_to_base(const geometry_msgs::Vector3 linear_vel)
    {
        geometry_msgs::Vector3 rotated;
        geometry_msgs::PointStamped vector_original_start, vector_original_end;
        geometry_msgs::PointStamped vector_transformed_start, vector_transformed_end;
        vector_original_end.header.frame_id = input_velocity_frame;
        vector_original_end.header.stamp = ros::Time();
        vector_original_end.point.x = linear_vel.x;
        vector_original_end.point.y = linear_vel.y;
        vector_original_end.point.z = linear_vel.z;
        vector_original_start.header.frame_id = input_velocity_frame;
        vector_original_start.header.stamp = ros::Time();
        vector_original_start.point.x = 0;
        vector_original_start.point.y = 0;
        vector_original_start.point.z = 0;
        try {
           this->tf_listener.transformPoint(output_velocity_frame, vector_original_start, vector_transformed_start);
           this->tf_listener.transformPoint(output_velocity_frame, vector_original_end,   vector_transformed_end);
        }
        catch(tf::TransformException& ex) {
            ROS_ERROR("[velocity_server]: Received an exception trying to transform \
                       a point from \"%s\" to \"%s\": %s", input_velocity_frame.c_str(), output_velocity_frame.c_str(), ex.what());
        }

        rotated.x = vector_transformed_end.point.x - vector_transformed_start.point.x;
        rotated.y = vector_transformed_end.point.y - vector_transformed_start.point.y;
        rotated.z = vector_transformed_end.point.z - vector_transformed_start.point.z;
        return rotated;
    }
};

boost::shared_ptr<TransformManager> transform_manager;



void callback_cntrl(const geometry_msgs::Twist vel)
{
    vel_cntrl_.linear = transform_manager->kin_to_base(vel.linear);
    vel_cntrl_.angular.x = vel.angular.x;
    vel_cntrl_.angular.y = vel.angular.y;
    vel_cntrl_.angular.z = vel.angular.z;
};

void callback_kbrd(const geometry_msgs::Twist vel)
{
    vel_kbrd_.linear = transform_manager->kin_to_base(vel.linear);
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
};




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
    if (!nh.getParam("velocity_server/input_velocity_frame",  input_velocity_frame)) input_velocity_frame = "/kinect_link";
    if (!nh.getParam("velocity_server/output_velocity_frame", output_velocity_frame)) output_velocity_frame  = "/base_link";
    if (!nh.getParam("/keyboard_control", keyboard_control)) keyboard_control = false;


    transform_manager = boost::shared_ptr<TransformManager> (new TransformManager(nh));


    sub_vel_cntrl = nh.subscribe<geometry_msgs::Twist > (input_vel_cntrl_topic, 1, callback_cntrl);
    sub_vel_kbrd  = nh.subscribe<geometry_msgs::Twist > (input_vel_kbrd_topic,  1, callback_kbrd);
    pub_vel     = nh.advertise<geometry_msgs::Twist >      (output_vel_topic,    1 );
    pub_rc      = nh.advertise<vel_cntrl::RC        >      (output_rc_topic,     1 );
    pub_mrk     = nh.advertise<visualization_msgs::Marker> (visualization_topic, 5 );

    state_to_rviz(true);

    if (!nh.getParam("/in_simulator", in_simulator)) {
        ROS_ERROR("[velocity_server]: the 'in_simulator' boolean parameter must be specified!");
        ros::shutdown();
    }

    if (!in_simulator) {
        ROS_INFO("[velocity_server]: Waiting for apm to be ready...");
        ros::service::waitForService("/arm");
        ROS_INFO("[velocity_server]: ...Success!");
    }

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












