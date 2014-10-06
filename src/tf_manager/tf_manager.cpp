#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>


std::string input_pose_topic;
std::string fixed_frame;
std::string base_footprint_frame;
std::string base_stabilized_frame;
std::string base_link_frame;
int publish_rate;

class TfHandle
{
private:
    ros::Subscriber sub_ekf;
    tf::TransformBroadcaster tf_broadcaster;

    tf::Transform transform_odom2base_footprint;
    tf::Transform transform_base_footprint2base_stabilized;
    tf::Transform transform_base_stabilized2base_link;

    ros::Rate r;


    void split_pose_ekf(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg_in)
    {
        double roll, pitch, yaw;
        tf::Quaternion orient(msg_in->pose.pose.orientation.x, msg_in->pose.pose.orientation.y, msg_in->pose.pose.orientation.z, msg_in->pose.pose.orientation.w);
        orient.normalize();
        tf::Matrix3x3(orient).getRPY(roll, pitch, yaw);
        if (isnan(msg_in->pose.pose.orientation.x) || isnan(msg_in->pose.pose.orientation.y) || isnan(msg_in->pose.pose.orientation.z) ||isnan(msg_in->pose.pose.orientation.w)) {
		ROS_WARN("%f, %f, %f, %f", (msg_in->pose.pose.orientation.x), (msg_in->pose.pose.orientation.y), (msg_in->pose.pose.orientation.z), (msg_in->pose.pose.orientation.w));
	}

	if (isnan(roll) || isnan(pitch) || isnan(yaw)) {
		ROS_WARN("RPY failure!!! (%f, %f, %f)", roll, pitch, yaw);
	}
	if (isnan(roll)) {
	    roll = 0;;
	}
	if (isnan(pitch)) {
            pitch = 0;
	}
	if (isnan(yaw)) {
            yaw = 0;
	}


        // broadcast
        // Examine Hector slam tutorial/How to set up hector_slam for your robot
        // (http://library.isr.ist.utl.pt/docs/roswiki/hector_slam(2f)Tutorials(2f)SettingUpForYourRobot.html)
        // the link may change with time
        // base_footprint
        this->transform_odom2base_footprint.setOrigin( tf::Vector3(msg_in->pose.pose.position.x, msg_in->pose.pose.position.y, 0.0) );
        this->transform_odom2base_footprint.setRotation(tf::createQuaternionFromRPY(0, 0, yaw));


        // base_stabilized
        transform_base_footprint2base_stabilized.setOrigin( tf::Vector3(0.0, 0.0, msg_in->pose.pose.position.z) );
        transform_base_footprint2base_stabilized.setRotation(tf::createQuaternionFromRPY(0, 0, 0));


        // base_link
        transform_base_stabilized2base_link.setOrigin( tf::Vector3(0.0, 0.0, 0.0) );
        transform_base_stabilized2base_link.setRotation(tf::createQuaternionFromRPY(roll, pitch, 0));
    }

    void spin()
    {
        ros::spinOnce();

        this->tf_broadcaster.sendTransform(tf::StampedTransform(transform_odom2base_footprint, ros::Time::now(),
                                                                                                          fixed_frame, base_footprint_frame));
        this->tf_broadcaster.sendTransform(tf::StampedTransform(transform_base_footprint2base_stabilized, ros::Time::now(),
                                                                                                base_footprint_frame, base_stabilized_frame));
        this->tf_broadcaster.sendTransform(tf::StampedTransform(transform_base_stabilized2base_link, ros::Time::now(),
                                                                                                     base_stabilized_frame, base_link_frame));
        this->r.sleep();
    }

public:
    TfHandle(ros::NodeHandle nh): r(publish_rate)
    {
        this->transform_odom2base_footprint.setIdentity();
        this->transform_base_footprint2base_stabilized.setIdentity();
        this->transform_base_stabilized2base_link.setIdentity();

        this->sub_ekf = nh.subscribe<geometry_msgs::PoseWithCovarianceStamped> (input_pose_topic, 1, &TfHandle::split_pose_ekf, this);
        while (ros::ok()) {
            this->spin();
        }
    }
};



int main(int argc, char** argv)
{
    ros::init(argc, argv, "airdrine_tf_manager");
    ros::NodeHandle node;

    if (!node.getParam("tf_manager/input_pose_topic", input_pose_topic)) input_pose_topic = "/robot_pose_ekf/odom";
    if (!node.getParam("tf_manager/fixed_frame", fixed_frame)) fixed_frame = "/odom";
    if (!node.getParam("tf_manager/base_footprint_frame", base_footprint_frame)) base_footprint_frame = "/base_footprint";
    if (!node.getParam("tf_manager/base_stabilized_frame", base_stabilized_frame)) base_stabilized_frame = "/base_stabilized";
    if (!node.getParam("tf_manager/base_link_frame", base_link_frame)) base_link_frame = "/base_link";
    if (!node.getParam("tf_manager/publish_rate", publish_rate)) publish_rate = 50;

    TfHandle tf_handle(node);

    ros::spin();
    return 0;
};
