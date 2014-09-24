#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>


std::string input_pose_topic;
std::string fixed_frame;
std::string base_footprint_frame;
std::string base_stabilized_frame;
std::string base_link_frame;


class TfHandle
{
private:
    ros::Subscriber sub_ekf;
    tf::TransformBroadcaster tf_broadcaster;

    void split_pose_ekf(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg_in)
    {
        double roll, pitch, yaw;
        tf::Quaternion orient(msg_in->pose.pose.orientation.x, msg_in->pose.pose.orientation.y, msg_in->pose.pose.orientation.z, msg_in->pose.pose.orientation.w);
        tf::Matrix3x3(orient).getRPY(roll, pitch, yaw);

        // broadcast
        // Examine Hector slam tutorial/How to set up hector_slam for your robot
        // (http://library.isr.ist.utl.pt/docs/roswiki/hector_slam(2f)Tutorials(2f)SettingUpForYourRobot.html)
        // the link may change with time
        // base_footprint
        tf::Transform transform_odom2base_footprint;
        transform_odom2base_footprint.setOrigin( tf::Vector3(msg_in->pose.pose.position.x,
                                                             msg_in->pose.pose.position.y,
                                                             0.0) );

        tf::Quaternion q = tf::createQuaternionFromRPY(0, 0, yaw);
        transform_odom2base_footprint.setRotation(q);
        this->tf_broadcaster.sendTransform(tf::StampedTransform(transform_odom2base_footprint, msg_in->header.stamp,
                                                                                                  fixed_frame, base_footprint_frame));

        // base_stabilized
        tf::Transform transform_base_footprint2base_stabilized;
        transform_base_footprint2base_stabilized.setOrigin( tf::Vector3(0.0, 0.0, msg_in->pose.pose.position.z) );

        transform_base_footprint2base_stabilized.setRotation(tf::createQuaternionFromRPY(0, 0, 0));
        this->tf_broadcaster.sendTransform(tf::StampedTransform(transform_base_footprint2base_stabilized, msg_in->header.stamp,
                                                                                        base_footprint_frame, base_stabilized_frame));

        // base_link
        tf::Transform transform_base_stabilized2base_link;
        transform_base_stabilized2base_link.setOrigin( tf::Vector3(0.0, 0.0, 0.0) );
        transform_base_stabilized2base_link.setRotation(tf::createQuaternionFromRPY(roll, pitch, 0));
        this->tf_broadcaster.sendTransform(tf::StampedTransform(transform_base_stabilized2base_link, msg_in->header.stamp,
                                                                                             base_stabilized_frame, base_link_frame));
    }

public:
    TfHandle(ros::NodeHandle nh)
    {
        this->sub_ekf = nh.subscribe<geometry_msgs::PoseWithCovarianceStamped> (input_pose_topic, 1, &TfHandle::split_pose_ekf, this);
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

    TfHandle tf_handle(node);
    ros::spin();
    
    return 0;
};
