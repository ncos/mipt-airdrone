#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>


class TfHandle
{
private:
    tf::TransformListener tf_listener;
    tf::TransformBroadcaster tf_broadcaster;

    void split_pose_ekf()
    {
        tf::StampedTransform transform_in;

        try {
            this->tf_listener.lookupTransform("/odom", "/ekf_result_frame", ros::Time(0), transform_in);
        }
        catch (tf::TransformException &ex) {
            ROS_ERROR("%s",ex.what());
            ros::Duration(1.0).sleep();
            return;
        }


        double roll, pitch, yaw;
        tf::Matrix3x3(transform_in.getRotation()).getRPY(roll, pitch, yaw);


        // broadcast
        // Examine Hector slam tutorial/How to set up hector_slam for your robot
        // (http://library.isr.ist.utl.pt/docs/roswiki/hector_slam(2f)Tutorials(2f)SettingUpForYourRobot.html)
        // the link may change with time
        // base_footprint
        tf::Transform transform_odom2base_footprint;
        transform_odom2base_footprint.setOrigin( tf::Vector3(transform_in.getOrigin().x(),
                                                             transform_in.getOrigin().y(),
                                                             0.0) );

        tf::Quaternion q = tf::createQuaternionFromRPY(0, 0, yaw);
        transform_odom2base_footprint.setRotation(q);
        this->tf_broadcaster.sendTransform(tf::StampedTransform(transform_odom2base_footprint, transform_in.stamp_, "odom", "base_footprint"));

        // base_stabilized
        tf::Transform transform_base_footprint2base_stabilized;
        transform_base_footprint2base_stabilized.setOrigin( tf::Vector3(0.0, 0.0, transform_in.getOrigin().z()) );

        transform_base_footprint2base_stabilized.setRotation(tf::createQuaternionFromRPY(0, 0, 0));
        this->tf_broadcaster.sendTransform(tf::StampedTransform(transform_base_footprint2base_stabilized, transform_in.stamp_,
                                                                                                         "base_footprint", "base_stabilized"));

        // base_link
        tf::Transform transform_base_stabilized2base_link;
        transform_base_stabilized2base_link.setOrigin( tf::Vector3(0.0, 0.0, 0.0) );
        transform_base_stabilized2base_link.setRotation(tf::createQuaternionFromRPY(roll, pitch, 0));
        this->tf_broadcaster.sendTransform(tf::StampedTransform(transform_base_stabilized2base_link, transform_in.stamp_,
                                                                                                              "base_stabilized", "base_link"));
    }

public:
    TfHandle()
    {
        this->tf_listener.waitForTransform("/odom", "/ekf_result_frame", ros::Time::now(), ros::Duration(3.0));
    }

    void spin()
    {
        this->split_pose_ekf();
    }
};



int main(int argc, char** argv)
{
    ros::init(argc, argv, "airdrine_tf_manager");
    ros::NodeHandle node;
    ros::Rate rate(30.0);
    TfHandle tf_handle;


    while (node.ok()) {
        tf_handle.spin();
        rate.sleep();
    }
    
    return 0;
};
