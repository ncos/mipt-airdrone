#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <apm_imu_driver/Mavlink_RAW_IMU.h>
#include <boost/assign/list_of.hpp> // for 'list_of()'




ros::Subscriber apm_imu_sub;
ros::Publisher  imu_pub;



void imu_callback(const apm_imu_driver::Mavlink_RAW_IMU msg)
{
    sensor_msgs::Imu imu_msg;
    imu_msg.orientation_covariance         =  boost::assign::list_of(0.0012755)  (0) (0)
                                                                    (0) (0.0012755)  (0)
                                                                    (0) (0) (0.00250000);
    imu_msg.angular_velocity_covariance    =  boost::assign::list_of(0.0025)  (0) (0)
                                                                    (0) (0.0025)  (0)
                                                                    (0) (0) (0.000225);
    imu_msg.linear_acceleration_covariance =  boost::assign::list_of(0.1225)  (0) (0)
                                                                    (0) (0.1225)  (0)
                                                                    (0) (0) (0.09000);
    geometry_msgs::Quaternion orientation;
    imu_msg.orientation = orientation;

    imu_msg.linear_acceleration.x = msg.xacc;
    imu_msg.linear_acceleration.y = msg.yacc;
    imu_msg.linear_acceleration.z = msg.zacc;

    imu_msg.angular_velocity.x = msg.xgyro;
    imu_msg.angular_velocity.y = msg.ygyro;
    imu_msg.angular_velocity.z = msg.zgyro;

    imu_pub.publish(imu_msg);
};




int main( int argc, char** argv )
{
    ros::init(argc, argv, "apm_imu_driver");
    ros::NodeHandle nh;


    std::string input_topic_apm_imu = nh.resolveName("/apm/raw_imu");
    std::string output_topic_imu    = nh.resolveName("/raw_imu"  );



    apm_imu_sub = nh.subscribe<apm_imu_driver::Mavlink_RAW_IMU> (input_topic_apm_imu,  1, imu_callback);
    imu_pub     = nh.advertise<sensor_msgs::Imu> (output_topic_imu, 1 );


    ros::spin();

    return 0;
};
