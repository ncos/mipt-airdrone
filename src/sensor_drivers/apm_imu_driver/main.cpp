#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <apm_imu_driver/Mavlink_RAW_IMU.h>




ros::Subscriber apm_imu_sub;
ros::Publisher  imu_pub;



void imu_callback(const apm_imu_driver::Mavlink_RAW_IMU msg)
{
    sensor_msgs::Imu imu_msg;

    imu_pub.publish(imu_msg);
};




int main( int argc, char** argv )
{
    ros::init(argc, argv, "apm_imu_driver");
    ros::NodeHandle nh;


    std::string input_topic_apm_imu = nh.resolveName("/raw_imu");
    std::string output_topic_imu    = nh.resolveName("/to_ekf/imu_data"  );



    apm_imu_sub = nh.subscribe<apm_imu_driver::Mavlink_RAW_IMU> (input_topic_apm_imu,  1, imu_callback);
    imu_pub     = nh.advertise<sensor_msgs::Imu> (output_topic_imu, 1 );


    ros::spin();

    return 0;
};
