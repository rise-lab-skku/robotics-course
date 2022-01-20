#include <ros/ros.h>
#include <sensor_msgs/MagneticField.h>

void magCallback(const sensor_msgs::MagneticField::ConstPtr& msg)
{
  ROS_INFO("Mag data received");
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "imu_calibration");
    ros::NodeHandle nh;
    ros::AsyncSpinner spinner(2);  // Use 2 threads
    spinner.start();

    // Subscribe to magnetometer data
    ros::Subscriber mag_sub = nh.subscribe("/imu/mag", 128, &magCallback);

    ros::waitForShutdown();
    return 0;
}