#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/Quaternion.h>

// Global variables to make this code easier
geometry_msgs::Quaternion quat;
quat.x = 0.0;
quat.y = 0.0;
quat.z = 0.0;
quat.w = 1.0;

void imuCallback(const sensor_msgs::Imu::ConstPtr& msg)
{
    ROS_INFO_STREAM("Mag data received."
                    << " x: " << msg->magnetic_field.x
                    << " y: " << msg->magnetic_field.y
                    << " z: " << msg->magnetic_field.z);
    ROS_INFO_STREAM("Min XYZ: " << x_min << ", " << y_min << ", " << z_min);
    ROS_INFO_STREAM("Max XYZ: " << x_max << ", " << y_max << ", " << z_max << "\n");
    if (msg->magnetic_field.x < x_min) {x_min = msg->magnetic_field.x;}
    if (msg->magnetic_field.y < y_min) {y_min = msg->magnetic_field.y;}
    if (msg->magnetic_field.z < z_min) {z_min = msg->magnetic_field.z;}
    if (msg->magnetic_field.x > x_max) {x_max = msg->magnetic_field.x;}
    if (msg->magnetic_field.y > y_max) {y_max = msg->magnetic_field.y;}
    if (msg->magnetic_field.z > z_max) {z_max = msg->magnetic_field.z;}
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "gyroscope");
    ros::NodeHandle nh;
    ros::AsyncSpinner spinner(1);  // Use 1 threads
    spinner.start();
    ROS_INFO_STREAM("Node started. Waiting for IMU data(orientation).");

    // Subscribe to magnetometer data
    ros::Subscriber mag_sub = nh.subscribe("/imu_raw", 128, &imuCallback);

    // Manually exit with Ctrl-C.
    ros::waitForShutdown();
    return 0;
}