#include <math.h>
#include <ros/ros.h>
#include <ros/package.h>
#include <yaml-cpp/yaml.h>
#include <geometry_msgs/Vector3.h>
#include <tf2/LinearMath/Quaternion.h>

// Global variables to make this code easier
geometry_msgs::Vector3 linear_accel;
geometry_msgs::Vector3 magnetic_field;

void imuCallback(const sensor_msgs::Imu::ConstPtr& msg)
{
    linear_accel = msg->linear_acceleration;
}

void magCallback(const sensor_msgs::MagneticField::ConstPtr& msg)
{
    magnetic_field = msg->magnetic_field;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "inertial_odom")
    ros::NodeHandle nh;
    ros::AsyncSpinner spinner(2);  // Use 2 threads
    spinner.start();
    ROS_INFO_STREAM("Node started. Waiting for /imu_raw and /mag_raw data.");

    // Subscribers
    ros::Subscriber imu_sub = nh.subscribe("/imu_raw", 128, &imuCallback);
    ros::Subscriber mag_sub = nh.subscribe("/mag_raw", 128, &magCallback);

    // Loop
    ros::Rate loop_rate(128);
    while (ros::ok())
    {
        // Quaternion from IMU accelerometer
        double ax = linear_accel.x;
        double ay = linear_accel.y;
        double azp1 = linear_accel.z + 1.0;
        tf2::Quaternion quat_acc(-ay/sqrt(2*azp1), ax/sqrt(2*azp1), 0.0, sqrt(azp1/2));
        // Find gamma
        
        // Quaternion from IMU magnetometer


        ros::spinOnce();
        loop_rate.sleep();
    }
    ros::waitForShutdown();
    return 0;
}