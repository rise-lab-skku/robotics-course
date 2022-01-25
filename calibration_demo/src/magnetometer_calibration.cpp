#include <limits>
#include <fstream>
#include <yaml-cpp/yaml.h>
#include <ros/ros.h>
#include <ros/package.h>
#include <sensor_msgs/MagneticField.h>

// Global variables to make this code easier
double x_min = std::numeric_limits<double>::max(), y_min = x_min, z_min = x_min;
double x_max = std::numeric_limits<double>::max() * -1.0, y_max = x_max, z_max = x_max;

void magCallback(const sensor_msgs::MagneticField::ConstPtr& msg)
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
    ros::init(argc, argv, "magnetometer_calibration");
    ros::NodeHandle nh;
    ros::AsyncSpinner spinner(1);  // Use 1 threads
    spinner.start();
    ROS_INFO_STREAM("Node started. Waiting for mag data.");

    // Subscribe to magnetometer data
    ros::Subscriber mag_sub = nh.subscribe("/mag_raw", 128, &magCallback);

    // Manually exit with Ctrl-C after `rosbag play` is complete.
    ros::waitForShutdown();
    std::cout << "==================" << std::endl;
    std::cout << "ROS was shut down." << std::endl;

    // Save calibration data to file
    const std::string package_path = ros::package::getPath("calibration_demo");
    const std::string file_path = package_path + "/config/magnetometer_calibration.yaml";
    std::ofstream calibration_file;
    calibration_file.open(file_path);

    // YAML::Emitter Tutorial
    // https://github.com/jbeder/yaml-cpp/wiki/How-To-Emit-YAML
    YAML::Emitter out;
    out.SetIndent(2);
    out << YAML::BeginMap;
    out << YAML::Key << "x";
    out << YAML::Value << YAML::BeginMap
        << YAML::Key << "min" << YAML::Value << x_min
        << YAML::Key << "max" << YAML::Value << x_max
        << YAML::EndMap;
    out << YAML::Key << "y";
    out << YAML::Value << YAML::BeginMap
        << YAML::Key << "min" << YAML::Value << y_min
        << YAML::Key << "max" << YAML::Value << y_max
        << YAML::EndMap;
    out << YAML::Key << "z";
    out << YAML::Value << YAML::BeginMap
        << YAML::Key << "min" << YAML::Value << z_min
        << YAML::Key << "max" << YAML::Value << z_max
        << YAML::EndMap;
    out << YAML::EndMap;

    calibration_file.write(out.c_str(), out.size());
    calibration_file.close();
    std::cout << "Calibration file saved at:" << std::endl;
    std::cout << "[ " << file_path << " ]" << std::endl;
    return 0;
}