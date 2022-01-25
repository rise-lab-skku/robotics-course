#include <math.h>
#include <ros/ros.h>
#include <ros/package.h>
#include <yaml-cpp/yaml.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/MagneticField.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Vector3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Vector3.h>

// Global variables to make this code easier
geometry_msgs::Vector3 linear_accel;
geometry_msgs::Vector3 magnetic_field;
geometry_msgs::PoseStamped gt_pose;

void imuCallback(const sensor_msgs::Imu::ConstPtr& msg)
{
    linear_accel = msg->linear_acceleration;
}

void magCallback(const sensor_msgs::MagneticField::ConstPtr& msg)
{
    magnetic_field = msg->magnetic_field;
}

void gtCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    gt_pose.header.frame_id = "map";
    gt_pose.pose = msg->pose;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "inertial_odom");
    ros::NodeHandle nh;
    ros::AsyncSpinner spinner(2);  // Use 2 threads
    spinner.start();
    ROS_INFO_STREAM("Node started. Waiting for /imu_raw and /mag_raw data.");

    // Magnetometer calibration
    std::string package_path = ros::package::getPath("calibration_demo");
    std::string file_path = package_path + "/config/magnetometer_calibration.yaml";
    YAML::Node cfg = YAML::LoadFile(file_path);
    const double x_min = cfg["x"]["min"].as<double>();
    const double x_max = cfg["x"]["max"].as<double>();
    const double y_min = cfg["y"]["min"].as<double>();
    const double y_max = cfg["y"]["max"].as<double>();
    const double z_min = cfg["z"]["min"].as<double>();
    const double z_max = cfg["z"]["max"].as<double>();


    // Subscribers
    ros::Subscriber imu_sub = nh.subscribe("/imu_raw", 128, &imuCallback);
    ros::Subscriber mag_sub = nh.subscribe("/mag_raw", 128, &magCallback);
    ros::Subscriber gt_sub = nh.subscribe("/vrpn_client_node/quad_imu_2/pose", 128, &gtCallback);

    // Visualization
    ros::Publisher odom_pub = nh.advertise<geometry_msgs::Vector3>("/odom_visualization", 128);
    ros::Publisher gt_pub = nh.advertise<geometry_msgs::Vector3>("/gt_visualization", 128);

    geometry_msgs::PoseStamped odom_pose;
    odom_pose.header.frame_id = "map";
    odom_pose.pose.position.x = 0.0;
    odom_pose.pose.position.y = 0.0;
    odom_pose.pose.position.z = 0.0;
    // Loop
    ros::Rate loop_rate(128);
    while (ros::ok())
    {
        // Quaternion from IMU accelerometer
        double azp1 = linear_accel.z + 1.0;
        tf2::Quaternion quat_acc(
            -linear_accel.y / sqrt(2 * azp1),
            linear_accel.x / sqrt(2 * azp1),
            0.0,
            sqrt(azp1 / 2)
        );
        // Find gamma
        tf2::Matrix3x3 mat_acc(quat_acc);
        tf2::Vector3 vec_mag(magnetic_field.y, magnetic_field.z, -magnetic_field.z);
        tf2::Vector3 vec_mag_rot = mat_acc.transpose() * vec_mag;
        double gamma = pow(vec_mag_rot.x(), 2) + pow(vec_mag_rot.y(), 2);
        // Quaternion from IMU magnetometer
        bool lx_gt_0 = vec_mag_rot.x() >= 0.0;
        double lx_m_sqrtGamma = vec_mag_rot.x() * sqrt(gamma);
        double lx_d_sqrtGamma = vec_mag_rot.x() / sqrt(gamma);
        tf2::Quaternion quat_mag(
            0.0,
            0.0,
            lx_gt_0 ? vec_mag_rot.y() / sqrt(2.0 * (gamma + lx_m_sqrtGamma)) : sqrt(0.5 * (1.0 - lx_d_sqrtGamma)),
            lx_gt_0 ? sqrt(0.5 * (1.0 + lx_d_sqrtGamma)) : vec_mag_rot.y() / sqrt(2.0 * (gamma - lx_m_sqrtGamma))
        );
        // Estimated Quaternion (Hamilton product)
        tf2::Quaternion quat_est = quat_acc * quat_mag;
        // tf2::Quaternion quat_est(  // This brings the same result.
        //     quat_acc.w() * quat_mag.x() + quat_acc.x() * quat_mag.w() + quat_acc.y() * quat_mag.z() - quat_acc.z() * quat_mag.y(),
        //     quat_acc.w() * quat_mag.y() - quat_acc.x() * quat_mag.z() + quat_acc.y() * quat_mag.w() + quat_acc.z() * quat_mag.x(),
        //     quat_acc.w() * quat_mag.z() + quat_acc.x() * quat_mag.y() - quat_acc.y() * quat_mag.x() + quat_acc.z() * quat_mag.w(),
        //     quat_acc.w() * quat_mag.w() - quat_acc.x() * quat_mag.x() - quat_acc.y() * quat_mag.y() - quat_acc.z() * quat_mag.z()
        // );

        //

        // Visualization
        odom_pose.pose.orientation = tf2::toMsg(quat_est);

        // TODO: mag calibration
        // TODO: 2sec average delay due to polar.


        ros::spinOnce();
        loop_rate.sleep();
    }
    ros::waitForShutdown();
    return 0;
}