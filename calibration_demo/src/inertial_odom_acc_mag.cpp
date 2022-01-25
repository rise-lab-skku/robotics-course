#include <fstream>
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
geometry_msgs::Vector3 magnetic_raw;
geometry_msgs::PoseStamped gt_pose;
ros::Time initial_magtime;
bool initial_magtime_set = false;

// Magnetometer calibration data structure
struct OneDimLimit
{
    double min, max;
};

struct MagCalibrationData
{
    OneDimLimit x;
    OneDimLimit y;
    OneDimLimit z;
};

void operator >> (const YAML::Node& node, OneDimLimit& limit)
{
    limit.min = node["min"].as<double>();
    limit.max = node["max"].as<double>();
}

void operator >> (const YAML::Node& node, MagCalibrationData& data)
{
    node["x"] >> data.x;
    node["y"] >> data.y;
    node["z"] >> data.z;
}

void magnetometerCalibration(
    const MagCalibrationData& cfg,
    const geometry_msgs::Vector3& raw,
    geometry_msgs::Vector3& calibrated)
{
    double sumx_half = (cfg.x.max + cfg.x.min) / 2.0;
    double sumy_half = (cfg.y.max + cfg.y.min) / 2.0;
    double sumz_half = (cfg.z.max + cfg.z.min) / 2.0;
    double dx_half = (cfg.x.max - cfg.x.min) / 2.0;
    double dy_half = (cfg.y.max - cfg.y.min) / 2.0;
    double dz_half = (cfg.z.max - cfg.z.min) / 2.0;
    double k = (dx_half + dy_half + dz_half) / 3.0;
    calibrated.x = (raw.x - sumx_half) * k / dx_half;
    calibrated.y = (raw.y - sumy_half) * k / dy_half;
    calibrated.z = (raw.z - sumz_half) * k / dz_half;
}

bool areQuaternionsClose(const tf2::Quaternion& q1, const tf2::Quaternion& q2)
{
    double dot = q1.x() * q2.x() + q1.y() * q2.y() + q1.z() * q2.z() + q1.w() * q2.w();
    return (dot >= 0.0);
}

tf2::Quaternion inverseSignQuaternion(const tf2::Quaternion& q)
{
    return tf2::Quaternion(-q.x(), -q.y(), -q.z(), -q.w());
}

void recursiveAverageQuat(const tf2::Quaternion& new_quat, tf2::Quaternion& avg_quat, int& count)
{
    if (count > 0)
    {
        tf2::Quaternion old_quat = avg_quat;
        tf2::Quaternion close_quat = (areQuaternionsClose(new_quat, old_quat)) ?
            new_quat : inverseSignQuaternion(new_quat);
        double k_1 = static_cast<double>(count - 1);
        double b = 1.0 / static_cast<double>(count);
        double a = k_1 * b;
        avg_quat = tf2::Quaternion(
            a * old_quat.x() + b * close_quat.x(),
            a * old_quat.y() + b * close_quat.y(),
            a * old_quat.z() + b * close_quat.z(),
            a * old_quat.w() + b * close_quat.w());
        avg_quat.normalize();
    }
    else
    {
        avg_quat = new_quat;
    }
    count++;
}

// ROS callback functions
void imuCallback(const sensor_msgs::Imu::ConstPtr& msg)
{
    linear_accel = msg->linear_acceleration;
}

void magCallback(const sensor_msgs::MagneticField::ConstPtr& msg)
{
    magnetic_raw = msg->magnetic_field;
    if (!initial_magtime_set)
    {
        initial_magtime = ros::Time::now();
        initial_magtime_set = true;
    }
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
    const std::string package_path = ros::package::getPath("calibration_demo");
    const std::string file_path = package_path + "/config/magnetometer_calibration.yaml";
    YAML::Node yaml_data = YAML::LoadFile(file_path);
    MagCalibrationData mag_cfg;
    yaml_data >> mag_cfg;
    ROS_INFO_STREAM("mag_cfg: \n"
        << "\tx: " << mag_cfg.x.min << " ~ " << mag_cfg.x.max << "\n"
        << "\ty: " << mag_cfg.y.min << " ~ " << mag_cfg.y.max << "\n"
        << "\tz: " << mag_cfg.z.min << " ~ " << mag_cfg.z.max);

    // Subscribers
    ros::Subscriber imu_sub = nh.subscribe("/imu_raw", 128, &imuCallback);
    ros::Subscriber mag_sub = nh.subscribe("/mag_raw", 128, &magCallback);
    ros::Subscriber gt_sub = nh.subscribe("/vrpn_client_node/quad_imu_2/pose", 128, &gtCallback);

    // Visualization
    ros::Publisher odom_pub = nh.advertise<geometry_msgs::PoseStamped>("/odom_visualization", 128);
    ros::Publisher gt_pub = nh.advertise<geometry_msgs::PoseStamped>("/gt_visualization", 128);

    // Local data
    geometry_msgs::Vector3 magnetic_calibrated;
    geometry_msgs::PoseStamped odom_pose;
    odom_pose.header.frame_id = "map";
    odom_pose.pose.position.x = 0.0;
    odom_pose.pose.position.y = 0.0;
    odom_pose.pose.position.z = 0.0;

    // Threshold of initial time to find workspace orientation
    const ros::Duration initial_time_threshold(2.5);
    tf2::Quaternion initial_avg_quat(0.0, 0.0, 0.0, 1.0);
    int data_count = 0;

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
        magnetometerCalibration(mag_cfg, magnetic_raw, magnetic_calibrated);
        tf2::Vector3 vec_mag(magnetic_calibrated.y, magnetic_calibrated.x, -magnetic_calibrated.z);
        tf2::Matrix3x3 mat_acc(quat_acc);
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

        if (initial_magtime_set)
        {
            if (ros::Time::now() > (initial_magtime + initial_time_threshold))
            {

                // Earth Orientation to Workspace Orientation
                tf2::Quaternion quat_from_ws = quat_est * initial_avg_quat.inverse();
                // Inverse quat
                quat_from_ws.setW(-quat_from_ws.w());
                // Visualization
                odom_pose.pose.orientation = tf2::toMsg(quat_from_ws);
                odom_pub.publish(odom_pose);
                gt_pub.publish(gt_pose);
                ROS_INFO_STREAM("odom quat: "
                    << quat_from_ws.x() << ", "
                    << quat_from_ws.y() << ", "
                    << quat_from_ws.z() << ", "
                    << quat_from_ws.w());
            }
            else
            {
                // Find Earth Orientation to Workspace Orientation
                recursiveAverageQuat(quat_est, initial_avg_quat, data_count);
                ROS_WARN_STREAM("Initial time NOT reached. AVG(quat): "
                    << initial_avg_quat.x() << ", "
                    << initial_avg_quat.y() << ", "
                    << initial_avg_quat.z() << ", "
                    << initial_avg_quat.w());
            }
        }
        else
        {
            ROS_INFO_STREAM("No magnetometer received. Waiting for mag data.");
            ros::Duration(0.1).sleep();
        }
        ros::spinOnce();
        loop_rate.sleep();
    }
    ros::waitForShutdown();
    return 0;
}