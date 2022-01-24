#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/LinearMath/Quaternion.h>

// Ground truth
geometry_msgs::PoseStamped gt_pose;

// Global variables to make this code easier
tf2::Quaternion quat(0, 0, 0, 1);
ros::Time prev_time;

void imuCallback(const sensor_msgs::Imu::ConstPtr& msg)
{
    // dt
    ros::Time current_time = ros::Time::now();
    double half_dt = (current_time - prev_time).toSec() / 2.0;
    prev_time = current_time;
    // previous quat
    double x = quat.x();
    double y = quat.y();
    double z = quat.z();
    double w = quat.w();
    // gyroscope
    double p = msg->angular_velocity.x *  10.0;
    double q = msg->angular_velocity.y *  10.0;
    double r = msg->angular_velocity.z *  10.0;
    // inertial odometry
    quat.setW(w + half_dt * ((p*x) + (q*y) + (r*z)));
    quat.setX(x - half_dt * ((p*w) + (-r*y) + (q*z)));
    quat.setY(y - half_dt * ((q*w) + (r*x) + (-p*z)));
    quat.setZ(z - half_dt * ((r*w) + (-q*x) + (p*y)));
    quat.normalize();
    ROS_INFO_STREAM("Mag data received. New quaternion: " << quat.x() << ", " << quat.y() << ", " << quat.z() << ", " << quat.w());
}

void gtCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    gt_pose.header.frame_id = "map";
    gt_pose.pose = msg->pose;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "gyroscope");
    ros::NodeHandle nh;
    ros::AsyncSpinner spinner(1);  // Use 1 threads
    spinner.start();
    prev_time = ros::Time::now();
    ROS_INFO_STREAM("Node started. Waiting for IMU data(orientation).");

    // Subscribe to magnetometer data
    ros::Subscriber mag_sub = nh.subscribe("/imu_raw", 128, &imuCallback);
    ros::Subscriber gt_sub = nh.subscribe("/vrpn_client_node/quad_imu_2/pose", 128, &gtCallback);

    // Visualization
    ros::Publisher odom_pub = nh.advertise<geometry_msgs::PoseStamped>("/odom_visualization", 128);
    ros::Publisher gt_pub = nh.advertise<geometry_msgs::PoseStamped>("/gt_visualization", 128);

    geometry_msgs::PoseStamped odom_pose;
    odom_pose.header.frame_id = "map";
    odom_pose.pose.position.x = 0.0;
    odom_pose.pose.position.y = 0.0;
    odom_pose.pose.position.z = 0.0;
    ros::Rate loop_rate(64);
    while (ros::ok())
    {
        odom_pose.pose.orientation = tf2::toMsg(quat);
        // Inverse quat
        odom_pose.pose.orientation.w = -odom_pose.pose.orientation.w;
        odom_pub.publish(odom_pose);

        gt_pub.publish(gt_pose);
        ros::spinOnce();
        loop_rate.sleep();
    }

    // Manually exit with Ctrl-C.
    ros::waitForShutdown();
    return 0;
}