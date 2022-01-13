#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/Odometry.h>
#include "turtle_control/kf.h"

class KalmanFilter1D
{
public:
    KalmanFilter1D(float x, float p) : x_(x), p_(p)
    {
    }
    void Init(ros::NodeHandle &nh)
    {
        sub_scan_ = nh.subscribe("/scan", 3, &KalmanFilter1D::LaserScanCallback, this);
        sub_odom_ = nh.subscribe("/odom", 3, &KalmanFilter1D::OdometryCallback, this);
        pub_kf_ = nh.advertise<turtle_control::kf>("kf", 3);
        prev_prediction_ = ros::Time(0);
    }
    ~KalmanFilter1D() {}
    // update from odometer
    void Predict(float u, float q)
    {

        // x = Fx + Bu
        // F == 1, B== 1
        x_ = x_ + u;
        // P = FPF^T + Q
        // F == 1
        p_ = p_ + q;
    }
    // predict from laser scan
    void Update(float z, float r)
    {
        // kalman gain
        float K = 0;

        if (p_ != 0)
        {
            K = p_ / (p_ + r);
        }

        // state
        x_ = x_ + K * (z - x_);

        // variance
        p_ = (1 - K) * p_;
    }

    void OdometryCallback(const nav_msgs::Odometry::ConstPtr &msg)
    {
        if (prev_prediction_ == ros::Time(0))
        {
            prev_prediction_ = msg->header.stamp;
            return;
        }
        const float &dt = (msg->header.stamp - prev_prediction_).toSec();

        float u = msg->twist.twist.linear.x * dt;
        float q = 0.0001;

        Predict(u, q);
        prev_prediction_ = msg->header.stamp;
    }

    void LaserScanCallback(const sensor_msgs::LaserScan::ConstPtr &msg)
    {
        float ang = msg->angle_min;

        const float &roi_x_min = 0.10;
        const float &roi_x_max = 1.0;
        const float &roi_y_min = -0.40;
        const float &roi_y_max = 0.40;

        float x_sum = 0;
        float x_sq_sum = 0;
        unsigned int count = 0;
        for (auto &r : msg->ranges)
        {

            const float &x = r * std::cos(ang);
            const float &y = r * std::sin(ang);
            if (x < roi_x_min || x > roi_x_max || y < roi_y_min || y > roi_y_max)
            {
                continue;
            }
            x_sum += x;
            x_sq_sum += x;
            count += 1;
            ang += msg->angle_increment;
        }
        if (count == 0)
        {
            return;
        }
        const float &x_mean = x_sum / count;
        const float &z = 1.0 - x_mean;
        const float &z_var = x_sq_sum / count - x_mean * x_mean;
        Update(z, z_var);
        ROS_INFO("%f, %f, %f", z, z_var, x_);
        turtle_control::kf kf_msg;
        kf_msg.x = x_;
        kf_msg.z = z;
        kf_msg.z_var = z_var;
        pub_kf_.publish(kf_msg);
        // ROS_INFO("z: %f, z_var: %f, x_expected: %f", z, z_var, x_);
    }

private:
    float x_;
    float p_;
    ros::Subscriber sub_odom_;
    ros::Subscriber sub_scan_;
    ros::Time prev_prediction_;
    ros::Publisher pub_kf_;
};

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "turtle_localization");
    ros::NodeHandle nh("~");

    KalmanFilter1D kf(0, 0.05);
    kf.Init(nh);
    ros::spin();

    return 0;
}
