#ifndef LIDAR_LIDAR_VLP16_SUBSCRIBER_H_
#define LIDAR_LIDAR_VLP16_SUBSCRIBER_H_

#include <ros/ros.h>
#include <iostream>
#include <vector>
#include <sensor_msgs/point_cloud_conversion.h>
#include <visualization_msgs/Marker.h>

// PCL specific includes
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/passthrough.h>
#include <pcl/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_ros/transforms.h>
#include <pcl_ros/point_cloud.h>
#include <cmath>

namespace ROIExample
{
    class VLP16ROI
    {
    public:
        VLP16ROI();

    private:
        ros::NodeHandle nh_;
        ros::Subscriber sub_;
        ros::Publisher pub_cloud_;
        ros::Publisher pub_line_;

        void CloudCallback(const sensor_msgs::PointCloud2ConstPtr &input);
        void SetROI(const pcl::PointCloud<pcl::PointXYZ>::Ptr &input);
        void VisROI();
    };
}
#endif // LIDAR_LIDAR_VLP16_SUBSCRIBER_H_