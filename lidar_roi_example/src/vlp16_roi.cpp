#include "lidar_roi_example/vlp16_roi.h"

namespace ROIExample
{
  VLP16ROI::VLP16ROI()
  {
    sub_ = nh_.subscribe<sensor_msgs::PointCloud2>("/velodyne_points", 1, &VLP16ROI::CloudCallback, this);
    pub_cloud_ = nh_.advertise<sensor_msgs::PointCloud2>("/roi_cloud", 1);
    pub_line_ = nh_.advertise<visualization_msgs::Marker>("/roi_box", 1);
  }

  void VLP16ROI::CloudCallback(const sensor_msgs::PointCloud2ConstPtr &input)
  {
    pcl::PointCloud<pcl::PointXYZ>::Ptr lidar_cloud_(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*input, *lidar_cloud_);
    SetROI(lidar_cloud_);
  }

  void VLP16ROI::SetROI(const pcl::PointCloud<pcl::PointXYZ>::Ptr &input)
  {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PassThrough<pcl::PointXYZ> pass;
    pass.setInputCloud(input);
    pass.setFilterFieldName("z");
    pass.setFilterLimits(-1, 1);
    pass.filter(*cloud_filtered);

    pass.setInputCloud(cloud_filtered);
    pass.setFilterFieldName("y");
    pass.setFilterLimits(-2, 2);
    pass.filter(*cloud_filtered);

    pass.setInputCloud(cloud_filtered);
    pass.setFilterFieldName("x");
    pass.setFilterLimits(0, 5);
    pass.filter(*cloud_filtered);

    pub_cloud_.publish(cloud_filtered);
    VisROI();
  }

  void VLP16ROI::VisROI()
  {
    visualization_msgs::Marker line_list;
    line_list.header.frame_id = "velodyne";
    line_list.header.stamp = ros::Time::now();
    line_list.ns = "roi_box";
    line_list.id = 0;
    line_list.type = 1;
    line_list.action = visualization_msgs::Marker::ADD;
    line_list.scale.x = 5.0;
    line_list.scale.y = 4.0;
    line_list.scale.z = 2.0;
    line_list.color.a = 0.5;
    line_list.color.r = 1.0;
    line_list.color.g = 1.0;
    line_list.color.b = 1.0;
    line_list.pose.position.x = 2.5;
    line_list.pose.position.y = 0.0;
    line_list.pose.position.z = 0.0;
    line_list.pose.orientation.x = 0;
    line_list.pose.orientation.y = 0;
    line_list.pose.orientation.z = 0;
    line_list.pose.orientation.w = 1;
    pub_line_.publish(line_list);
  }
}
