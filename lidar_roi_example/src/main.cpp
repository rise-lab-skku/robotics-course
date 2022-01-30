#include "lidar_roi_example/vlp16_roi.h"

int main(int argc, char **argv)
{
    // Initialize ROS
    ros::init(argc, argv, "lidar_roi");

    ROIExample::VLP16ROI roi;

    // Spin
    ros::spin();
}