#include <path_tracker/turtlebot_path_tracker.h>

int main(int argc, char** argv)
{   
    ros::init(argc, argv, "path_tracker_node");
    ros::NodeHandle nh("~");

    path_tracker::Tracker track;
    track.Init(nh);
    ros::spin();

    // ros::Rate rate(20);

    // while (ros::ok())
    // {
    //     rate.sleep();
    // }

    return 0;
}