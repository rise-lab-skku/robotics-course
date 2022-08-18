#ifndef TURTLEBOT_PATH_TRACKER_PATH_GENERATOR_TURTLEBOT_PATH_GENERATOR_H_
#define TURTLEBOT_PATH_TRACKER_PATH_GENERATOR_TURTLEBOT_PATH_GENERATOR_H_

#include <turtlebot_path_tracker/LocalPathPoints.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Point.h>

#include <ros/ros.h>
#include <vector>

namespace path_generator
{
    class ExamplePath
    {
    public:
        ExamplePath();
        void PathGenerator();
    
    private:
        void PathInterpolation();
        ros::NodeHandle nh_;
        ros::Publisher pub_path_;
        ros::Publisher pub_points_;
        std::vector<geometry_msgs::Point> vis_points_;
        std::vector<geometry_msgs::Point> path_;
        geometry_msgs::Point p1_, p2_, p3_, p4_, p5_, p6_;
    };
}

#endif //TURTLEBOT_PATH_TRACKER_PATH_GENERATOR_TURTLEBOT_PATH_GENERATOR_H_