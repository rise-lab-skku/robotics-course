#include "path_generator/turtlebot_path_generator.h"

namespace path_generator
{
    ExamplePath::ExamplePath()
    {
        pub_path_ = nh_.advertise<visualization_msgs::Marker>("/turtlebot_example/path", 3);
        pub_points_ = nh_.advertise<turtlebot_path_tracker::LocalPathPoints>("/path", 3);
    }

    void ExamplePath::PathGenerator()
    {
        visualization_msgs::Marker marker;
        marker.header.frame_id = "odom";
        marker.header.stamp = ros::Time();
        marker.ns = "path";
        marker.id = 0;
        // marker.type = visualization_msgs::Marker::SPHERE;
        marker.type = 8;
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = 0.0;
        marker.pose.orientation.z = 0.0;
        marker.pose.orientation.w = 1.0;
        marker.scale.x = 0.1;
        marker.scale.y = 0.1;
        marker.scale.z = 0.1;
        marker.color.a = 1.0;
        marker.color.r = 0.0;
        marker.color.g = 0.0;
        marker.color.b = 1.0;

        //make way points
        p1_.x = 0.0; p1_.y = 0.0; vis_points_.push_back(p1_);
        p2_.x = 1.0; p2_.y = 0.0; vis_points_.push_back(p2_);
        p3_.x = 1.0; p3_.y = -1.0; vis_points_.push_back(p3_);
        p4_.x = 0.0; p4_.y = -1.0; vis_points_.push_back(p4_);
        p5_.x = 0.0; p5_.y = -2.0; vis_points_.push_back(p5_);
        p6_.x = 1.0; p6_.y = -2.0; vis_points_.push_back(p6_);
        marker.points = vis_points_;
        PathInterpolation();
        pub_path_.publish(marker);
    }

    void ExamplePath::PathInterpolation()
    {
        float itp = 20.0;
        auto it = vis_points_.begin();
        auto end = vis_points_.end();

        for (it; it != end - 1; it++)
        {
            for (int i=0; i<itp; i++)
            {
                geometry_msgs::Point itp_point;
                itp_point.x = (*it).x + ((*(it+1)).x - (*it).x) / itp * i;
                itp_point.y = (*it).y + ((*(it+1)).y - (*it).y) / itp * i;
                path_.push_back(itp_point);
            }
        }

        turtlebot_path_tracker::LocalPathPoints path;
        path.path_points = path_;
        pub_points_.publish(path);
        path_.clear();
        vis_points_.clear();
    }
}