#ifndef TURTLEBOT_PATH_TRACKER_PATH_TRACKER_TURTLEBOT_PATH_TRACKER_H_
#define TURTLEBOT_PATH_TRACKER_PATH_TRACKER_TURTLEBOT_PATH_TRACKER_H_

#include <ros/ros.h>
#include <vector>
#include <nav_msgs/Odometry.h>
#include <turtlebot3_fake/WheelMsg.h>
#include <turtlebot_path_tracker/LocalPathPoints.h>
#include <geometry_msgs/Point.h>

namespace path_tracker
{
    class Tracker
    {
    public:
        void Init(ros::NodeHandle &nh);

    private:
        void FeedbackCallback(const nav_msgs::Odometry::Ptr &msg);
        void PathCallback(const turtlebot_path_tracker::LocalPathPoints::Ptr &path);
        //void UpdateLD();
        //void UpdateWL();
        void UpdateIndex();
        void UpdateDelta();
        void UpdateCmd();
        float Distance(const geometry_msgs::Point &p0, const geometry_msgs::Point &p1);

        ros::Publisher pub_cmd;
        ros::Subscriber sub_state;
        ros::Subscriber sub_path;
        
        // latest state
        nav_msgs::Odometry latest_state;

        // path and indices
        std::vector<geometry_msgs::Point> local_path;
        int idx_n1 = 0;

        // vehicle property
        float delta_to_command;
        float L;
        float Ke;

        // control parameter
        float arrive_distance;

        float LD;
        float alpha;
        geometry_msgs::Point WL_xy;
        turtlebot3_fake::WheelMsg msg_command;
        geometry_msgs::Point n1;
        geometry_msgs::Point cur_pos;
    };
}
#endif //TURTLEBOT_PATH_TRACKER_PATH_TRACKER_TURTLEBOT_PATH_TRACKER_H_