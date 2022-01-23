#include <path_tracker/turtlebot_path_tracker.h>

namespace path_tracker
{
    void Tracker::Init(ros::NodeHandle &nh)
    {
        // get parameter
        ROS_ASSERT(nh.getParam("arrive_dis", arrive_distance)); //도착 판단 거리
        ROS_ASSERT(nh.getParam("LD", LD));
        ROS_ASSERT(nh.getParam("Ke", Ke));

        // ready for publish
        pub_cmd = nh.advertise<turtlebot3_fake::WheelMsg>("/cmd_vel", 3);

        // subscriber
        sub_path = nh.subscribe("/path", 1, &Tracker::PathCallback, this);
        sub_state = nh.subscribe("/odom", 1, &Tracker::FeedbackCallback, this);
    }

    void Tracker::FeedbackCallback(const nav_msgs::Odometry::Ptr &msg)
    {
        // udpate latest state
        latest_state = *msg;
        cur_pos.x = latest_state.pose.pose.position.x;
        cur_pos.y = latest_state.pose.pose.position.y;

        if (local_path.size() == 0)
        {
            return;
        }

        if (idx_n1 == local_path.size() - 1)
        {
            float goal_distance = Distance(cur_pos, local_path[local_path.size() - 1]);
            if (goal_distance < 0.01)
            {
                msg_command.left_wheel = 0.0;
                msg_command.right_wheel = 0.0;
                pub_cmd.publish(msg_command);
                return;
            }
        }

        UpdateIndex();
        UpdateDelta();
        UpdateCmd();

        pub_cmd.publish(msg_command);
    }

    void Tracker::PathCallback(const turtlebot_path_tracker::LocalPathPoints::Ptr &path)
    {
        local_path = path->path_points;
        n1 = local_path[idx_n1];
    }

    void Tracker::UpdateIndex()
    {
        // 로봇이 way point를 지나면 idx가 1증가
        float distance = Distance(cur_pos, n1);

        // Arrival distance와 비교
        if (distance < arrive_distance)
        {
            while (true)
            {
                idx_n1++;
                if (idx_n1 >= local_path.size() - 1)
                {
                    idx_n1 = local_path.size() - 1;
                }

                distance = Distance(cur_pos, local_path[idx_n1]);
                if (distance > arrive_distance || idx_n1 == local_path.size() - 1)
                {
                    break;
                }
            }
        }

        n1 = local_path[idx_n1];
    }

    void Tracker::UpdateDelta()
    {
        // finding alpha
        const float &X = - n1.y + latest_state.pose.pose.position.y;
        const float &Y = n1.x - latest_state.pose.pose.position.x;
        float heading = (asin(latest_state.pose.pose.orientation.z) * 2) * (180 / M_PI);
        float path_heading = atan2(Y, X) * (180 / M_PI) - 90;

        path_heading = (path_heading == -90) ? 0 : path_heading;
        path_heading = (fabs(path_heading) > 180) ? path_heading + 360 : path_heading;
        alpha = (path_heading - heading) * (M_PI / 180);
    }

    void Tracker::UpdateCmd()
    {
        msg_command.right_wheel = 0.05 + 0.16 * Ke * alpha;
        msg_command.left_wheel = 0.05 - 0.16 * Ke * alpha;
    }

    float Tracker::Distance(const geometry_msgs::Point &p0, const geometry_msgs::Point &p1)
    {
        return sqrt(pow((p0.x - p1.x), 2) + pow((p0.y - p1.y), 2));
    }
}