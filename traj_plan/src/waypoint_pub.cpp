#include <trajectory_msgs/JointTrajectory.h>
#include <traj_plan/PoseStampedArray.h>
#include <ros/ros.h>
#include <iostream>
#include <vector>

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "waypoint");
    ros::NodeHandle nh("~");

    ros::Publisher pub_jl = nh.advertise<trajectory_msgs::JointTrajectory>("/traj_plan/linear/joint_waypoints", 3);
    ros::Publisher pub_js = nh.advertise<trajectory_msgs::JointTrajectory>("/traj_plan/spline/joint_waypoints", 3);
    ros::Publisher pub_pl = nh.advertise<traj_plan::PoseStampedArray>("/traj_plan/linear/pose_waypoints", 3);
    ros::Publisher pub_ps = nh.advertise<traj_plan::PoseStampedArray>("/traj_plan/spline/pose_waypoints", 3);

    traj_plan::PoseStampedArray new_pose_msg;
    ros::Time start = ros::Time::now();

    new_pose_msg.data.push_back(geometry_msgs::PoseStamped());
    new_pose_msg.data.back().header.stamp = start + ros::Duration(0.0);
    new_pose_msg.data.back().pose.position.x = 0.0;
    new_pose_msg.data.back().pose.position.y = 0.0;
    new_pose_msg.data.back().pose.position.z = 0.0;
    new_pose_msg.data.back().pose.orientation.w = 1.0;
    new_pose_msg.data.back().pose.orientation.x = 0.0;
    new_pose_msg.data.back().pose.orientation.y = 0.0;
    new_pose_msg.data.back().pose.orientation.z = 0.0;

    new_pose_msg.data.push_back(geometry_msgs::PoseStamped());
    new_pose_msg.data.back().header.stamp = start + ros::Duration(1.0);
    new_pose_msg.data.back().pose.position.x = 1.0;
    new_pose_msg.data.back().pose.position.y = 0.0;
    new_pose_msg.data.back().pose.position.z = 0.0;
    new_pose_msg.data.back().pose.orientation.w = 1.0;
    new_pose_msg.data.back().pose.orientation.x = 0.0;
    new_pose_msg.data.back().pose.orientation.y = 0.0;
    new_pose_msg.data.back().pose.orientation.z = 0.0;

    new_pose_msg.data.push_back(geometry_msgs::PoseStamped());
    new_pose_msg.data.back().header.stamp = start + ros::Duration(2.0);
    new_pose_msg.data.back().pose.position.x = 1.0;
    new_pose_msg.data.back().pose.position.y = 1.0;
    new_pose_msg.data.back().pose.position.z = 0.0;
    new_pose_msg.data.back().pose.orientation.w = 1.0;
    new_pose_msg.data.back().pose.orientation.x = 0.0;
    new_pose_msg.data.back().pose.orientation.y = 0.0;
    new_pose_msg.data.back().pose.orientation.z = 0.0;

    new_pose_msg.data.push_back(geometry_msgs::PoseStamped());
    new_pose_msg.data.back().header.stamp = start + ros::Duration(3.0);
    new_pose_msg.data.back().pose.position.x = 0.0;
    new_pose_msg.data.back().pose.position.y = 1.0;
    new_pose_msg.data.back().pose.position.z = 0.0;
    new_pose_msg.data.back().pose.orientation.w = 1.0;
    new_pose_msg.data.back().pose.orientation.x = 0.0;
    new_pose_msg.data.back().pose.orientation.y = 0.0;
    new_pose_msg.data.back().pose.orientation.z = 0.0;

    trajectory_msgs::JointTrajectory new_msg;
    new_msg.header.frame_id = "waypoints";

    new_msg.joint_names.push_back("j0");
    new_msg.joint_names.push_back("j1");
    new_msg.joint_names.push_back("j2");

    new_msg.points.push_back(trajectory_msgs::JointTrajectoryPoint());
    new_msg.points.back().time_from_start = ros::Duration(0);
    new_msg.points.back().positions.push_back(0);
    new_msg.points.back().positions.push_back(0);
    new_msg.points.back().positions.push_back(0);

    new_msg.points.push_back(trajectory_msgs::JointTrajectoryPoint());
    new_msg.points.back().time_from_start = ros::Duration(1);
    new_msg.points.back().positions.push_back(1);
    new_msg.points.back().positions.push_back(2);
    new_msg.points.back().positions.push_back(3);

    new_msg.points.push_back(trajectory_msgs::JointTrajectoryPoint());
    new_msg.points.back().time_from_start = ros::Duration(2);
    new_msg.points.back().positions.push_back(5);
    new_msg.points.back().positions.push_back(4);
    new_msg.points.back().positions.push_back(3);

    new_msg.points.push_back(trajectory_msgs::JointTrajectoryPoint());
    new_msg.points.back().time_from_start = ros::Duration(3);
    new_msg.points.back().positions.push_back(7);
    new_msg.points.back().positions.push_back(8);
    new_msg.points.back().positions.push_back(9);

    ros::Rate rate(1);
    while (ros::ok())
    {
        new_msg.header.stamp = ros::Time::now();
        pub_jl.publish(new_msg);
        pub_js.publish(new_msg);

        pub_pl.publish(new_pose_msg);
        pub_ps.publish(new_pose_msg);

        ros::spinOnce();
        rate.sleep();
    }
    return 0;
}