#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_eigen/tf2_eigen.h>

namespace rvt = rviz_visual_tools;

void convert_JointTraj_to_PoseTraj(
    std::vector<geometry_msgs::Pose> &path,
    const robot_state::RobotStatePtr &rs,
    const std::string &planning_group,
    const trajectory_msgs::JointTrajectory &joint_traj)
{
    path.clear();
    const moveit::core::JointModelGroup *jmg = rs->getJointModelGroup(planning_group);
    const std::string &eef_name = jmg->getLinkModelNames().back();
    // Joint trajectory to Cartesian path (i == trajectory length)
    for (int i = 0; i < joint_traj.points.size(); i++)
    {
        // Joint trajectory to robot state
        rs->setJointGroupPositions(jmg, joint_traj.points[i].positions);
        // Get the position of the end-effector from the RobotState
        Eigen::Affine3d eef_transformation = rs->getGlobalLinkTransform(eef_name);
        geometry_msgs::Pose eef_pose = Eigen::toMsg(eef_transformation);
        path.push_back(eef_pose);
    }
}

// Linear interpolation between two position of poses
void linear_interpolation(
    std::vector<geometry_msgs::Pose> &path,
    const geometry_msgs::Pose &pose1,
    const geometry_msgs::Pose &pose2,
    int n_points)
{
    path.clear();
    double dx = (pose2.position.x - pose1.position.x) / (n_points - 1);
    double dy = (pose2.position.y - pose1.position.y) / (n_points - 1);
    double dz = (pose2.position.z - pose1.position.z) / (n_points - 1);
    for (int i = 0; i < n_points; i++)
    {
        geometry_msgs::Pose pose = pose1;
        pose.position.x += i * dx;
        pose.position.y += i * dy;
        pose.position.z += i * dz;
        path.push_back(pose);
    }
    path.push_back(pose2);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "ik_linear");
    ros::NodeHandle nh("/ik_linear");
    ros::AsyncSpinner spinner(4); // Use 4 threads
    spinner.start();
    static const std::string LOGNAME = "ik_linear";

    // rosrun kinematics_dmeo ik_linear _robot:=puma_560
    std::string planning_group;
    if (nh.getParam("robot", planning_group))
    {
        ROS_INFO_STREAM_NAMED(LOGNAME, "Using planning group: " << planning_group);
    }
    else
    {
        ROS_ERROR_STREAM_NAMED(LOGNAME, "No planning group specified");
        return 1;
    }

    // Waypoints: Sqaure trajectory
    geometry_msgs::Pose target_pose1;
    double dx, dy;
    if (planning_group.compare("puma_560") == 0)
    {
        target_pose1.position.x = 0.3;
        target_pose1.position.y = -0.2;
        target_pose1.position.z = 0.6;
        tf2::Quaternion target_quat(tf2::Vector3(1.0, 0.0, 0.0), M_PI);
        target_pose1.orientation = tf2::toMsg(target_quat);
        dx = 0.4;
        dy = 0.4;
    }
    else if (planning_group.compare("rrr") == 0)
    {
        target_pose1.position.x = 0.25;
        target_pose1.position.y = -0.1;
        target_pose1.position.z = 0.02;
        target_pose1.orientation.w = 1.0;
        dx = 0.2;
        dy = 0.2;
    }
    else
    {
        ROS_ERROR_STREAM_NAMED(LOGNAME, "Unsupported planning group: " << planning_group);
        return 1;
    }

    int num_waypoints = 4;
    // setFromIK 함수가 입력을 Pose로만 받아서, PoseStamped가 아니라 Pose로 작성함.
    std::vector<geometry_msgs::Pose> waypoints(num_waypoints, target_pose1);
    waypoints[1].position.x += dx;
    waypoints[2].position.x += dx;
    waypoints[2].position.y += dy;
    waypoints[3].position.y += dy;
    // For waypoint visualization
    std::vector<geometry_msgs::Pose> straight_line = waypoints;
    straight_line.push_back(waypoints[0]);

    // Visualization
    std::string base_frame = "link1";
    moveit_visual_tools::MoveItVisualTools visual_tools(base_frame);
    visual_tools.deleteAllMarkers();
    visual_tools.loadRemoteControl();

    // Setup for planning
    moveit::planning_interface::MoveGroupInterface move_group(planning_group);
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

    // Setup for IK
    robot_state::RobotStatePtr kinematic_state(move_group.getCurrentState());
    const robot_state::JointModelGroup *joint_model_group = kinematic_state->getJointModelGroup(planning_group);

    // Placeholder for joint values
    std::vector<double> joint_values(joint_model_group->getVariableCount());

    // Demo
    int i = 0;
    while (ros::ok())
    {
        int next_i = (i + 1) % num_waypoints;
        ROS_INFO("< Planning waypoint from %d to %d >", i, next_i);

        // IK
        const unsigned int attempts = 10;
        const double timeout = 0.1;
        bool found_ik = kinematic_state->setFromIK(joint_model_group, waypoints[next_i], attempts, timeout);
        if (found_ik)
        {
            kinematic_state->copyJointGroupPositions(joint_model_group, joint_values);
            for (std::size_t j = 0; j < joint_values.size(); ++j)
            {
                ROS_INFO("\t(IK solution of way%d) Joint %d: %f", next_i, j, joint_values[j]);
            }
            Eigen::Vector3d reference_point_position(0.0, 0.0, 0.0);
            Eigen::MatrixXd jacobian;
            kinematic_state->getJacobian(
                joint_model_group,
                kinematic_state->getLinkModel(joint_model_group->getLinkModelNames().back()),
                reference_point_position,
                jacobian);
            ROS_INFO_STREAM("Jacobian: \n" << jacobian);
        }
        else
        {
            ROS_WARN("Did not find IK solution");
        }

        // Pose -> PoseStamped
        geometry_msgs::PoseStamped pose_stamped;
        pose_stamped.header.frame_id = move_group.getPlanningFrame();
        pose_stamped.pose = waypoints[i];

        // Linear interpolation of Position
        std::vector<geometry_msgs::Pose> waypoints_lin;
        linear_interpolation(waypoints_lin, waypoints[i], waypoints[next_i], 10);

        // Motion planning
        moveit_msgs::RobotTrajectory trajectory;
        double fraction = -1.0;  // -1.0 is error
        for (std::size_t attempts_ = 10; attempts_ > 0; attempts_--)
        {
            fraction = move_group.computeCartesianPath(
                waypoints_lin,
                0.01,  // eef_step
                0.0,   // jump_threshold
                trajectory
            );
            if (fraction > 0.0) {break;}
        }
        ROS_INFO("Cartesian path (%.2f%% acheived)", fraction * 100.0);

        moveit::planning_interface::MoveGroupInterface::Plan my_plan;
        if (fraction > 0.95)
        {
            // Path visualization
            std::vector<geometry_msgs::Pose> eef_path;
            convert_JointTraj_to_PoseTraj(
                eef_path,
                kinematic_state,
                planning_group,
                trajectory.joint_trajectory);
            visual_tools.publishPath(eef_path, rvt::LIME_GREEN, rvt::SMALL);
            for (std::size_t i = 0; i < num_waypoints; ++i)
            {
                visual_tools.publishAxisLabeled(waypoints[i], "waypoint" + std::to_string(i));
            }
            visual_tools.publishPath(straight_line, rvt::RED, rvt::SMALL);
            visual_tools.trigger();

            // Result
            ROS_INFO("Planning successful");
            my_plan.trajectory_ = trajectory;
            move_group.execute(my_plan);
            ROS_INFO("Moving successful");

            // Last joints
            trajectory_msgs::JointTrajectoryPoint last_point = trajectory.joint_trajectory.points.back();
            for (std::size_t i = 0; i < last_point.positions.size(); ++i)
            {
                ROS_INFO("\t(Current) Joint %d: %f", i, last_point.positions[i]);
            }
        }
        else
        {
            ROS_WARN("Planning failed");
        }
        // Indexing
        i = next_i;
        ROS_INFO("-----------------\n");
    }
    ros::waitForShutdown();
    return 0;
}