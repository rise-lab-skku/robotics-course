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

int main(int argc, char **argv)
{
    ros::init(argc, argv, "ik_node");
    ros::NodeHandle nh("/ik_node");
    ros::AsyncSpinner spinner(4); // Use 4 threads
    spinner.start();
    static const std::string LOGNAME = "ik_node";

    // rosrun fk_moveit fk_node _robot:=puma_560
    int robot_type;
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
    move_group.setWorkspace(-2, -2, 0, 2, 2, 3);

    // Setup for IK
    robot_state::RobotStatePtr kinematic_state(move_group.getCurrentState());
    const robot_state::JointModelGroup *joint_model_group = kinematic_state->getJointModelGroup(planning_group);

    // Placeholder for joint values
    std::vector<double> target_joints(joint_model_group->getVariableCount());

    // Demo
    int i = 0;
    while (ros::ok())
    {
        ROS_INFO("< Planning to waypoint %d >", i);

        // IK
        const unsigned int attempts = 10;
        const double timeout = 0.1;
        bool found_ik = kinematic_state->setFromIK(joint_model_group, waypoints[i], attempts, timeout);
        if (found_ik)
        {
            kinematic_state->copyJointGroupPositions(joint_model_group, target_joints);
            for (std::size_t i = 0; i < target_joints.size(); ++i)
            {
                ROS_INFO("\t(IK solution) Joint %d: %f", i, target_joints[i]);
            }
            Eigen::Vector3d reference_point_position(0.0, 0.0, 0.0);
            Eigen::MatrixXd jacobian;
            kinematic_state->getJacobian(
                joint_model_group,
                kinematic_state->getLinkModel(joint_model_group->getLinkModelNames().back()),
                reference_point_position,
                jacobian);
            ROS_INFO_STREAM("Jacobian: \n"
                            << jacobian);
        }
        else
        {
            ROS_WARN("Did not find IK solution");
        }

        // Motion planning
        move_group.setJointValueTarget(target_joints);
        moveit::planning_interface::MoveGroupInterface::Plan my_plan;
        bool success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
        if (success)
        {
            // Path visualization
            std::vector<geometry_msgs::Pose> eef_path;
            convert_JointTraj_to_PoseTraj(
                eef_path,
                kinematic_state,
                planning_group,
                my_plan.trajectory_.joint_trajectory);
            visual_tools.publishPath(eef_path, rvt::LIME_GREEN, rvt::SMALL);
            for (std::size_t i = 0; i < num_waypoints; ++i)
            {
                visual_tools.publishAxisLabeled(waypoints[i], "waypoint" + std::to_string(i));
            }
            visual_tools.publishPath(straight_line, rvt::RED, rvt::SMALL);
            visual_tools.trigger();

            // Result
            ROS_INFO("Planning successful");
            ROS_INFO("Planning time: %.4f sec", my_plan.planning_time_);
            move_group.execute(my_plan);
            ROS_INFO("Moving successful");

            // Last joints
            trajectory_msgs::JointTrajectoryPoint last_point = my_plan.trajectory_.joint_trajectory.points.back();
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
        i = (i + 1) % num_waypoints;
        ROS_INFO("-----------------\n");
    }
    ros::waitForShutdown();
    return 0;
}