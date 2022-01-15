/**
 * Boundary searching algorithm for the workspace of a robot.
 * Idea1: Octree, Quadtree
 * Idea2: Marching squares
 */
#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_eigen/tf2_eigen.h>

namespace rvt = rviz_visual_tools;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "ws_drawing");
    ros::NodeHandle nh("/ws_drawing");
    ros::AsyncSpinner spinner(4); // Use 4 threads
    spinner.start();
    static const std::string LOGNAME = "ws_drawing";

    // Moveit setup
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
    moveit::planning_interface::MoveGroupInterface move_group(planning_group);
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    robot_state::RobotStatePtr kinematic_state(move_group.getCurrentState());
    const robot_state::JointModelGroup *joint_model_group = kinematic_state->getJointModelGroup(planning_group);

    // Zero pose via FK
    std::vector<double> zero_joint_values(joint_model_group->getVariableCount(), 0.0);
    kinematic_state->setJointGroupPositions(joint_model_group, zero_joint_values);
    const std::string &eef_link = move_group.getEndEffectorLink();
    Eigen::Affine3d eef_transformation = kinematic_state->getGlobalLinkTransform(eef_link);
    geometry_msgs::Pose eef_pose = Eigen::toMsg(eef_transformation);
    ROS_INFO_STREAM("Zero pose: " << eef_pose);

    // Planning
    double max_sec = move_group.getPlanningTime();
    ROS_INFO_STREAM_NAMED(LOGNAME, "Planning time: " << max_sec);

    // move_group.setPositionTarget(eef_pose.position.x, eef_pose.position.y, eef_pose.position.z);
    // move_group.setGoalOrientationTolerance(M_PI * 2.0);

    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    bool success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    if (success)
    {
        ROS_INFO("Planning successful");
        move_group.execute(my_plan);
    }
    else
    {
        ROS_ERROR("Planning failed");
        return 1;
    }

    ros::waitForShutdown();
    return 0;
}