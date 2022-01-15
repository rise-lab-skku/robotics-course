/**
 * Boundary searching algorithm for the workspace of a robot.
 * Idea1: Octree
 * Idea2: Marching squares
 */
#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_eigen/tf2_eigen.h>

namespace rvt = rviz_visual_tools;

void calcFK(
    const std::vector<double>& joint_values,
    const robot_state::RobotStatePtr& kinematic_state,
    const robot_model::JointModelGroup* joint_model_group,
    const std::string& eef_link,
    geometry_msgs::Pose& pose)
{
    kinematic_state->setJointGroupPositions(joint_model_group, joint_values);
    Eigen::Affine3d eef_transformation = kinematic_state->getGlobalLinkTransform(eef_link);
    pose = Eigen::toMsg(eef_transformation);
}

bool calcIK(
    const geometry_msgs::Pose& eef_pose,
    const robot_state::RobotStatePtr& kinematic_state,
    const robot_model::JointModelGroup* joint_model_group,
    std::vector<double>& joint_values)
{
    const unsigned int attempts = 10;
    const double timeout = 0.1;
    bool found_ik = kinematic_state->setFromIK(joint_model_group, eef_pose, attempts, timeout);
    if (found_ik)
    {
        kinematic_state->copyJointGroupPositions(joint_model_group, joint_values);
    }
    return found_ik;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "ws_drawing");
    ros::NodeHandle nh("/ws_drawing");
    ros::AsyncSpinner spinner(4); // Use 4 threads
    spinner.start();
    static const std::string LOGNAME = "ws_drawing";

    // Get the input arguments
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
    if (planning_group.compare("rrr") == 0)
    {
        ROS_WARN_STREAM_NAMED(LOGNAME, "Planning group rrr is not supported. Due to the 2D workspace. This ws_drawing is designed for the 3D workspace.");
        return 1;
    }

    // Moveit setup
    moveit::planning_interface::MoveGroupInterface move_group(planning_group);
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    robot_state::RobotStatePtr kinematic_state(move_group.getCurrentState());
    const robot_state::JointModelGroup *joint_model_group = kinematic_state->getJointModelGroup(planning_group);


    // Set a rosParam for the KDL Kinematics Plugin
    const std::string position_only_ik_param_name =
        "/robot_description_kinematics/" + planning_group + "/position_only_ik";
    nh.setParam(position_only_ik_param_name, true);
    ROS_INFO("Waiting for the param to be set");
    while (true)
    {
        bool position_only = false;
        nh.getParam(position_only_ik_param_name, position_only);
        if (position_only) { break; }
        else { ros::Duration(0.1).sleep(); }
    }
    ROS_INFO("Param setting complete!");

    // Print some info
    ROS_INFO_STREAM("      Joint tolerance: " << move_group.getGoalJointTolerance());
    ROS_INFO_STREAM("   Position tolerance: " << move_group.getGoalPositionTolerance());
    ROS_INFO_STREAM("Orientation tolerance: " << move_group.getGoalOrientationTolerance());
    ROS_INFO_STREAM("      Reference frame: " << move_group.getPlanningFrame());
    ROS_INFO_STREAM("    End effector link: " << move_group.getEndEffectorLink());

    // Placeholder for the IK solution
    std::vector<double> joint_values(joint_model_group->getVariableCount(), 0.0);

    // Zero pose for the initial pose
    const std::string &eef_link = move_group.getEndEffectorLink();
    geometry_msgs::Pose zero_pose;
    calcFK(joint_values, kinematic_state, joint_model_group, eef_link, zero_pose);
    ROS_INFO_STREAM("Initial eef pose (zero_pose): " << zero_pose);

    /**
     * Octree + Marching squares
     */
    




    const unsigned int attempts = 10;
    const double timeout = 0.1;
    bool found_ik = kinematic_state->setFromIK(joint_model_group, eef_pose, attempts, timeout);
    if (found_ik)
    {
        kinematic_state->copyJointGroupPositions(joint_model_group, joint_values);
        for (std::size_t j = 0; j < joint_values.size(); ++j)
        {
            ROS_INFO("(IK solution) Joint %d: %f", j, joint_values[j]);
        }
    }
    else
    {
        ROS_WARN("Did not find IK solution");
    }

    // Motion planning
    move_group.setJointValueTarget(joint_values);
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    bool success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    if (success)
    {
        // Result
        ROS_INFO("Planning successful");
        ROS_INFO("Planning time: %.4f sec", my_plan.planning_time_);
        move_group.execute(my_plan);
        ROS_INFO("Moving successful");
    }
    else
    {
        ROS_WARN("Planning failed");
    }

    ros::waitForShutdown();
    return 0;
}