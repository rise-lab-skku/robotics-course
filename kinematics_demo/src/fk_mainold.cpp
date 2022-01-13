#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <trajectory_msgs/JointTrajectory.h>

namespace rvt = rviz_visual_tools;

const int PUMA_TYPE = 0;
const int RRR_TYPE = 1;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "fk_node");
    ros::NodeHandle nh("/fk_node");
    ros::AsyncSpinner spinner(4); // Use 4 threads
    spinner.start();
    static const std::string LOGNAME = "fk_node";

    // rosrun fk_moveit fk_node _robot:=puma_560
    int robot_type;
    std::string planning_group;
    if (nh.getParam("robot", planning_group)) {
        ROS_INFO_STREAM_NAMED(LOGNAME, "Using planning group: " << planning_group);
    } else {
        ROS_ERROR_STREAM_NAMED(LOGNAME, "No planning group specified");
        return 1;
    }

    if (planning_group.compare("puma_560") == 0) {robot_type = PUMA_TYPE;}
    else if (planning_group.compare("rrr") == 0) {robot_type = RRR_TYPE;}
    else {
        ROS_ERROR_STREAM_NAMED(LOGNAME, "Unsupported planning group: " << planning_group);
        return 1;
    }

    // Setup
    moveit::planning_interface::MoveGroupInterface move_group(planning_group);
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    const robot_state::JointModelGroup *joint_model_group =
        move_group.getCurrentState()->getJointModelGroup(planning_group);

    // Visualization
    moveit_visual_tools::MoveItVisualTools visual_tools("link1");
    visual_tools.deleteAllMarkers();
    visual_tools.loadRemoteControl();

    // Joint trajectory before spline
    std::vector<std::vector<double>> joint_waypoints;
    switch (robot_type)
    {
    case PUMA_TYPE:
        joint_waypoints = {
            {-0.158621, 1.182338, -0.839760, 0.000001, 0.342578, -0.158622},  // way0
            {-0.070632, 0.628602,  0.316969, 0.000000, 0.945571, -0.070632},  // way1
            { 0.485967, 0.628602,  0.316969, 0.000000, 0.945570,  0.485967},  // way2
            { 1.017371, 1.182317, -0.839752, 0.000000, 0.342565,  1.017371}   // way3
        };
        break;
    case RRR_TYPE:
        joint_waypoints = {
            {0.0, 0.0, 0.0},
            {2.104, -2.636, 0.0},
            {1.508, -1.446, 0.0},
            {-0.063, -1.446, 0.0},
            {0.533, -2.636, 0.0}};
        break;
    default:
        ROS_ERROR_STREAM_NAMED(LOGNAME, "Not implemented error. robot type: " << robot_type);
        return 1;
    }

    // Current joints
    std::vector<double> group_variable_values;
    group.getCurrentState()->copyJointGroupPositions(group.getCurrentState()->getRobotModel()->getJointModelGroup(group.getName()), group_variable_values);

    // Find a joint trajectory without MoveIt
    trajectory_msgs::JointTrajectory sparse_traj;

    // Cubic spline

    // Run
    

    ros::waitForShutdown();
    return 0;
}