#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <rviz_visual_tools/rviz_visual_tools.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_eigen/tf2_eigen.h>

namespace rvt = rviz_visual_tools;

void calcFK(
    const std::vector<double> &joint_values,
    const robot_state::RobotStatePtr &kinematic_state,
    const robot_model::JointModelGroup *joint_model_group,
    const std::string &eef_link,
    geometry_msgs::Pose &pose)
{
    kinematic_state->setJointGroupPositions(joint_model_group, joint_values);
    Eigen::Affine3d eef_transformation = kinematic_state->getGlobalLinkTransform(eef_link);
    pose = Eigen::toMsg(eef_transformation);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "reachable_ws_fk");
    ros::NodeHandle nh;
    ros::AsyncSpinner spinner(1);
    spinner.start();

    // Resolution of the configuration space
    double revolute_resolution_deg_;
    double prismatic_resolution_m;
    std::string planning_group;
    nh.param<double>("revolute_resolution_deg", revolute_resolution_deg_, 10);
    nh.param<double>("prismatic_resolution_m", prismatic_resolution_m, 0.01);
    nh.param<std::string>("planning_group", planning_group, "scara");
    double revolute_resolution_rad = revolute_resolution_deg_ * M_PI / 180.0;

    // Create a MoveGroup
    moveit::planning_interface::MoveGroupInterface move_group(planning_group);
    robot_state::RobotStatePtr kinematic_state(move_group.getCurrentState());
    const robot_state::JointModelGroup *joint_model_group =
        kinematic_state->getJointModelGroup(planning_group);

    // Get joint information
    const std::vector<const robot_state::JointModel*> &joint_model_vector =
        joint_model_group->getActiveJointModels();
    for (const robot_state::JointModel *joint_model : joint_model_vector)
    {
        ROS_INFO_STREAM("Joint [ " << joint_model->getName() << " ] has type " << joint_model->getType());
        ROS_INFO_STREAM("    and has a limit of " << joint_model->getVariableBounds("min_position") << " to " << joint_model->getVariableBounds("max_position"));
    }

    ros::waitForShutdown();
    return 0;
}