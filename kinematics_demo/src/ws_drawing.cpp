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

bool checkIK(
    const geometry_msgs::Pose &eef_pose,
    const robot_state::RobotStatePtr &kinematic_state,
    const robot_model::JointModelGroup *joint_model_group)
{
    const unsigned int attempts = 10;
    const double timeout = 0.1;
    return kinematic_state->setFromIK(joint_model_group, eef_pose, attempts, timeout);
}

bool calcIK(
    const geometry_msgs::Pose &eef_pose,
    const robot_state::RobotStatePtr &kinematic_state,
    const robot_model::JointModelGroup *joint_model_group,
    std::vector<double> &joint_values)
{
    bool found_ik = checkIK(eef_pose, kinematic_state, joint_model_group);
    if (found_ik)
        kinematic_state->copyJointGroupPositions(joint_model_group, joint_values);
    return found_ik;
}

class Cube
{
public:
    Cube() : anchor(geometry_msgs::Point()) {}
    ~Cube() {}

    geometry_msgs::Point anchor; // The position of ba corner
    double width;
    /**
     * IK results for each corner. true == solution found.
     * Cube 8 division order:
     *     [+x: foward, +y: left, +z: up] => (+++)
     *     Top CCW   : ta(--+) -> tb(+-+) -> tc(+++) -> td(-++)
     *     Bottom CCW: ba(---) -> bb(+--) -> bc(++-) -> bd(-+-)
     */
    bool ta, tb, tc, td;
    bool ba, bb, bc, bd;
};

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

    // Search space
    // double x_min = -2.0;
    // double y_min = -2.0;
    // double z_min = -2.0;
    // double x_max = 2.0;
    // double y_max = 2.0;
    // double z_max = 2.0;
    // move_group.setWorkspace(x_min, y_min, z_min, x_max, y_max, z_max);

    // Set a rosParam for the KDL Kinematics Plugin
    const std::string position_only_ik_param_name =
        "/robot_description_kinematics/" + planning_group + "/position_only_ik";
    nh.setParam(position_only_ik_param_name, true);

    ROS_INFO("Waiting for the param to be set");
    bool position_only = false;
    while (!position_only)
    {
        nh.getParam(position_only_ik_param_name, position_only);
        ros::Duration(0.1).sleep();
    }
    ROS_INFO("Param setting complete!: position_only_ik = %d", position_only);

    // Print some info
    ROS_INFO_STREAM("      Joint tolerance: " << move_group.getGoalJointTolerance());
    ROS_INFO_STREAM("   Position tolerance: " << move_group.getGoalPositionTolerance());
    ROS_INFO_STREAM("Orientation tolerance: " << move_group.getGoalOrientationTolerance());
    ROS_INFO_STREAM("      Reference frame: " << move_group.getPlanningFrame());
    ROS_INFO_STREAM("    End effector link: " << move_group.getEndEffectorLink());

    // Zero pose for the initial pose
    const std::string &eef_link = move_group.getEndEffectorLink();
    geometry_msgs::Pose zero_pose;
    std::vector<double> joint_values(joint_model_group->getVariableCount(), 0.0);
    calcFK(joint_values, kinematic_state, joint_model_group, eef_link, zero_pose);
    ROS_INFO_STREAM("Initial eef pose (zero_pose):\n"
                    << zero_pose);

    /**
     * Octree + Marching squares
     */
    zero_pose.position.z += 0.3;
    calcIK(zero_pose, kinematic_state, joint_model_group, joint_values);
    for (auto &jv : joint_values)
        ROS_INFO_STREAM("IK solution1: " << jv);

    /***************
     * Octree
     ***************/
    const double resolution = 0.1; // Minimum cube width
    std::vector<Cube> openlist(128, Cube());
    std::size_t top = 0; // Top index of the openlist

    // Initialize the openlist
    double initial_width = 2.0;
    openlist.at(top).

        /*
        openlist = { initial 4 cube }

        while (openlist has elem)
        {
            cube = openlist.pop_back()

            if (8 vertices are same)
            {
                continue
                top -= 1
            }
            else if (width > min_width)
            {
                split into 8 cubes
                openlist.push_back(8 cubes)
                top += 8
            }
            else
            {
                draw marching square
            }
        }

        */

        ros::waitForShutdown();
    return 0;
}