#include <iomanip>
#include <vector>
#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <geometry_msgs/Pose.h>
#include <sensor_msgs/JointState.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/InteractiveMarkerControl.h>
#include <interactive_markers/interactive_marker_server.h>
#include <tf2_eigen/tf2_eigen.h>

// Global variables to make this code easier
geometry_msgs::Pose target1;
geometry_msgs::Pose target2;
const std::string t1_name = "target1";
const std::string t2_name = "target2";

/***********************************
 * VISUALIZATION
 ***********************************/

/** References
 * https://github.com/ros-visualization/visualization_tutorials/blob/indigo-devel/interactive_marker_tutorials/src/simple_marker.cpp
 * http://wiki.ros.org/rviz/Tutorials/Interactive%20Markers%3A%20Basic%20Controls
 * http://docs.ros.org/en/noetic/api/visualization_msgs/html/msg/InteractiveMarkerControl.html
 */

namespace rvt = rviz_visual_tools;
namespace vmsgs = visualization_msgs;

void processFeedback(const vmsgs::InteractiveMarkerFeedbackConstPtr &feedback)
{
    ROS_INFO_STREAM("Rviz Feedback: " << feedback->marker_name
        << " is now at (" << std::setprecision(2)
        << feedback->pose.position.x << ", "
        << feedback->pose.position.y << ", "
        << feedback->pose.position.z << ") with orientation ("
        << feedback->pose.orientation.x << ", "
        << feedback->pose.orientation.y << ", "
        << feedback->pose.orientation.z << ", "
        << feedback->pose.orientation.w << ")");
    if (t1_name.compare(feedback->marker_name) == 0)
    {
        target1 = feedback->pose;
    }
    else if (t2_name.compare(feedback->marker_name) == 0)
    {
        target2 = feedback->pose;
    }
}

vmsgs::Marker makeBox(vmsgs::InteractiveMarker &msg)
{
    vmsgs::Marker marker;
    marker.type = vmsgs::Marker::CUBE;
    marker.scale.x = msg.scale * 0.45;
    marker.scale.y = msg.scale * 0.45;
    marker.scale.z = msg.scale * 0.45;
    marker.color.r = 0.5;
    marker.color.g = 0.5;
    marker.color.b = 0.5;
    marker.color.a = 0.5;
    return marker;
}

vmsgs::InteractiveMarkerControl& makeBoxControl(vmsgs::InteractiveMarker &msg)
{
    vmsgs::InteractiveMarkerControl control;
    control.always_visible = true;
    control.markers.push_back(makeBox(msg));
    control.orientation.x = 0.0;
    control.orientation.y = 0.0;
    control.orientation.z = 0.0;
    control.orientation.w = 1.0;
    msg.controls.push_back(control);
    return msg.controls.back();
}

void makeRoundTripMarker(
    interactive_markers::InteractiveMarkerServer &server,
    const std::string &int_marker_name,
    const std::string &frame_id,
    const geometry_msgs::Pose &pose,
    const double& marker_scale)
{
    vmsgs::InteractiveMarker int_marker;
    int_marker.header.frame_id = frame_id;
    int_marker.name = int_marker_name;
    int_marker.description = "Round-trip Pose Target";
    int_marker.pose = pose;
    int_marker.scale = marker_scale;
    // Insert a box
    makeBoxControl(int_marker);
    int_marker.controls[0].interaction_mode = vmsgs::InteractiveMarkerControl::MOVE_3D;
    // Add 6 DoF control axes
    {
        /** Available interaction modes
         * 2D modes
         *      MOVE_AXIS: Translate along local x-axis.
         *      MOVE_PLANE: Translate in local y-z plane.
         *      ROTATE_AXIS: Rotate around local x-axis.
         *      MOVE_ROTATE: Combines MOVE_PLANE and ROTATE_AXIS.
         * 3D modes
         *      MOVE_3D: Translate freely in 3D space.
         *      ROTATE_3D: Rotate freely in 3D space about the origin of parent frame.
         *      MOVE_ROTATE_3D: MOVE_3D (default) + ROTATE_3D (while holding ctrl)
         */
        vmsgs::InteractiveMarkerControl control;
        // X axis
        control.orientation.w = 1;
        control.orientation.x = 1;
        control.orientation.y = 0;
        control.orientation.z = 0;
        control.name = "rotate_x";
        control.interaction_mode = vmsgs::InteractiveMarkerControl::ROTATE_AXIS;
        int_marker.controls.push_back(control);
        control.name = "move_x";
        control.interaction_mode = vmsgs::InteractiveMarkerControl::MOVE_AXIS;
        int_marker.controls.push_back(control);
        // Y axis
        control.orientation.w = 1;
        control.orientation.x = 0;
        control.orientation.y = 1;
        control.orientation.z = 0;
        control.name = "rotate_z";
        control.interaction_mode = vmsgs::InteractiveMarkerControl::ROTATE_AXIS;
        int_marker.controls.push_back(control);
        control.name = "move_z";
        control.interaction_mode = vmsgs::InteractiveMarkerControl::MOVE_AXIS;
        int_marker.controls.push_back(control);
        // Z axis
        control.orientation.w = 1;
        control.orientation.x = 0;
        control.orientation.y = 0;
        control.orientation.z = 1;
        control.name = "rotate_y";
        control.interaction_mode = vmsgs::InteractiveMarkerControl::ROTATE_AXIS;
        int_marker.controls.push_back(control);
        control.name = "move_y";
        control.interaction_mode = vmsgs::InteractiveMarkerControl::MOVE_AXIS;
        int_marker.controls.push_back(control);
    }
    // Add into the server
    server.insert(int_marker, &processFeedback);
}

/***********************************
 * KINEMATICS
 ***********************************/

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
    ros::init(argc, argv, "singularity");
    ros::NodeHandle nh("/singularity");
    ros::AsyncSpinner spinner(1);
    spinner.start();

    // rosrun kinematics_demo singularity _robot:=puma_560
    std::string planning_group;
    if (nh.getParam("robot", planning_group))
    {
        ROS_INFO_STREAM("Using planning group: " << planning_group);
    }
    else
    {
        ROS_ERROR_STREAM("No planning group specified");
        return 1;
    }

    // Setup for MoveIt
    moveit::planning_interface::MoveGroupInterface move_group(planning_group);
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    robot_state::RobotStatePtr kinematic_state(move_group.getCurrentState());
    const robot_state::JointModelGroup *joint_model_group = kinematic_state->getJointModelGroup(planning_group);

    move_group.setWorkspace(-2, -2, -0.5, 2, 2, 3);
    const std::string frame_id = move_group.getPlanningFrame();
    const std::string eef_link = move_group.getEndEffectorLink();
    ROS_INFO_STREAM("Using frame_id: " << frame_id);

    // Initial end-effector(tool tip center) pose
    geometry_msgs::Pose zero_pose;
    std::vector<double> zero_joints(joint_model_group->getVariableCount(), 0.0);
    calcFK(zero_joints, kinematic_state, joint_model_group, eef_link, zero_pose);

    // Round-trip pose target (rviz interactive markers)
    interactive_markers::InteractiveMarkerServer server("round_trip_targets");
    {
        // Interactive marker for the round-trip pose target
        geometry_msgs::Pose pose1 = zero_pose;
        pose1.position.y += 0.5;
        makeRoundTripMarker(server, t1_name, frame_id, pose1, 0.2);
        geometry_msgs::Pose pose2 = zero_pose;
        pose2.position.y -= 0.5;
        makeRoundTripMarker(server, t2_name, frame_id, pose2, 0.2);
    }
    server.applyChanges();



    // Fake joint states to fool the MoveIt
    ros::Publisher joint_state_pub =
        nh.advertise<sensor_msgs::JointState>("/move_group/fake_controller_joint_states", 1);

    // Fake joint message
    sensor_msgs::JointState js_msg;
    js_msg.header.frame_id = frame_id;
    js_msg.name = move_group.getJointNames();
    js_msg.position.resize(js_msg.name.size());

    ros::Rate rate(3);
    while (ros::ok())
    {
        // Set joint positions
        // joint_state_pub.publish(js_msg);

        ros::spinOnce();
        rate.sleep();
    }
    ros::waitForShutdown();
    return 0;
}