#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_eigen/tf2_eigen.h>
#include <traj_plan/JointInterpolation.h>

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
    ros::init(argc, argv, "fk_node");
    ros::NodeHandle nh("/fk_node");
    ros::AsyncSpinner spinner(4); // Use 4 threads
    spinner.start();
    static const std::string LOGNAME = "fk_node";

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

    // Waypoints: Sqaure trajectory (Joint-space)
    std::vector<std::vector<double>> joint_waypoints;
    // Corresponding Cartesian waypoints
    std::vector<geometry_msgs::Pose> cartesian_waypoints;
    if (planning_group.compare("puma_560") == 0)
    {
        joint_waypoints = {
            {-0.158621, 1.182338, -0.839760, 0.000001, 0.342578, -0.158622}, // way0
            {-0.070632, 0.628602, 0.316969, 0.000000, 0.945571, -0.070632},  // way1
            {0.485967, 0.628602, 0.316969, 0.000000, 0.945570, 0.485967},    // way2
            {1.017371, 1.182317, -0.839752, 0.000000, 0.342565, 1.017371}    // way3
        };
        geometry_msgs::Pose cartesian_way1;
        cartesian_way1.position.x = 0.3;
        cartesian_way1.position.y = -0.2;
        cartesian_way1.position.z = 0.6;
        tf2::Quaternion target_quat(tf2::Vector3(1.0, 0.0, 0.0), M_PI);
        cartesian_way1.orientation = tf2::toMsg(target_quat);
        cartesian_waypoints.push_back(geometry_msgs::Pose(cartesian_way1));
        cartesian_waypoints.push_back(geometry_msgs::Pose(cartesian_way1));
        cartesian_waypoints.push_back(geometry_msgs::Pose(cartesian_way1));
        cartesian_waypoints.push_back(geometry_msgs::Pose(cartesian_way1));
        cartesian_waypoints[1].position.x += 0.4;
        cartesian_waypoints[2].position.x += 0.4;
        cartesian_waypoints[2].position.y += 0.4;
        cartesian_waypoints[3].position.y += 0.4;
    }
    else if (planning_group.compare("rrr") == 0)
    {
        joint_waypoints = {
            {0.595, -1.951, 1.356},  // way0
            {0.064, -0.565, 0.501},  // way1
            {0.501, -0.565, 0.064},  // way2
            {1.356, -1.951, 0.595}   // way3
        };
        geometry_msgs::Pose cartesian_way1;
        cartesian_way1.position.x = 0.25;
        cartesian_way1.position.y = -0.1;
        cartesian_way1.position.z = 0.02;
        cartesian_way1.orientation.w = 1.0;
        cartesian_waypoints.push_back(geometry_msgs::Pose(cartesian_way1));
        cartesian_waypoints.push_back(geometry_msgs::Pose(cartesian_way1));
        cartesian_waypoints.push_back(geometry_msgs::Pose(cartesian_way1));
        cartesian_waypoints.push_back(geometry_msgs::Pose(cartesian_way1));
        cartesian_waypoints[1].position.x += 0.2;
        cartesian_waypoints[2].position.x += 0.2;
        cartesian_waypoints[2].position.y += 0.2;
        cartesian_waypoints[3].position.y += 0.2;
    }
    else
    {
        ROS_ERROR_STREAM_NAMED(LOGNAME, "Unsupported planning group: " << planning_group);
        return 1;
    }
    int num_waypoints = joint_waypoints.size();

    // For waypoint visualization
    std::vector<geometry_msgs::Pose> straight_line = cartesian_waypoints;
    straight_line.push_back(cartesian_waypoints[0]);

    // Visualization
    std::string base_frame = "link1";
    moveit_visual_tools::MoveItVisualTools visual_tools(base_frame);
    visual_tools.deleteAllMarkers();
    visual_tools.loadRemoteControl();

    // Setup for planning
    moveit::planning_interface::MoveGroupInterface move_group(planning_group);
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    robot_state::RobotStatePtr kinematic_state(move_group.getCurrentState());
    const robot_state::JointModelGroup *joint_model_group = kinematic_state->getJointModelGroup(planning_group);

    // Interpolation
    ros::ServiceClient client = nh.serviceClient<traj_plan::JointInterpolation>(
        "/traj_plan/spline/joint_trajectory_service");

    // Placeholder for joint values
    std::vector<double> current_joints(joint_model_group->getVariableCount());

    // Demo
    int i = 0;
    while (ros::ok())
    {
        ROS_INFO("< Planning to waypoint %d >", i);
        current_joints = move_group.getCurrentJointValues();

        // Forward kinematics validation
        const std::string &eef_name = joint_model_group->getLinkModelNames().back();
        kinematic_state->setJointGroupPositions(joint_model_group, joint_waypoints[i]);
        Eigen::Affine3d eef_transformation = kinematic_state->getGlobalLinkTransform(eef_name);
        geometry_msgs::Pose eef_pose = Eigen::toMsg(eef_transformation);
        ROS_INFO_STREAM("EEF pose of waypoint" << i << " : " << eef_pose);

        // Cubic interpolation
        trajectory_msgs::JointTrajectory req_waypoints;
        req_waypoints.joint_names = joint_model_group->getActiveJointModelNames();
        req_waypoints.points.push_back(trajectory_msgs::JointTrajectoryPoint());
        req_waypoints.points.push_back(trajectory_msgs::JointTrajectoryPoint());
        req_waypoints.points[0].positions = current_joints;
        req_waypoints.points[1].positions = joint_waypoints[i]; // target_joints

        ROS_INFO("Wait for service: /traj_plan/spline/joint_trajectory_service");
        client.waitForExistence();

        traj_plan::JointInterpolation srv;
        srv.request.waypoints = req_waypoints;
        ROS_INFO("Calling service");
        ros::Time start_time = ros::Time::now();
        if (client.call(srv))
        {
            ROS_INFO("< Successfully interpolated to waypoint %d >", i);
        }
        else
        {
            ROS_ERROR("< Failed to interpolate to waypoint %d >", i);
            return 1;
        }
        ros::Time end_time = ros::Time::now();

        // Motion
        moveit::planning_interface::MoveGroupInterface::Plan my_plan;
        my_plan.trajectory_.joint_trajectory = srv.response.result;

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
            visual_tools.publishAxisLabeled(cartesian_waypoints[i], "waypoint" + std::to_string(i));
        }
        visual_tools.publishPath(straight_line, rvt::RED, rvt::SMALL);
        visual_tools.trigger();

        // Result
        ROS_INFO("Planning successful");
        ROS_INFO("Planning time: %.4f sec", end_time.toSec() - start_time.toSec());
        move_group.execute(my_plan);
        ROS_INFO("Moving successful");

        // Last joints
        trajectory_msgs::JointTrajectoryPoint last_point = my_plan.trajectory_.joint_trajectory.points.back();
        for (std::size_t i = 0; i < last_point.positions.size(); ++i)
        {
            ROS_INFO("\t(Current) Joint %d: %f", i, last_point.positions[i]);
        }

        // Indexing
        i = (i + 1) % num_waypoints;
        ROS_INFO("-----------------\n");
    }
    ros::waitForShutdown();
    return 0;
}