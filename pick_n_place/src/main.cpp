#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

namespace rvt = rviz_visual_tools;

int main(int argc, char **argv)
{
  /**
   * The ros::init() function needs to see argc and argv so that it can perform
   * any ROS arguments and name remapping that were provided at the command line.
   * For programmatic remappings you can use a different version of init() which takes
   * remappings directly, but for most command-line programs, passing argc and argv is
   * the easiest way to do it.  The third argument to init() is the name of the node.
   *
   * You must call one of the versions of ros::init() before using any other
   * part of the ROS system.
   */
  ros::init(argc, argv, "my_puma");

  /**
   * NodeHandle is the main access point to communications with the ROS system.
   * The first NodeHandle constructed will fully initialize this node, and the last
   * NodeHandle destructed will close down the node.
   */
  ros::NodeHandle nh("/my_puma");

  // roscpp provides some built-in support for calling callbacks from multiple threads.
  ros::AsyncSpinner spinner(4); // Use 4 threads
  spinner.start();

  // Setup
  // ^^^^^
  // planning group == joint model group
  static const std::string PLANNING_GROUP = "puma_560";
  static const std::string LOGNAME = "my_puma_tutorial";

  ROS_INFO_STREAM_NAMED(LOGNAME, "Starting MoveIt Tutorials...");

  moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

  const robot_state::JointModelGroup *joint_model_group =
      move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);

  // Visualization
  // ^^^^^^^^^^^^^
  moveit_visual_tools::MoveItVisualTools visual_tools("link1");
  visual_tools.deleteAllMarkers();
  visual_tools.loadRemoteControl();

  Eigen::Isometry3d text_pose = Eigen::Isometry3d::Identity();
  text_pose.translation().x() = 0.4;
  text_pose.translation().z() = 1.2;
  visual_tools.publishText(text_pose, "MoveItCpp Demo", rvt::WHITE, rvt::XLARGE);
  visual_tools.trigger();

  ROS_INFO_NAMED("tutorial", "========================================");
  ROS_INFO_NAMED("tutorial", "Planning frame: %s", move_group.getPlanningFrame().c_str());
  ROS_INFO_NAMED("tutorial", "End effector link: %s", move_group.getEndEffectorLink().c_str());
  std::ostringstream oss;
  std::copy(move_group.getJointModelGroupNames().begin(),
            move_group.getJointModelGroupNames().end(),
            std::ostream_iterator<std::string>(oss, ", "));
  ROS_INFO_NAMED("turotial", "Available Planning Groups: %s", oss.str().c_str());
  ROS_INFO_NAMED("tutorial", "========================================");

  geometry_msgs::PoseStamped target_pose1;
  target_pose1.header.frame_id = "link1";
  target_pose1.pose.position.x = 0.4;
  target_pose1.pose.position.y = 0.25;
  target_pose1.pose.position.z = 0.5;
  tf2::Quaternion target_quat(tf2::Vector3(1.0, 0.0, 0.0), M_PI);
  target_pose1.pose.orientation = tf2::toMsg(target_quat);

  geometry_msgs::PoseStamped target_pose2;
  target_pose2.header.frame_id = "link1";
  target_pose2.pose.position.x = 0.4;
  target_pose2.pose.position.y = 0.25;
  target_pose2.pose.position.z = 0.7;
  target_pose2.pose.orientation = tf2::toMsg(target_quat);

  geometry_msgs::PoseStamped target_pose3;
  target_pose3.header.frame_id = "link1";
  target_pose3.pose.position.x = 0.4;
  target_pose3.pose.position.y = -0.25;
  target_pose3.pose.position.z = 0.7;
  target_pose3.pose.orientation = tf2::toMsg(target_quat);

  geometry_msgs::PoseStamped target_pose4;
  target_pose4.header.frame_id = "link1";
  target_pose4.pose.position.x = 0.4;
  target_pose4.pose.position.y = -0.25;
  target_pose4.pose.position.z = 0.5;
  target_pose4.pose.orientation = tf2::toMsg(target_quat);

  std::vector<geometry_msgs::PoseStamped> waypoints;
  waypoints.push_back(target_pose1);
  waypoints.push_back(target_pose2);
  waypoints.push_back(target_pose3);
  waypoints.push_back(target_pose4);
  for (auto &waypoint : waypoints) {
    visual_tools.publishAxis(waypoint.pose);
  }
  visual_tools.trigger();

  int i = 0;
  int direction = -1;
  while (ros::ok())
  {
    ROS_INFO_NAMED("tutorial", "Waypoint %d", i);
    move_group.setPoseTarget(waypoints[i]);
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    bool success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    ROS_INFO_NAMED("tutorial", "Visualizing plan (pose goal) %s", success ? "true" : "false");
    ROS_INFO_NAMED("tutorial", "Planning time: %.2f sec", my_plan.planning_time_);
    move_group.execute(my_plan);

    if (i == 3 || i == 0)
    {
      direction *= -1;
      ROS_INFO_NAMED("tutorial", "Wait 2.0 sec");
      ros::Duration(2.0).sleep();
    }
    i += direction;
  }

  ROS_INFO_STREAM_NAMED(LOGNAME, "Shutting down.");
  ros::waitForShutdown();
  return 0;
}