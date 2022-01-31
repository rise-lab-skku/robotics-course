#include <iomanip>
#include <vector>
#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
// #include <moveit_visual_tools/moveit_visual_tools.h>
#include <geometry_msgs/Pose.h>
#include <sensor_msgs/JointState.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/InteractiveMarkerControl.h>
#include <interactive_markers/interactive_marker_server.h>
#include <tf2_eigen/tf2_eigen.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf/tf.h>

// Global variables to make this code easier
bool is_global_initialized = false;
geometry_msgs::Pose eef_target1;
geometry_msgs::Pose eef_target2;
const std::string t1_name = "eef_target1";
const std::string t2_name = "eef_target2";

void debugPause()
{
    ROS_WARN_STREAM("Press ENTER to continue...");
    std::cin.ignore();
}

/***********************************
 * VISUALIZATION
 ***********************************/

/** Code References
 * https://github.com/ros-visualization/visualization_tutorials/blob/indigo-devel/interactive_marker_tutorials/src/simple_marker.cpp
 * http://wiki.ros.org/rviz/Tutorials/Interactive%20Markers%3A%20Basic%20Controls
 * http://docs.ros.org/en/noetic/api/visualization_msgs/html/msg/InteractiveMarkerControl.html
 */

// namespace rvt = rviz_visual_tools;
namespace vmsgs = visualization_msgs;

namespace visualization
{
    void processFeedback(const vmsgs::InteractiveMarkerFeedbackConstPtr &feedback)
    {
        ROS_INFO_STREAM("Rviz Feedback: " << feedback->marker_name
            << " is now at XYZ(" << std::setprecision(2)
            << feedback->pose.position.x << ", "
            << feedback->pose.position.y << ", "
            << feedback->pose.position.z << ") WXYZ("
            << feedback->pose.orientation.w << ", "
            << feedback->pose.orientation.x << ", "
            << feedback->pose.orientation.y << ", "
            << feedback->pose.orientation.z << ")");
        if (t1_name.compare(feedback->marker_name) == 0)
        {
            eef_target1 = feedback->pose;
        }
        else if (t2_name.compare(feedback->marker_name) == 0)
        {
            eef_target2 = feedback->pose;
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
        const std::string &description,
        const std::string &frame_id,
        const geometry_msgs::Pose &pose,
        const double& marker_scale)
    {
        vmsgs::InteractiveMarker int_marker;
        int_marker.name = int_marker_name;
        int_marker.description = description;
        int_marker.header.frame_id = frame_id;
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
}

/***********************************
 * KINEMATICS
 ***********************************/

namespace kinematics
{
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

    geometry_msgs::Pose dPose(
        const geometry_msgs::Pose &target,
        const geometry_msgs::Pose &ref)
    {
        tf::Transform target_tf(
            tf::Quaternion(target.orientation.x, target.orientation.y, target.orientation.z, target.orientation.w),
            tf::Vector3(target.position.x, target.position.y, target.position.z));
        tf::Transform ref_tf(
            tf::Quaternion(ref.orientation.x, ref.orientation.y, ref.orientation.z, ref.orientation.w),
            tf::Vector3(ref.position.x, ref.position.y, ref.position.z));
        // target - ref
        tf::Transform d_tf = ref_tf.inverse() * target_tf;
        geometry_msgs::Pose d_pose;
        tf::poseTFToMsg(d_tf, d_pose);
        return d_pose;
    }

    /** This method is also called damped least squares method.
     * Copied from STOMP
     * http://docs.ros.org/en/kinetic/api/stomp_moveit/html/namespacestomp__moveit_1_1utils_1_1kinematics.html#a1a46c199beea4b6d10f18f9c709ebdef
     */
    void calculateDampedPseudoInverse(
        const Eigen::MatrixXd &jacb, Eigen::MatrixXd &jacb_pseudo_inv, double eps, double lambda)
    {
        using namespace Eigen;
        //Calculate A+ (pseudoinverse of A) = V S+ U*, where U* is Hermition of U (just transpose if all values of U are real)
        //in order to solve Ax=b -> x*=A+ b
        Eigen::JacobiSVD<MatrixXd> svd(jacb, Eigen::ComputeThinU | Eigen::ComputeThinV);
        const MatrixXd &U = svd.matrixU();
        const VectorXd &Sv = svd.singularValues();
        const MatrixXd &V = svd.matrixV();

        // calculate the reciprocal of Singular-Values
        // damp inverse with lambda so that inverse doesn't oscillate near solution
        size_t nSv = Sv.size();
        VectorXd inv_Sv(nSv);
        for(size_t i=0; i< nSv; ++i)
        {
            // ROS_WARN_STREAM("fabs(Sv(i)): " << fabs(Sv(i)) << " eps: " << eps);
            // ROS_INFO_STREAM("1/Sv(i): " << 1.0/Sv(i));
            // ROS_INFO_STREAM("Sv(i)/(Sv*Sv+lmb*lmb): " << Sv(i) / (Sv(i)*Sv(i) + lambda*lambda) << " lambda: " << lambda);
            // debugPause();
            if (fabs(Sv(i)) > eps)
            {
                inv_Sv(i) = 1/Sv(i);
            }
            else
            {
                inv_Sv(i) = Sv(i) / (Sv(i)*Sv(i) + lambda*lambda);
            }
        }
        jacb_pseudo_inv = V * inv_Sv.asDiagonal() * U.transpose();
    }

    /**
     * Copied from STOMP
     * http://docs.ros.org/en/indigo/api/stomp_moveit/html/namespacestomp__moveit_1_1utils_1_1kinematics.html#a14b644b93916381e79420d4e5ec4ea2c
     */
    static void reduceJacobian(const Eigen::MatrixXd& jacb,
        const std::vector<int>& indices,Eigen::MatrixXd& jacb_reduced)
    {
        jacb_reduced.resize(indices.size(),jacb.cols());
        for(auto i = 0u; i < indices.size(); i++)
        {
            jacb_reduced.row(i) = jacb.row(indices[i]);
        }
    }

    /**
     * Copied from STOMP
     * http://docs.ros.org/en/kinetic/api/stomp_moveit/html/namespacestomp__moveit_1_1utils_1_1kinematics.html#a20302c0200bda263138abeda4e91d0f4
     */
    bool computeJacobianNullSpace(moveit::core::RobotStatePtr state,std::string group,std::string tool_link,
                                         const Eigen::ArrayXi& constrained_dofs,const Eigen::VectorXd& joint_pose,
                                         Eigen::MatrixXd& jacb_nullspace)
    {
        using namespace Eigen;
        using namespace moveit::core;

        // robot state
        const JointModelGroup* joint_group = state->getJointModelGroup(group);
        state->setJointGroupPositions(joint_group,joint_pose);
        Affine3d tool_pose = state->getGlobalLinkTransform(tool_link);

        // jacobian calculations
        static MatrixXd jacb_transform(6,6);
        MatrixXd jacb, jacb_reduced, jacb_pseudo_inv;
        jacb_transform.setZero();

        if(!state->getJacobian(joint_group,state->getLinkModel(tool_link),Vector3d::Zero(),jacb))
        {
        ROS_ERROR("Failed to get Jacobian for link %s",tool_link.c_str());
        return false;
        }

        // transform jacobian rotational part to tool coordinates
        auto rot = tool_pose.inverse().rotation();
        jacb_transform.setZero();
        jacb_transform.block(0,0,3,3) = rot;
        jacb_transform.block(3,3,3,3) = rot;
        jacb = jacb_transform*jacb;

        // reduce jacobian and compute its pseudo inverse
        std::vector<int> indices;
        for(auto i = 0u; i < constrained_dofs.size(); i++)
        {
            if(constrained_dofs(i) != 0) {indices.push_back(i);}
        }
        reduceJacobian(jacb,indices,jacb_reduced);
        double EPSILON = 0.011;
        double LAMBDA = 0.01;
        calculateDampedPseudoInverse(jacb_reduced,jacb_pseudo_inv,EPSILON,LAMBDA);
        int num_joints = joint_pose.size();
        jacb_nullspace = MatrixXd::Identity(num_joints,num_joints) - jacb_pseudo_inv*jacb_reduced;
        return true;
    }

}

class LocalTarget
{
public:
    LocalTarget(const geometry_msgs::Pose &pose, double linear_vel, double dquat_rot_vel):
        pose_(pose), linear_vel_(linear_vel), dquat_rot_vel_(dquat_rot_vel)
    {
        waitForGlobalInitialization_(); // Wait for the interactive markers
        reset_(1);
    };
    ~LocalTarget() {};

    geometry_msgs::Pose getPose() {return pose_;}

    void updatePose()
    {
        // Check the local target is close to the global target
        if (isClose_()) {reset_((global_target_id_ + 1) % 2);}
        // slerp_t
        double time_elapsed = (ros::Time::now() - start_time_).toSec();
        double slerp_t = time_elapsed / expected_duration_;
        if (slerp_t > 1.0) {slerp_t = 1.0;}
        // Calculate the new position with linear interpolation
        pose_.position.x = (to_.position.x - from_.position.x) * slerp_t + from_.position.x;
        pose_.position.y = (to_.position.y - from_.position.y) * slerp_t + from_.position.y;
        pose_.position.z = (to_.position.z - from_.position.z) * slerp_t + from_.position.z;
        // Calculate the new orientation with spherical linear interpolation (slerp)
        tf2::Quaternion q1(from_.orientation.x, from_.orientation.y, from_.orientation.z, from_.orientation.w);
        tf2::Quaternion q2(to_.orientation.x, to_.orientation.y, to_.orientation.z, to_.orientation.w);
        tf2::Quaternion q_slerp = tf2::slerp(q1, q2, slerp_t);
        pose_.orientation = tf2::toMsg(q_slerp);
    }

private:
    void waitForGlobalInitialization_()
    {
        while(!is_global_initialized)
        {
            ROS_INFO_STREAM("Waiting for interactive markers initialization...");
            ros::Duration(1.0).sleep();
        }
    }

    bool isClose_()
    {
        const double tolerance = 0.01;
        return (fabs(pose_.position.x - to_.position.x) < tolerance &&
                fabs(pose_.position.y - to_.position.y) < tolerance &&
                fabs(pose_.position.z - to_.position.z) < tolerance &&
                fabs(pose_.orientation.x - to_.orientation.x) < tolerance &&
                fabs(pose_.orientation.y - to_.orientation.y) < tolerance &&
                fabs(pose_.orientation.z - to_.orientation.z) < tolerance &&
                fabs(pose_.orientation.w - to_.orientation.w) < tolerance);
    }

    void reset_(int global_target_id)
    {
        global_target_id_ = global_target_id;
        from_ = pose_;
        if (global_target_id_ == 1) {to_ = eef_target1;}
        else {to_ = eef_target2;}
        start_time_ = ros::Time::now();
        /**
         * Initialize slerp parameters
         */
        geometry_msgs::Pose d_pose = kinematics::dPose(to_, from_);
        // Estimate the required time due to position
        double position_dist = sqrt(
            d_pose.position.x*d_pose.position.x +
            d_pose.position.y*d_pose.position.y +
            d_pose.position.z*d_pose.position.z);
        double position_time = position_dist / linear_vel_;
        // Estimate the required time due to orientation
        tf::Quaternion d_quat(d_pose.orientation.x, d_pose.orientation.y, d_pose.orientation.z, d_pose.orientation.w);
        double quat_angle = d_quat.getAngleShortestPath();
        double quat_time = quat_angle / dquat_rot_vel_;
        // Maximal time
        double max_time = std::max(position_time, quat_time);
        expected_duration_ = max_time;
    }

    // Config
    const double linear_vel_;    // [m/sec]
    const double dquat_rot_vel_; // [rad/sec]

    // Interpolation variables (position: linear, orientation: slerp)
    int global_target_id_;
    geometry_msgs::Pose from_;
    geometry_msgs::Pose to_;
    ros::Time start_time_;
    double expected_duration_;

    // Output
    geometry_msgs::Pose pose_;
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "singularity");
    ros::NodeHandle nh("/singularity");
    ros::AsyncSpinner spinner(1);
    spinner.start();

    // ROS parameters
    std::string planning_group;
    double marker_scale;
    double dls_eps;
    double dls_lambda;
    nh.param<std::string>("robot", planning_group, "puma_560");
    nh.param<double>("marker_scale", marker_scale, 0.1);
    nh.param<double>("epsilon", dls_eps, 1);
    nh.param<double>("lambda", dls_lambda, 20);

    // Setup for MoveIt
    moveit::planning_interface::MoveGroupInterface move_group(planning_group);
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    robot_state::RobotStatePtr kinematic_state(move_group.getCurrentState());
    const robot_state::LinkModel *eef_link = kinematic_state->getLinkModel(move_group.getEndEffectorLink());
    const robot_state::JointModelGroup *joint_model_group = kinematic_state->getJointModelGroup(planning_group);

    move_group.setWorkspace(-2, -2, -0.5, 2, 2, 3);
    const std::string frame_id = move_group.getPlanningFrame();
    const std::string eef_name = move_group.getEndEffectorLink();
    ROS_INFO_STREAM("Using frame_id: " << frame_id);

    // Initial end-effector(tool tip center) pose
    geometry_msgs::Pose eef_pose;
    std::vector<double> zero_joints(joint_model_group->getVariableCount(), 0.0);
    kinematics::calcFK(zero_joints, kinematic_state, joint_model_group, eef_name, eef_pose);

    // Round-trip pose target (rviz interactive markers)
    interactive_markers::InteractiveMarkerServer server("round_trip_targets");
    {
        // Interactive marker for the round-trip pose target
        eef_target1 = eef_pose;
        // eef_target1.position.x -= 0.1;
        // eef_target1.position.y += 0.1;
        visualization::makeRoundTripMarker(
            server, t1_name, "Round-trip Pose Target 1", frame_id, eef_target1, marker_scale);
        eef_target2 = eef_pose;
        // eef_target2.position.x += 0.3;
        visualization::makeRoundTripMarker(
            server, t2_name, "Round-trip Pose Target 2", frame_id, eef_target2, marker_scale);
        is_global_initialized = true;

    }
    server.applyChanges();

    // debugPause();

    // Fake joint states to fool the MoveIt
    ros::Publisher joint_state_pub =
        nh.advertise<sensor_msgs::JointState>("/move_group/fake_controller_joint_states", 1);

    // Current joint state (Fake joint message)
    sensor_msgs::JointState current_joints;
    current_joints.header.frame_id = frame_id;
    current_joints.name = move_group.getJointNames();
    current_joints.position.resize(current_joints.name.size(), 0.0);

    // Round-trip
    double max_linear_vel;    // [m/sec]
    double dquat_rot_deg; // [deg/sec]
    nh.param<double>("max_linear_vel", max_linear_vel, 0.02);
    nh.param<double>("max_dquat_rot_degVel", dquat_rot_deg, 10.0);
    const double max_dquat_rot_vel = dquat_rot_deg * (M_PI / 180.0);
    LocalTarget local_target(eef_pose, max_linear_vel, max_dquat_rot_vel);

    ros::Publisher eef_pub =
        nh.advertise<geometry_msgs::PoseStamped>("/singularity/current_eef", 1);
    ros::Publisher local_target_pub =
        nh.advertise<geometry_msgs::PoseStamped>("/singularity/local_target", 1);

    // Main Loop
    const double rad2deg = 180.0 / M_PI;
    int count = 0;
    ros::Time last_time = ros::Time::now();
    ros::Rate rate(512);
    while (ros::ok())
    {
        // ref_pose from FK
        kinematics::calcFK(
            current_joints.position, kinematic_state, joint_model_group, eef_name, eef_pose);
        geometry_msgs::PoseStamped ref_ps;
        ref_ps.header.frame_id = frame_id;
        ref_ps.header.stamp = ros::Time::now();
        ref_ps.pose = eef_pose;
        eef_pub.publish(ref_ps);

        // target_pose from LocalTarget
        local_target.updatePose();
        geometry_msgs::PoseStamped target_ps;
        target_ps.header.frame_id = frame_id;
        target_ps.header.stamp = ros::Time::now();
        target_ps.pose = local_target.getPose();
        local_target_pub.publish(target_ps);

        // d_error
        geometry_msgs::Pose error_pose = kinematics::dPose(target_ps.pose, eef_pose);
        Eigen::VectorXd d_error(7);  // position, quaternion
        d_error << error_pose.position.x,
                   error_pose.position.y,
                   error_pose.position.z,
                   error_pose.orientation.w,  // w first (since, jacobian_with_quat)
                   error_pose.orientation.x,
                   error_pose.orientation.y,
                   error_pose.orientation.z;
        Eigen::VectorXd d_error_deg = d_error * rad2deg;
        ROS_INFO_STREAM("d_error (deg): " << d_error_deg.transpose());

        /**
         * Jacobian with quaternion:
         * https://docs.ros.org/en/indigo/api/moveit_core/html/robot__state_8cpp_source.html#l01184
         */
        const Eigen::Vector3d reference_point_position(0.0, 0.0, 0.0);
        const bool use_quat_repr = true;
        Eigen::MatrixXd jacobian;
        kinematic_state->getJacobian(
            joint_model_group, eef_link, reference_point_position, jacobian, use_quat_repr);
        // ROS_INFO_STREAM("Jacobian : \n" << jacobian);

        // Damped pseudo-inverse
        Eigen::MatrixXd jacb_pseudo_inv;
        kinematics::calculateDampedPseudoInverse(jacobian, jacb_pseudo_inv, dls_eps, dls_lambda);

        // d_theta = damped_pseudo_inverse * d_error
        Eigen::VectorXd d_theta = jacb_pseudo_inv * d_error;
        Eigen::VectorXd d_theta_deg = d_theta * rad2deg;
        ROS_INFO_STREAM("d_theta (deg): \n" << d_theta_deg.transpose());

        // Nullspace
        // http://docs.ros.org/en/kinetic/api/stomp_moveit/html/namespacestomp__moveit_1_1utils_1_1kinematics.html#a20302c0200bda263138abeda4e91d0f4
        // https://homes.cs.washington.edu/~todorov/courses/cseP590/06_JacobianMethods.pdf
        // int num_joints = current_joints.position.size();
        // Eigen::MatrixXd jacb_nullspace = Eigen::MatrixXd::Identity(num_joints,num_joints) - jacb_pseudo_inv * jacobian;
        // ROS_INFO_STREAM("Nullspace : \n" << jacb_nullspace);
        // Eigen::VectorXd d_theta_nullspace = -d_theta;
        // ROS_INFO_STREAM("d_theta_nullspace : \n" << d_theta_nullspace);
        // Eigen::VectorXd nullspace_theta = (jacb_nullspace * d_theta_nullspace);
        // ROS_INFO_STREAM("Nullspace theta: \n" << nullspace_theta);

        // joint += d_theta
        for (int i = 0; i < current_joints.position.size(); i++)
        {
            const double dmax = 0.05 * M_PI / 180.0;
            if (d_theta[i] > dmax) {d_theta[i] = dmax;}
            else if (d_theta[i] < -dmax) {d_theta[i] = -dmax;}

            current_joints.position[i] += d_theta[i];
        }

        // Set joint positions on the Rviz
        joint_state_pub.publish(current_joints);
        // Update the current joint state in the MoveIt
        kinematic_state->setJointGroupPositions(joint_model_group, current_joints.position);

        // Print status
        {
            ros::Time now = ros::Time::now();
            double hz = 1.0 / (now - last_time).toSec();
            last_time = now;

            std::string j_value_str;
            for (int i = 0; i < current_joints.position.size(); i++)
            {
                j_value_str += "\t" + std::to_string(current_joints.position[i] * rad2deg) + "\n";
            }
            d_theta *= rad2deg;
            ROS_INFO_STREAM("d_theta (deg): " << d_theta.transpose() << " [" << hz << " Hz]\n" <<
                "current joints (deg):\n" << j_value_str);
            // count++;
            // if (count > 20) {debugPause();}
        }
        ros::spinOnce();
        rate.sleep();
    }
    ros::waitForShutdown();
    return 0;
}