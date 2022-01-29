#include <map>
#include <iomanip>
#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <rviz_visual_tools/rviz_visual_tools.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <moveit_msgs/JointLimits.h>
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

class JointTreversal
{
public:
    JointTreversal(
        const std::vector<const robot_state::JointModel*> &jmv,
        const std::map<robot_state::JointModel::JointType, double> &jlims):
        joint_model_vector(jmv), resolution(jlims)
    {
        joint_limits.resize(jmv.size());
        joint_resolution.resize(jmv.size());
        joint_cases_vector.resize(jmv.size(), 0);
        joint_index_vector.resize(jmv.size(), 0);
        joint_values.resize(jmv.size());
        _calcNumTotalCases();  // Initialized here
    }
    ~JointTreversal()
    {}

    void moveToNextSample()
    {
        // {
        //     std::stringstream ss;
        //     ss << "joint_cases_vector: (";
        //     for (int i = 0; i < joint_cases_vector.size(); i++)
        //     {
        //         ss << joint_cases_vector[i] << ", ";
        //     }
        //     ss << ")";
        //     ROS_INFO_STREAM(ss.str());
        // }
        // {
        //     std::stringstream ss;
        //     ss << "Before Joint index vector: (";
        //     for (int i = 0; i < joint_index_vector.size(); i++)
        //     {
        //         ss << joint_index_vector[i] << ", ";
        //     }
        //     ss << ") count: " << count;
        //     ROS_INFO_STREAM(ss.str());
        // }
        _recurseMoveTo(joint_values.size() - 1);
        count++;
        // {
        //     std::stringstream ss;
        //     ss << " After Joint index vector: (";
        //     for (int i = 0; i < joint_index_vector.size(); i++)
        //     {
        //         ss << joint_index_vector[i] << ", ";
        //     }
        //     ss << ") count: " << count <<"\n";
        //     ROS_INFO_STREAM(ss.str());
        //     // // Pause for debugging
        //     // char kk;
        //     // std::cout << "Press any key to continue...";
        //     // std::cin >> kk;
        // }
    }

    const std::vector<double>& getJointValues() {return joint_values;}
    bool isAvailable() {return initialized;}
    bool workRemains() {return count < num_total_cases;}
    float getProgress() {return 100.0 * ((float)count / (float)num_total_cases);}

private:
    void _resetJointValues(const std::size_t start_idx)
    {
        for (std::size_t i = start_idx; i < joint_values.size(); i++)
        {
            joint_index_vector[i] = 0;
            joint_values[i] = joint_limits[i].min_position;
        }
    }

    void _recurseMoveTo(std::size_t joint_idx)
    {
        std::size_t sample_idx = joint_index_vector[joint_idx] + 1;
        if (sample_idx < joint_cases_vector[joint_idx])
        {
            _resetJointValues(joint_idx + 1);
            double sample_value = joint_values[joint_idx] + joint_resolution[joint_idx];
            joint_values[joint_idx] = (sample_value > joint_limits[joint_idx].max_position) ?
                joint_limits[joint_idx].max_position : sample_value;
            joint_index_vector[joint_idx] = sample_idx;
        }
        else
        {
            if (joint_idx > 0) {_recurseMoveTo(joint_idx - 1);}
        }
    }

    void _calcNumTotalCases()
    {
        std::size_t idx = 0;
        // Number of cases to explore
        unsigned long long num_cases = 1;
        initialized = true;
        for (const robot_state::JointModel *jm : joint_model_vector)
        {
            moveit_msgs::JointLimits jl = jm->getVariableBoundsMsg().at(0);
            if (!jl.has_position_limits)
            {
                ROS_WARN_STREAM("Joint [ " << jm->getName() << " ] has no position limits");
                if (jm->getType() == robot_state::JointModel::REVOLUTE)
                {
                    ROS_WARN_STREAM("but since it is a revolute, set the limit to [-pi, pi]");
                    jl.min_position = -M_PI;
                    jl.max_position = M_PI;
                }
                else
                {
                    ROS_ERROR_STREAM("It is not supported because it is NOT a revolute type.");
                    initialized = false;
                }
            }

            double resol = resolution.at(jm->getType());
            unsigned long long num_cases_joint =
                (unsigned long long)std::ceil((jl.max_position - jl.min_position) / resol) + 1;
            num_cases *= num_cases_joint;
            ROS_INFO_STREAM("Joint [ " << jm->getName() << " ] has " << num_cases_joint << " cases");

            // Initialization
            joint_limits[idx] = jl;
            joint_resolution[idx] = resol;
            joint_cases_vector[idx] = num_cases_joint;
            joint_index_vector[idx] = 0;
            joint_values[idx] = jl.min_position;
            idx++;
        }
        num_total_cases = num_cases;
        ROS_WARN_STREAM("Number of cases to explore: " << num_cases);
    }

    const std::vector<const robot_state::JointModel*> joint_model_vector;
    const std::map<robot_state::JointModel::JointType, double> resolution;

    bool initialized = false;
    unsigned long long num_total_cases = 0;
    unsigned long long count = 0;

    std::vector<moveit_msgs::JointLimits> joint_limits;
    std::vector<double> joint_resolution;

    std::vector<std::size_t> joint_cases_vector;
    std::vector<std::size_t> joint_index_vector;
    std::vector<double> joint_values;
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "reachable_ws_fk");
    ros::NodeHandle nh("/reachable_ws_fk");
    ros::AsyncSpinner spinner(1);
    spinner.start();

    // Resolution of the configuration space
    double revolute_resolution_deg_;
    double prismatic_resolution_m;
    double marker_scale;
    std::string planning_group;
    nh.param<double>("revolute_resolution_deg", revolute_resolution_deg_, 30);
    nh.param<double>("prismatic_resolution_m", prismatic_resolution_m, 0.02);
    nh.param<double>("marker_scale", marker_scale, 0.01);
    nh.param<std::string>("planning_group", planning_group, "scara");
    double revolute_resolution_rad = revolute_resolution_deg_ * M_PI / 180.0;

    std::map<robot_state::JointModel::JointType, double> resolution;
    resolution[robot_state::JointModel::REVOLUTE] = revolute_resolution_rad;
    resolution[robot_state::JointModel::PRISMATIC] = prismatic_resolution_m;
    // Print resolution of each type
    for (auto it = resolution.begin(); it != resolution.end(); ++it)
    {
        ROS_WARN_STREAM("Joint type " << it->first << " has resolution " << it->second);
    }

    // Create a MoveGroup
    moveit::planning_interface::MoveGroupInterface move_group(planning_group);
    robot_state::RobotStatePtr kinematic_state(move_group.getCurrentState());
    const robot_state::JointModelGroup *joint_model_group =
        kinematic_state->getJointModelGroup(planning_group);
    const std::string eef_link = move_group.getEndEffectorLink();
    const std::string base_link = move_group.getPlanningFrame();

    // Visualization
    rvt::RvizVisualToolsPtr visual_tools = rvt::RvizVisualToolsPtr(
        new rvt::RvizVisualTools(base_link, "/rviz_visual_tools"));
    visual_tools->deleteAllMarkers();
    visual_tools->enableBatchPublishing();
    visual_tools->setAlpha(0.5);

    // Get joint information
    const std::vector<const robot_state::JointModel*> &joint_model_vector =
        joint_model_group->getActiveJointModels();

    // Get joint limits
    for (const robot_state::JointModel *jm : joint_model_vector)
    {
        moveit_msgs::JointLimits jl = jm->getVariableBoundsMsg().at(0);
        std::string limit_info = jl.has_position_limits ?
            "Position limits " + std::to_string(jl.min_position) + " to " + std::to_string(jl.max_position) :
            "No position limits";
        ROS_INFO_STREAM("Joint [ " << jm->getName() << " ] has type " << jm->getType() << "(" << jm->getTypeName() << ") with " << limit_info);
    }
    ROS_INFO_STREAM("");

    // Joint treversal object
    JointTreversal jt(joint_model_vector, resolution);
    if (!jt.isAvailable()) { return 1; }

    // Find reachable workspace
    std::size_t count = 0;
    while (jt.workRemains())
    {
        // Get joint values
        const std::vector<double> joint_values = jt.getJointValues();
        jt.moveToNextSample();

        // FK
        geometry_msgs::Pose eef_pose;
        calcFK(joint_values, kinematic_state, joint_model_group, eef_link, eef_pose);

        // Visualization
        visual_tools->publishSphere(eef_pose, rvt::BLUE, marker_scale);
        count++;

        // Print progress
        if (count > 100)
        {
            visual_tools->trigger();
            float process = jt.getProgress();
            ROS_INFO_STREAM("Process: " << std::fixed << std::setprecision(3) << process << "%");
            count = 0;
            ros::Duration(0.5).sleep();
        }
    }
    visual_tools->trigger();
    ROS_INFO_STREAM("Process: 100%");
    ROS_WARN_STREAM("Reachable workspace found!");
    ros::waitForShutdown();
    return 0;
}
