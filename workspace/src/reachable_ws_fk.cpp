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

void prepareSphere(const rvt::RvizVisualToolsPtr &visual_tools, const geometry_msgs::Pose &pose)
{
    const double scale = 0.1;
    visual_tools->publishSphere(pose, rvt::BLUE, scale);
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
        joint_resolution.resize(jmv.size()); // Must be initialized with min.

        joint_index_vector.resize(jmv.size(), 0);
        joint_values.resize(jmv.size());     // Must be initialized with min.
        calcNumTotalCases();
    }
    ~JointTreversal()
    {}

    void resetJointValues(const std::size_t start_idx)
    {
        for (std::size_t i = start_idx; i < joint_values.size(); i++)
        {
            joint_index_vector[i] = 0;
            joint_values[i] = joint_limits[i].min_position;
        }
    }

    void moveToNextSample()
    {
        std::size_t joint_idx = joint_values.size() - 1;
        std::size_t sample_idx = joint_index_vector[joint_idx] + 1;
        double sample_value = joint_values[joint_idx] + joint_resolution[joint_idx];
        if (sample_value > joint_limits[joint_idx].max_position)
        {
            // sample_value = joint_limits[joint_idx].min_position;
            // sample_idx = 0;
            // joint_idx--;
        }


    }


    // std::vector<std::vector<double>> joint_values(joint_model_vector.size());
    // // std::size_t index = 0;
    // for (const robot_state::JointModel *jm : joint_model_vector)
    // {
    //     moveit_msgs::JointLimits jl = jm->getVariableBoundsMsg().at(0);
    //     if (!jl.has_position_limits)
    //     {
    //         ROS_ERROR_STREAM("Joint [ " << jm->getName() << " ] has no position limits");
    //         return 1;
    //     }
    //     // std::vector<double>& jv = joint_values[index];
    //     // index++;
    //     // double value = jl.min_position;
    //     double resol = resolution[jm->getType()];
    //     // while (value < jl.max_position)
    //     // {
    //     //     jv.push_back(value);
    //     //     value += resol;
    //     // }
        // jv.push_back(jl.max_position);


    const std::vector<double>& getJointValues() {return joint_values;}
    bool isAvailable() {return initialized;}
    bool workRemains() {return count < num_total_cases;}
    float getProgress() {return 100.0 * ((float)count / (float)num_total_cases);}

private:
    void calcNumTotalCases()
    {
        std::size_t idx = 0;
        // Number of cases to explore
        unsigned long long num_cases = 1;
        for (const robot_state::JointModel *jm : joint_model_vector)
        {
            moveit_msgs::JointLimits jl = jm->getVariableBoundsMsg().at(0);
            if (!jl.has_position_limits)
            {
                ROS_ERROR_STREAM("Joint [ " << jm->getName() << " ] has no position limits");
                initialized = false;
            }
            double resol = resolution.at(jm->getType());
            unsigned long long num_cases_joint =
                (unsigned long long)std::ceil((jl.max_position - jl.min_position) / resol) + 1;
            num_cases *= num_cases_joint;
            ROS_INFO_STREAM("Joint [ " << jm->getName() << " ] has " << num_cases_joint << " cases");
            // Initialization
            joint_limits[idx] = jl;
            joint_resolution[idx] = resol;
            joint_index_vector[idx] = 0;
            joint_values[idx] = jl.min_position;
        }
        num_total_cases = num_cases;
        ROS_WARN_STREAM("Number of cases to explore: " << num_cases);
        initialized = true;
    }

    const std::vector<const robot_state::JointModel*> joint_model_vector;
    const std::map<robot_state::JointModel::JointType, double> resolution;

    bool initialized = false;
    unsigned long long num_total_cases = 0;
    unsigned long long count = 0;

    std::vector<moveit_msgs::JointLimits> joint_limits;
    std::vector<double> joint_resolution;

    std::vector<std::size_t> joint_index_vector;
    std::vector<double> joint_values;
};

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

    std::map<robot_state::JointModel::JointType, double> resolution;
    resolution[robot_state::JointModel::REVOLUTE] = revolute_resolution_rad;
    resolution[robot_state::JointModel::PRISMATIC] = prismatic_resolution_m;
    // Print resolution of each type
    for (auto it = resolution.begin(); it != resolution.end(); ++it)
    {
        ROS_WARN_STREAM("Joint type " << it->first << " has resolution " << it->second);
    }

    // Visualization
    rvt::RvizVisualToolsPtr visual_tools = rvt::RvizVisualToolsPtr(
        new rvt::RvizVisualTools("map", "/rviz_visual_markers"));
    visual_tools->deleteAllMarkers();
    visual_tools->setAlpha(0.5);

    // Create a MoveGroup
    moveit::planning_interface::MoveGroupInterface move_group(planning_group);
    robot_state::RobotStatePtr kinematic_state(move_group.getCurrentState());
    const robot_state::JointModelGroup *joint_model_group =
        kinematic_state->getJointModelGroup(planning_group);
    const std::string eef_link = move_group.getEndEffectorLink();

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
    ros::Time check_time = ros::Time::now();
    double dt = 1.0; // seconds
    while (jt.workRemains())
    {
        // Get joint values
        const std::vector<double> joint_values = jt.getJointValues();
        jt.moveToNextSample();

        // FK
        geometry_msgs::Pose eef_pose;
        calcFK(joint_values, kinematic_state, joint_model_group, eef_link, eef_pose);

        // Visualization
        prepareSphere(visual_tools, eef_pose);

        // Print progress
        if ((ros::Time::now() - check_time).toSec() > dt)
        {
            visual_tools->trigger();
            float process = jt.getProgress();
            ROS_INFO_STREAM("Process: " << std::fixed << std::setprecision(3) << process << "%");
            check_time = ros::Time::now();
        }
    }
    visual_tools->trigger();

    ros::waitForShutdown();
    return 0;
}
