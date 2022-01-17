/**
 * Fast boundary search for the workspace of a robot.
 * [ DFS -> Octree -> Pseudo marching cubes ]
 */
#include <bitset>
#include <memory>
#include <array>
#include <set>
#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <rviz_visual_tools/rviz_visual_tools.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <geometry_msgs/Point.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_eigen/tf2_eigen.h>

namespace rvt = rviz_visual_tools;

void debugPause()
{
    // User input
    ROS_INFO_STREAM("Please press any key to continue...");
    char _;
    std::cin >> _;
}

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
    // 2 msec is enough. ​Usually, 0.02~0.05 msec is required to solve IK.
    const double timeout = 0.002;
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

void drawCuboidFromAnchor(
    const geometry_msgs::Point &anchor,
    const double width,
    moveit_visual_tools::MoveItVisualTools &visual_tools,
    const rvt::colors color)
{
    geometry_msgs::Pose p;
    p.position = anchor;
    p.orientation.w = 1.0;
    visual_tools.publishCuboid(p, width, width, width, color);
}

namespace Octree
{
    inline unsigned char makeEightIKs(
        const std::bitset<27> &checkpoints,
        const std::size_t ba_index,
        const std::size_t bb_index,
        const std::size_t bc_index,
        const std::size_t bd_index,
        const std::size_t ta_index,
        const std::size_t tb_index,
        const std::size_t tc_index,
        const std::size_t td_index)
    {
        /**
         * { ba, bb, bc, bd, ta, tb, tc, td } eight_iks
         *    7,  6,  5,  4,  3,  2,  1,  0   => Required shift
         */
        return (checkpoints[ba_index] << 7) |
               (checkpoints[bb_index] << 6) |
               (checkpoints[bc_index] << 5) |
               (checkpoints[bd_index] << 4) |
               (checkpoints[ta_index] << 3) |
               (checkpoints[tb_index] << 2) |
               (checkpoints[tc_index] << 1) |
               checkpoints[td_index];
    }

    inline unsigned char makeEightIKs(
        const bool ba, const bool bb, const bool bc, const bool bd,
        const bool ta, const bool tb, const bool tc, const bool td)
    {
        return (ba << 7) | (bb << 6) | (bc << 5) | (bd << 4) | (ta << 3) | (tb << 2) | (tc << 1) | td;
    }

    class Cube
    {
    public:
        Cube() {}
        ~Cube() {}

        void init(
            const geometry_msgs::Point &anchor,
            const double &width,
            const unsigned char &eight_iks)
        {
            anchor_ = anchor;
            width_ = width;
            eight_iks_ = eight_iks;
        }

        void init(
            const geometry_msgs::Point &anchor,
            const double &width,
            const robot_state::RobotStatePtr &kinematic_state,
            const robot_model::JointModelGroup *joint_model_group)
        {
            anchor_ = anchor;
            width_ = width;
            // eight_iks_
            geometry_msgs::Pose eef;
            eef.position = anchor_;
            const bool ba = checkIK(eef, kinematic_state, joint_model_group);
            eef.position.x += width_;
            const bool bb = checkIK(eef, kinematic_state, joint_model_group);
            eef.position.y += width_;
            const bool bc = checkIK(eef, kinematic_state, joint_model_group);
            eef.position.x -= width_;
            const bool bd = checkIK(eef, kinematic_state, joint_model_group);
            eef.position.z += width_;
            const bool td = checkIK(eef, kinematic_state, joint_model_group);
            eef.position.x += width_;
            const bool tc = checkIK(eef, kinematic_state, joint_model_group);
            eef.position.y -= width_;
            const bool tb = checkIK(eef, kinematic_state, joint_model_group);
            eef.position.x -= width_;
            const bool ta = checkIK(eef, kinematic_state, joint_model_group);
            eight_iks_ = makeEightIKs(ba, bb, bc, bd, ta, tb, tc, td);
        }

        geometry_msgs::Point getAnchor() const { return anchor_; }
        double getWidth() const { return width_; }
        unsigned char getEightIks() const { return eight_iks_; }

        bool operator<(const Cube &other) const
        {
            return (width_ < other.getWidth()) &
                   (anchor_.x < other.getAnchor().x) &
                   (anchor_.y < other.getAnchor().y) &
                   (anchor_.z < other.getAnchor().z);
        }

        bool isUseful() const
        {
            // 0xFF: Every corner has a solution.
            // 0x00: Every corner has no solution.
            return (eight_iks_) && (eight_iks_ != 0xFF);
        }

        // Split without if statements (for optimization)
        void split(
            std::vector<Cube> &openlist,
            int &top,
            const robot_state::RobotStatePtr &kinematic_state,
            const robot_model::JointModelGroup *joint_model_group)
        {
            // Backup this cube to prevent data conflict.
            geometry_msgs::Point this_anchor(anchor_);
            double this_width(width_);
            unsigned char this_eight_iks(eight_iks_);

            /**
             * 27 IK checkpoints (0: middle point)
             * { ba(---), (0--), bb(+--),  (-0-), (00-), (+0-),  bd(-+-), (0+-), bc(++-),  // 012 345 678
             *     (--0), (0-0),   (+-0),  (-00), (000), (+00),    (-+0), (0+0),   (++0),  // 9*1 234 567
             *   ta(--+), (0-+), tb(+-+),  (-0+), (00+), (+0+),  td(-++), (0++), tc(+++) } // 89* 123 456
             * ___________________________________________________
             * { ba, bb, bc, bd, ta, tb, tc, td } => Already known
             */
            std::bitset<27> checkpoints(0);
            // 8 Knowns
            checkpoints[0] = this_eight_iks & 0x80;  // ba, 0x80, BOTTOM_RIGHT_BACK
            checkpoints[2] = this_eight_iks & 0x40;  // bb, 0x40, BOTTOM_RIGHT_FRONT
            checkpoints[6] = this_eight_iks & 0x20;  // bc, 0x20, BOTTOM_LEFT_FRONT
            checkpoints[8] = this_eight_iks & 0x10;  // bd, 0x10, BOTTOM_LEFT_BACK
            checkpoints[18] = this_eight_iks & 0x08; // ta, 0x08, TOP_RIGHT_BACK
            checkpoints[20] = this_eight_iks & 0x04; // tb, 0x04, TOP_RIGHT_FRONT
            checkpoints[24] = this_eight_iks & 0x02; // tc, 0x02, TOP_LEFT_FRONT
            checkpoints[26] = this_eight_iks & 0x01; // td, 0x01, TOP_LEFT_BACK
            // 19 Unkowns
            const double half_width = this_width / 2.0;
            geometry_msgs::Pose eef;
            eef.position.z = this_anchor.z; // bottom
            eef.position.y = this_anchor.y;
            eef.position.x = this_anchor.x + half_width;
            checkpoints[1] = checkIK(eef, kinematic_state, joint_model_group);
            eef.position.y = this_anchor.y + half_width;
            eef.position.x = this_anchor.x;
            checkpoints[3] = checkIK(eef, kinematic_state, joint_model_group);
            eef.position.x += half_width;
            checkpoints[4] = checkIK(eef, kinematic_state, joint_model_group);
            eef.position.x += half_width;
            checkpoints[5] = checkIK(eef, kinematic_state, joint_model_group);
            eef.position.y += half_width;
            eef.position.x -= half_width;
            checkpoints[7] = checkIK(eef, kinematic_state, joint_model_group);
            eef.position.z += half_width; // middle
            eef.position.y = this_anchor.y;
            eef.position.x = this_anchor.x;
            checkpoints[9] = checkIK(eef, kinematic_state, joint_model_group);
            eef.position.x += half_width;
            checkpoints[10] = checkIK(eef, kinematic_state, joint_model_group);
            eef.position.x += half_width;
            checkpoints[11] = checkIK(eef, kinematic_state, joint_model_group);
            eef.position.y += half_width;
            eef.position.x = this_anchor.x;
            checkpoints[12] = checkIK(eef, kinematic_state, joint_model_group);
            eef.position.x += half_width;
            checkpoints[13] = checkIK(eef, kinematic_state, joint_model_group);
            eef.position.x += half_width;
            checkpoints[14] = checkIK(eef, kinematic_state, joint_model_group);
            eef.position.y += half_width;
            eef.position.x = this_anchor.x;
            checkpoints[15] = checkIK(eef, kinematic_state, joint_model_group);
            eef.position.x += half_width;
            checkpoints[16] = checkIK(eef, kinematic_state, joint_model_group);
            eef.position.x += half_width;
            checkpoints[17] = checkIK(eef, kinematic_state, joint_model_group);
            eef.position.z += half_width; // top
            eef.position.y = this_anchor.y;
            eef.position.x = this_anchor.x + half_width;
            checkpoints[19] = checkIK(eef, kinematic_state, joint_model_group);
            eef.position.y += half_width;
            eef.position.x = this_anchor.x;
            checkpoints[21] = checkIK(eef, kinematic_state, joint_model_group);
            eef.position.x += half_width;
            checkpoints[22] = checkIK(eef, kinematic_state, joint_model_group);
            eef.position.x += half_width;
            checkpoints[23] = checkIK(eef, kinematic_state, joint_model_group);
            eef.position.y += half_width;
            eef.position.x -= half_width;
            checkpoints[25] = checkIK(eef, kinematic_state, joint_model_group);

            // Push 8 cubes into the openlist
            top++; // ba-side
            geometry_msgs::Point new_anchor = this_anchor;
            openlist[top].init(new_anchor, half_width, makeEightIKs(checkpoints, 0, 1, 4, 3, 9, 10, 13, 12));
            top++; // bb-side
            new_anchor.x += half_width;
            openlist[top].init(new_anchor, half_width, makeEightIKs(checkpoints, 1, 2, 5, 4, 10, 11, 14, 13));
            top++; // bc-side
            new_anchor.y += half_width;
            openlist[top].init(new_anchor, half_width, makeEightIKs(checkpoints, 4, 5, 8, 7, 13, 14, 17, 16));
            top++; // bd-side
            new_anchor.x -= half_width;
            openlist[top].init(new_anchor, half_width, makeEightIKs(checkpoints, 3, 4, 7, 6, 12, 13, 16, 15));
            top++; // td-side
            new_anchor.z += half_width;
            openlist[top].init(new_anchor, half_width, makeEightIKs(checkpoints, 12, 13, 16, 15, 21, 22, 25, 24));
            top++; // tc-side
            new_anchor.x += half_width;
            openlist[top].init(new_anchor, half_width, makeEightIKs(checkpoints, 13, 14, 17, 16, 22, 23, 26, 25));
            top++; // tb-side
            new_anchor.y -= half_width;
            openlist[top].init(new_anchor, half_width, makeEightIKs(checkpoints, 10, 11, 14, 13, 19, 20, 23, 22));
            top++; // ta-side
            new_anchor.x -= half_width;
            openlist[top].init(new_anchor, half_width, makeEightIKs(checkpoints, 9, 10, 13, 12, 18, 19, 22, 21));
        }

    private:
        geometry_msgs::Point anchor_; // The position of ba corner
        double width_;
        /**
         * IK results for each corner. true == solution found.
         * Cube 8 division order:
         *     [+x: foward, +y: left, +z: up] => (+++)
         *     Bottom CCW : ba(---) -> bb(+--) -> bc(++-) -> bd(-+-)
         *     Top CCW    : ta(--+) -> tb(+-+) -> tc(+++) -> td(-++)
         * eight_iks_ == 0b {ba bb bc bd} {ta tb tc td}
         */
        unsigned char eight_iks_;
    };

    void printDebugInfo(const Cube* c)
    {
        ROS_INFO_STREAM("Anchor: \n"
                        << c->getAnchor()
                        << "\nWidth: " << c->getWidth()
                        << "\nEight IKs: " << std::bitset<8>(c->getEightIks())
                        << "\nIs useful: " << (c->isUseful() ? "true" : "false"));
    }
}

namespace DFS
{
    using AnchorPtr = std::shared_ptr<geometry_msgs::Point>;

    struct AnchorPtrCompare
    {
        bool operator() (const AnchorPtr& lhs, const AnchorPtr& rhs) const
        {
            int lx = int(lhs->x * 1000);
            int ly = int(lhs->y * 1000);
            int lz = int(lhs->z * 1000);
            int rx = int(rhs->x * 1000);
            int ry = int(rhs->y * 1000);
            int rz = int(rhs->z * 1000);
            return (lx < rx) ||
                   (lx == rx && ly < ry) ||
                   (lx == rx && ly == ry && lz < rz);
        }
    };

    std::array<AnchorPtr, 26> expand(const AnchorPtr& center, const double width)
    {
        std::array<AnchorPtr, 26> outer;
        for (auto& p : outer) { p = AnchorPtr(new geometry_msgs::Point(*center)); }
        for (int i = 0; i < 9; i++)
        {
            outer[i]->z -= width;       // Bottom
            outer[17 + i]->z += width;  // Top
        }
        for (int i = 0; i < 3; i++)
        {
            // Front (ba-bb-tb-ta)
            outer[i]->y -= width;
            outer[9 + i]->y -= width;
            outer[17 + i]->y -= width;  // indexing w/o center{
            // Back (bc-bd-td-tc)
            outer[6 + i]->y += width;
            outer[14 + i]->y += width;
            outer[23 + i]->y += width;  // indexing w/o center
        }
        for (int i = 0; i < 10; i += 3) // 0, 3, 6, 9
        {
            outer[i]->x -= width;
            outer[2 + i]->x += width;
            outer[14 + i]->x -= width;
            outer[16 + i]->x += width;
        }
        outer[12]->x -= width;
        outer[13]->x += width;
        return outer;
    }
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
    move_group.setWorkspace(-2.0, -2.0, -2.0, 2.0, 2.0, 2.0); // Search space

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

    // Visualization
    const std::string base_frame = move_group.getPlanningFrame();
    ROS_INFO_STREAM("Base frame: " << base_frame);
    moveit_visual_tools::MoveItVisualTools visual_tools(base_frame);
    visual_tools.deleteAllMarkers();
    visual_tools.loadRemoteControl();
    visual_tools.setAlpha(0.2); // 0 is invisible
    visual_tools.enableBatchPublishing();
    std::size_t marker_count = 0;

    /**********************************
     * MAIN ALGORITHM
     **********************************/
    // Cube width
    const double dfs_resolution = 0.16;  // DFS
    const double marching_resolution = 0.4;  // Octree

    ros::Time step1_start_time = ros::Time::now();

    // [ STEP 1 ] DFS
    // ^^^^^^^^^^^^^^
    std::vector<DFS::AnchorPtr> roi;
    {
        std::vector<DFS::AnchorPtr> dfs_openlist;
        std::set<DFS::AnchorPtr, DFS::AnchorPtrCompare> dfs_closedlist;

        DFS::AnchorPtr root(new geometry_msgs::Point(zero_pose.position));
        dfs_openlist.push_back(root);
        dfs_closedlist.insert(root);

        while (dfs_openlist.size())
        {
            DFS::AnchorPtr current = dfs_openlist.back();
            dfs_openlist.pop_back();

            drawCuboidFromAnchor(*current, dfs_resolution, visual_tools, rvt::RED);
            visual_tools.trigger();

            // Expand the current node
            for (const DFS::AnchorPtr& child : DFS::expand(current, dfs_resolution))
            {
                // If the child is NOT in the closed list
                if (dfs_closedlist.find(child) == dfs_closedlist.end())
                {
                    dfs_closedlist.insert(child);

                    // If IK has a solution
                    geometry_msgs::Pose eef;
                    eef.position = *child;
                    if (checkIK(eef, kinematic_state, joint_model_group))
                    {
                        dfs_openlist.push_back(child);
                    }
                }
                else
                {
                    ROS_WARN("The child is already in the closed list");
                }
                ROS_INFO_STREAM("dfs_openlist.size(): " << dfs_openlist.size());
                ROS_INFO_STREAM("dfs_closedlist size: " << dfs_closedlist.size());
            }
        }
        std::copy(dfs_closedlist.begin(), dfs_closedlist.end(), std::back_inserter(roi));
    }
    ROS_WARN_STREAM("Step 1 complete! Delete all markers");
    debugPause();

    visual_tools.deleteAllMarkers();
    visual_tools.trigger();
    ros::Duration(1.0).sleep();
    for (const auto& anchor : roi)
    {
        drawCuboidFromAnchor(*anchor, dfs_resolution, visual_tools, rvt::GREEN);
    }
    visual_tools.trigger();
    debugPause();

    visual_tools.deleteAllMarkers();
    visual_tools.trigger();
    ros::Duration(1.0).sleep();
    debugPause();

    ros::Time step2_start_time = ros::Time::now();

    // [ STEP 2 ] Octree
    // ^^^^^^^^^^^^^^^^^
    std::vector<Octree::Cube> octree_openlist(128, Octree::Cube());
    int top = -1;  // Last-in first-out (top can be negative)

    /***************
     * Pseudo Code
     * ^^^^^^^^^^^
     * openlist = { initial 8 cubes }  // Last-in first-out
     * while (openlist is not empty)
     * {
     *     cube = openlist.pop_back()
     *     if (all 8 cube.vertices have the same checkIK result)
     *     {
     *         Ignore it and continue
     *     }
     *     else if (cube.width > resolution)
     *     {
     *         Split cube into 8 cubes
     *         openlist.push_back(8 cubes)
     *     }
     *     else
     *     {
     *         Marching cubes
     *     }
     * }
     ***************/
    while ((top >= 0) || (roi.size() > 0))
    {
        ROS_INFO_STREAM("Current top: " << top << " / roi.size: " << roi.size());
        if (top < 0)
        {
            top++;
            octree_openlist[top].init(*roi.back(), dfs_resolution, kinematic_state, joint_model_group);
            roi.pop_back();
        }

        // Pop the last cube
        Octree::Cube *cube = &octree_openlist[top];
        top--;

        // Debug
        Octree::printDebugInfo(cube);


        if (cube->isUseful())
        {
            if (cube->getWidth() < marching_resolution)
            {
                // Pseudo marching cubes
                drawCuboidFromAnchor(cube->getAnchor(), cube->getWidth(), visual_tools, rvt::BLUE);
                marker_count++;
                if (marker_count % 64 == 0) { visual_tools.trigger(); }
            }
            else
            {
                // Split cube into 8 cubes
                // ROS_INFO_STREAM("Will_split Popped idx: " << top + 1 << ", Popped Cube width: " << cube->getWidth());
                cube->split(octree_openlist, top, kinematic_state, joint_model_group);
                // ROS_INFO_STREAM("After split top: " << top);
            }
        }
        debugPause();
    }
    visual_tools.trigger();

    ros::Time finish_time = ros::Time::now();
    ROS_WARN_STREAM("========== DONE! ==========");
    ROS_WARN_STREAM("Total execution time: " << (finish_time - step1_start_time).toSec() << " sec");
    ROS_WARN_STREAM("    > Step1 : " << (step2_start_time - step1_start_time).toSec() << " sec");
    ROS_WARN_STREAM("    > Step2 : " << (finish_time - step2_start_time).toSec() << " sec");

    ros::waitForShutdown();
    return 0;
}
