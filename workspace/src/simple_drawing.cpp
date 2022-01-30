#include <math.h>
#include <ros/ros.h>
#include <rviz_visual_tools/rviz_visual_tools.h>
#include <geometry_msgs/Pose.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "simple_drawing");
    ros::NodeHandle nh;
    ros::AsyncSpinner spinner(1);
    spinner.start();

    // Create a namespace for this node so multiple RVizVisualTools can be created
    namespace rvt = rviz_visual_tools;
    // namespace mvt = moveit_visual_tools;

    // Create an RVizVisualTools object.
    // RVizVisualTools provides many functions for drawing basic shapes.
    // For example, a sphere can be drawn with rviz_visual_tools::drawSphere()
    // MoveItVisualTools is a subclass of RVizVisualTools.
    // It also provides functions for drawing planning scene objects.
    rvt::RvizVisualToolsPtr visual_tools = rvt::RvizVisualToolsPtr(
        new rvt::RvizVisualTools("map", "/rviz_visual_markers"));
    visual_tools->deleteAllMarkers();

    // RVizVisualTools allows many options to be changed.
    visual_tools->setLifetime(1.0);
    visual_tools->setAlpha(0.5);

    /******************
     * Reference of rviz_visual_tools:
     *    http://docs.ros.org/en/melodic/api/rviz_visual_tools/html/classrviz__visual__tools_1_1RvizVisualTools.html
     * Reference of moveit_visual_tools:
     *    https://docs.ros.org/en/melodic/api/moveit_visual_tools/html/classmoveit__visual__tools_1_1MoveItVisualTools.html
     ******************/

    double depth = 1.0;
    double width = 0.5;
    double height = 0.1;

    ros::Time start_time = ros::Time::now();
    double radius = 2.0;

    ros::Rate rate(5);
    while (ros::ok())
    {
        double t = (ros::Time::now() - start_time).toSec();

        // RVizVisualTools also has functions for drawing a cube.
        geometry_msgs::Pose center;
        center.position.x = radius * cos(t);
        center.position.y = radius * sin(t);
        center.orientation.w = 1.0;
        visual_tools->publishCuboid(center, depth, width, height, rvt::BLUE);
        visual_tools->trigger();  // Publish/update RViz.

        ROS_INFO_STREAM("Published cuboid at (" << center.position.x << ", " << center.position.y << ")");

        ros::spinOnce();
        rate.sleep();
    }
    ros::waitForShutdown();
    return 0;
}