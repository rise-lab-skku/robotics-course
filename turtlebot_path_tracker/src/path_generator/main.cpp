#include "path_generator/turtlebot_path_generator.h"

int main(int argc, char** argv)
{
ros::init(argc, argv, "path_generator_node");
  path_generator::ExamplePath path;

  ros::Rate rate(20);

  while (ros::ok())
  {
    path.PathGenerator();
    rate.sleep();
  }

  return 0;
}