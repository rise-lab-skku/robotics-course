#include "ros/ros.h"
#include "std_msgs/Bool.h"

int main(int argc, char **argv)
{
  
  ros::init(argc, argv, "friction_node");

  ros::NodeHandle nh;

  ros::Publisher impact_pub = nh.advertise<std_msgs::Bool>("/impact", 100);
  ros::Publisher friction_compen_pub = nh.advertise<std_msgs::Bool>("/friction_compensation", 100);
  ros::Publisher gravity_compen_pub = nh.advertise<std_msgs::Bool>("/gravity_compensation", 100);

  ros::Rate loop_rate(10);

  // init publish data
  bool is_impact, is_friction_compen, is_gravity_compen;
  is_impact = is_friction_compen = is_gravity_compen = false;

  // get parameters 
  ros::param::get("/impact_pub", is_impact);
  ros::param::get("/friction_compen_pub", is_friction_compen);
  ros::param::get("/gravity_compen_pub", is_gravity_compen);

  // insert into msg
  ROS_INFO("Publishing messages..");

  int count = 0;
  while (ros::ok())
  {
    std_msgs::Bool impact_msg;
    std_msgs::Bool friction_compen_msg;
    std_msgs::Bool gravity_compen_msg;

    impact_msg.data = is_impact;
    impact_pub.publish(impact_msg);

    friction_compen_msg.data = is_friction_compen;
    friction_compen_pub.publish(friction_compen_msg);

    gravity_compen_msg.data = is_gravity_compen;
    gravity_compen_pub.publish(gravity_compen_msg);

    ros::spinOnce();

    loop_rate.sleep();

    ++count;
  }


  return 0;
}