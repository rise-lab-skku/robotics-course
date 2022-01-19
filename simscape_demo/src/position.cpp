#include <ros/ros.h>
#include <std_msgs/Float32.h>

// Global variables
float simscape_input_pos_;
float simscape_output_pos_;
float simscape_output_torque_;

void simscape_input_callback(const std_msgs::Float32::ConstPtr& msg)
{
    simscape_input_pos_ = msg->data;
}

void simscape_output_pos_callback(const std_msgs::Float32::ConstPtr& msg)
{
    simscape_output_pos_ = msg->data;
}

void simscape_output_torque_callback(const std_msgs::Float32::ConstPtr& msg)
{
    simscape_output_torque_ = msg->data;
}

int main(int argc, char **argv){
    ros::init(argc, argv, "position_node");
    ros::NodeHandle nh;
    ros::AsyncSpinner spinner(3); // Use 3 threads
    spinner.start();

    // Subscribers
    ros::Subscriber sub_input_pos = nh.subscribe("/input_pos", 128, simscape_input_callback);
    ros::Subscriber sub_output_pos = nh.subscribe("/output_pos", 128, simscape_output_pos_callback);
    ros::Subscriber sub_output_torque = nh.subscribe("/output_torque", 128, simscape_output_torque_callback);

    ros::Rate loop_rate(2);  // Hz
    while (ros::ok())
    {
        ROS_INFO_STREAM("Received input position : " << simscape_input_pos_ << " rad");
        ROS_INFO_STREAM("Received output position: " << simscape_output_pos_ << " rad");
        ROS_INFO_STREAM("Received output torque  : " << simscape_output_torque_ << " Nm\n");
        ros::spinOnce();
        loop_rate.sleep();
    }
    ros::waitForShutdown();
    return 0;
}
