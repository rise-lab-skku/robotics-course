#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Bool.h>


class PidController
{
public:
    PidController(ros::NodeHandle &nh)
    {
        // Publishers
        pub_pid_ = nh.advertise<std_msgs::Float32>("/pid_result", 1000);
        pub_target_ = nh.advertise<std_msgs::Float32>("/target_pos", 1);
        pub_gc_ = nh.advertise<std_msgs::Bool>("/gravity_compensation", 1);
        // Subscriber
        sub_pos_ = nh.subscribe("/output_pos", 1000, &PidController::posCallback, this);
    }
    ~PidController() {}

    void run(float Kp, float Ki, float Kd, float target_pos, bool gravity_compensation)
    {
        // PID Controller
        float accumulated_error = 0;  // Integral term
        float prev_error = 0;         // Derivative term

        ros::Time prev_time = ros::Time::now();
        ros::Rate loop_rate(100);  // [Hz]

        while (ros::ok())
        {
            // Time
            ros::Time cur_time = ros::Time::now();
            float dt = (cur_time - prev_time).toSec();
            prev_time = cur_time;

            // Propotional term
            float error = target_pos - last_pos_;
            float Kp_term = Kp * error;
            // Integral term (with normalization about time)
            accumulated_error += error * dt;
            float Ki_term = Ki * accumulated_error;
            // Derivative term
            float deriv_error = (error - prev_error) / dt;
            float Kd_term = Kd * deriv_error;
            prev_error = error;

            // PID Controller
            std_msgs::Float32 pid_msg;
            pid_msg.data = Kp_term + Ki_term + Kd_term;
            pub_pid_.publish(pid_msg);

            // Target position
            std_msgs::Float32 target_msg;
            target_msg.data = target_pos;
            pub_target_.publish(target_msg);

            // Gravity Compensation
            std_msgs::Bool gc_msg;
            gc_msg.data = gravity_compensation;
            pub_gc_.publish(gc_msg);

            ros::spinOnce();
            loop_rate.sleep();
        }
    }

private:
    void posCallback(const std_msgs::Float32::ConstPtr &msg)
    {
        last_pos_ = msg->data;
        ROS_INFO_STREAM("Received position: " << last_pos_ << " rad");
    }

    ros::Publisher pub_pid_;
    ros::Publisher pub_target_;
    ros::Publisher pub_gc_;
    ros::Subscriber sub_pos_;
    float last_pos_;
};


int main(int argc, char **argv)
{
    ros::init(argc, argv, "gravity_node");
    ros::NodeHandle nh("/gravity_node");
    ros::AsyncSpinner spinner(4); // Use 4 threads
    spinner.start();

    // Essential Parameters
    float Kp;
    float target_pos;
    if (!nh.getParam("Kp", Kp) || !nh.getParam("target_pos", target_pos))
    {
        ROS_ERROR("Failed to get essential parameters");
        return 1;
    }
    // Optional Parameters (with default values)
    float Ki, Kd;
    bool gravity_compensation;
    nh.param<float>("Ki", Ki, 0);
    nh.param<float>("Kd", Kd, 0);
    nh.param<bool>("gravity_compensation", gravity_compensation, false);

    // PID Controller
    PidController controller(nh);
    controller.run(Kp, Ki, Kd, target_pos, gravity_compensation);

    ros::waitForShutdown();
    return 0;
}