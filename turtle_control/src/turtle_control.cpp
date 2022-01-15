#include <ros/ros.h>
#include <geometry_msgs/Vector3.h>
#include <vector>

class VelocityProfile
{
public:
    //! @brief Create a new Velocity Profile
    //!
    //! @param[in] v_st The steady velocity. negative value means opposite direction
    //! @param[in] acc The acceleration at constant acceleration step. negative value means opposite direction.
    //! @param[in] t_total The total time of the motion.
    VelocityProfile(float v_st, float acc, float t_total) : v_st_(v_st), acc_(acc), t_total_(t_total)
    {
        ROS_ASSERT(v_st != 0);
        ROS_ASSERT(acc != 0);
        ROS_ASSERT(t_total > 0);
    }

    ~VelocityProfile() = default;
    float GetVelocity(float t)
    {
        const float &v_acc = acc_ * t;
        const float &v_dec = -acc_ * (t - t_total_);

        ROS_ASSERT(v_st_ * acc_ > 0);
        ROS_ASSERT(t >= 0);
        if (t < 0.0 || t > t_total_)
        {
            return 0.0;
        }
        if (v_st_ > 0 && acc_ > 0)
        {
            return std::min(std::min(v_acc, v_dec), v_st_);
        }
        else
        {
            return std::max(std::max(v_acc, v_dec), v_st_);
        }
    }
    float GetTotalTime()
    {
        return t_total_;
    }

private:
    float v_st_;
    float acc_;
    float t_total_;
};

class CompositeProfile
{
public:
    float GetVelocity(float t)
    {
        auto it = profiles_.begin();
        while (it->GetTotalTime() < t && it + 1 != profiles_.end())
        {
            t -= it->GetTotalTime();
            it++;
        }
        return it->GetVelocity(t);
    }
    void AddProfile(VelocityProfile &profile)
    {
        profiles_.push_back(profile);
    }

private:
    std::vector<VelocityProfile> profiles_;
};

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "turtle_control");
    ros::NodeHandle nh("~");
    const float &v_max = 0.1;
    const float &acc = 0.1;

    CompositeProfile cp;

    for (int j = 0; j < 4; j++)
    {
        VelocityProfile vp0(v_max, acc, 5.0);
        VelocityProfile vp1(-v_max, -acc, 4.0);
        cp.AddProfile(vp0);
        cp.AddProfile(vp1);
    }

    ros::Publisher pub = nh.advertise<geometry_msgs::Vector3>("/wheel_control", 3);
    ros::Rate rate(20);
    for (float i = 0.; i < 1000.0; i += 0.05)
    {
        const float &vel = cp.GetVelocity(i);
        ROS_INFO("set linear velocity: %f", vel);

        geometry_msgs::Vector3 msg;
        msg.x = 1.0; // velocity_mode
        msg.y = vel; //left wheel velocity
        msg.z = vel; //right wheel velocity
        pub.publish(msg);

        ros::spinOnce();
        rate.sleep();
    }
    return 0;
}