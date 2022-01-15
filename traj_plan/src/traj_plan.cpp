#include <vector>
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <traj_plan/PoseStampedArray.h>
#include <traj_plan/JointInterpolation.h>

struct CubicSplineSegment
{
    float a;
    float b;
    float c;
    float d;
    float x;
};

class Interpolater
{
public:
    Interpolater() {}
    virtual ~Interpolater() {}
    virtual float Interpolate(float x) = 0;
    virtual void Setup(std::vector<float> &x, std::vector<float> &y) = 0;
};

class LinearInterpolater : public Interpolater
{
public:
    LinearInterpolater() : Interpolater() {}
    ~LinearInterpolater() {}
    virtual float Interpolate(float x)
    {
        auto it_x = x_.begin() + 1;
        auto it_prev_x = x_.begin();
        auto it_y = y_.begin() + 1;
        auto it_prev_y = y_.begin();

        while (it_x != x_.end() && *it_x < x)
        {
            it_x++;
            it_prev_x++;
            it_y++;
            it_prev_y++;
        }
        return *it_prev_y + (*it_y - *it_prev_y) / (*it_x - *it_prev_x) * (x - *it_prev_x);
    };
    virtual void Setup(std::vector<float> &x, std::vector<float> &y)
    {
        x_ = x;
        y_ = y;
    };

private:
    std::vector<float> x_;
    std::vector<float> y_;
};

class SplineInterpolater : public Interpolater
{
public:
    SplineInterpolater() : Interpolater() {}
    ~SplineInterpolater() {}

    virtual float Interpolate(float x)
    {
        auto it_sp = spline.begin();
        auto end_sp = spline.end();
        while ((it_sp + 1) != spline.end() && (it_sp + 1)->x < x)
        {
            it_sp++;
        }
        const float &tt = x - it_sp->x;
        const float &tt2 = tt * tt;
        const float &tt3 = tt2 * tt;
        return it_sp->a + it_sp->b * tt + it_sp->c * tt2 + it_sp->d * tt3;
    }

    virtual void Setup(std::vector<float> &x, std::vector<float> &y)
    {
        // realization of the algorithm from https://en.wikipedia.org/wiki/Spline_(mathematics)
        size_t n = x.size() - 1;
        spline.resize(n);

        std::vector<float> a(n + 1);
        std::vector<float> b(n);
        std::vector<float> c(n + 1);
        std::vector<float> d(n);
        std::vector<float> h(n);
        std::vector<float> alpha(n);
        std::vector<float> l(n + 1);
        std::vector<float> mu(n + 1);
        std::vector<float> z(n + 1);

        a.insert(a.begin(), y.begin(), y.end());

        for (size_t i = 0; i < n; ++i)
        {
            h[i] = x[i + 1] - x[i];
        }
        for (size_t i = 1; i < n; ++i)
        {
            alpha[i] = (3.0f / h[i]) * (a[i + 1] - a[i]) - (3.0f / h[i - 1]) * (a[i] - a[i - 1]);
        }
        l[0] = 1;
        mu[0] = 0;
        z[0] = 0;
        for (size_t i = 1; i < n; ++i)
        {
            l[i] = 2.0f * (x[i + 1] - x[i - 1]) - h[i - 1] * mu[i - 1];
            mu[i] = h[i] / l[i];
            z[i] = (alpha[i] - h[i - 1] * z[i - 1]) / l[i];
        }
        l[n] = 1;
        z[n] = 0;
        c[n] = 0;
        for (int i = n - 1; i >= 0; --i)
        {
            c[i] = z[i] - mu[i] * c[i + 1];
            b[i] = (a[i + 1] - a[i]) / h[i] - h[i] * (c[i + 1] + 2 * c[i]) / 3.0f;
            d[i] = (c[i + 1] - c[i]) / (3.0f * h[i]);
        }
        for (size_t i = 0; i < n; ++i)
        {
            spline[i].a = a[i];
            spline[i].b = b[i];
            spline[i].c = c[i];
            spline[i].d = d[i];
            spline[i].x = x[i];
        }
    }

private:
    std::vector<CubicSplineSegment> spline;
};

template <typename InterpolatorType>
class TrajectoryPlanner
{
public:
    TrajectoryPlanner() {}
    virtual ~TrajectoryPlanner() {}

    void Init(ros::NodeHandle &nh, float time_period)
    {
        time_period_ = time_period;

        // Topic version
        pub_traj_ = nh.advertise<trajectory_msgs::JointTrajectory>("joint_trajectory", 3, false);
        pub_pose_ = nh.advertise<traj_plan::PoseStampedArray>("pose_trajectory", 3, false);
        sub_traj_ = nh.subscribe("joint_waypoints", 3, &TrajectoryPlanner::JointTrajectoryCallback, this);
        sub_pose_ = nh.subscribe("pose_waypoints", 3, &TrajectoryPlanner::PoseStampedCallback, this);

        // Service version
        joint_traj_service_ = nh.advertiseService("joint_trajectory_service", &TrajectoryPlanner::JointTrajectoryService, this);
    }
    void PoseStampedCallback(const traj_plan::PoseStampedArray::ConstPtr &msg)
    {
        std::vector<std::vector<float>> positions(7);
        std::vector<InterpolatorType> interpolators(7);
        std::vector<float> curve_param;
        // make joint positions vectors for each joint.
        double start = msg->data.front().header.stamp.toSec();
        for (auto &point : msg->data)
        {
            positions[0].push_back(point.pose.position.x);
            positions[1].push_back(point.pose.position.y);
            positions[2].push_back(point.pose.position.z);
            positions[3].push_back(point.pose.orientation.w);
            positions[4].push_back(point.pose.orientation.x);
            positions[5].push_back(point.pose.orientation.y);
            positions[6].push_back(point.pose.orientation.z);
            curve_param.push_back(point.header.stamp.toSec() - start);
        }

        // Setup interpolater
        for (int i = 0; i < positions.size(); ++i)
        {
            interpolators[i].Setup(curve_param, positions[i]);
        }

        // set a new message
        traj_plan::PoseStampedArray new_msg;

        float prev_t = -1;
        auto it_points = msg->data.begin();
        for (float t = curve_param.front() + time_period_; t < curve_param.back(); t += time_period_)
        {

            if ((it_points->header.stamp.toSec() - start) > prev_t && (it_points->header.stamp.toSec() - start) < t)
            {
                new_msg.data.push_back(*it_points);
                it_points++;
            }
            std::cout << "t: " << t << std::endl;
            new_msg.data.push_back(geometry_msgs::PoseStamped());
            new_msg.data.back().pose.position.x = interpolators[0].Interpolate(t);
            new_msg.data.back().pose.position.y = interpolators[1].Interpolate(t);
            new_msg.data.back().pose.position.z = interpolators[2].Interpolate(t);
            const float &w = interpolators[3].Interpolate(t);
            const float &x = interpolators[4].Interpolate(t);
            const float &y = interpolators[5].Interpolate(t);
            const float &z = interpolators[6].Interpolate(t);
            const float &abs = sqrt(w * w + x * x + y * y + z * z);
            new_msg.data.back().pose.orientation.w = w / abs;
            new_msg.data.back().pose.orientation.x = x / abs;
            new_msg.data.back().pose.orientation.y = y / abs;
            new_msg.data.back().pose.orientation.z = z / abs;
            new_msg.data.back().header.stamp = ros::Time(t + start);
            prev_t = t;
        }

        new_msg.data.push_back(msg->data.back());
        pub_pose_.publish(new_msg);
    }
    void JointTrajectoryCallback(const trajectory_msgs::JointTrajectory::ConstPtr &msg)
    {
        trajectory_msgs::JointTrajectory new_msg;
        calcJointTrajectory(new_msg, msg);
        pub_traj_.publish(new_msg);
    }

    bool JointTrajectoryService(traj_plan::JointInterpolationRequest &req, traj_plan::JointInterpolationResponse &res)
    {
        trajectory_msgs::JointTrajectory new_msg;
        trajectory_msgs::JointTrajectory::ConstPtr msg(new trajectory_msgs::JointTrajectory(req.waypoints));
        calcJointTrajectory(new_msg, msg);
        res.result = new_msg;
        return true;
    }

    void calcJointTrajectory(trajectory_msgs::JointTrajectory &res, const trajectory_msgs::JointTrajectory::ConstPtr &req)
    {
        std::vector<std::vector<float>> positions(req->joint_names.size());
        std::vector<InterpolatorType> interpolators(req->joint_names.size());
        std::vector<float> curve_param;
        // make joint positions vectors for each joint.
        int i = 0;
        for (auto &point : req->points)
        {
            auto it = positions.begin();
            for (auto &p : point.positions)
            {
                it->push_back(p);
                it++;
            }
            curve_param.push_back(i++);
        }

        // Setup interpolater
        for (int i = 0; i < positions.size(); ++i)
        {
            interpolators[i].Setup(curve_param, positions[i]);
        }

        // set a new message
        res.header = req->header;
        res.joint_names = req->joint_names;

        float prev_t = -1;
        auto it_points = req->points.begin();
        for (float t = curve_param.front() + time_period_; t < curve_param.back(); t += time_period_)
        {

            if (it_points->time_from_start.toSec() > prev_t && it_points->time_from_start.toSec() < t)
            {
                res.points.push_back(*it_points);
                it_points++;
            }

            res.points.push_back(trajectory_msgs::JointTrajectoryPoint());

            for (auto &interpolator : interpolators)
            {
                res.points.back().positions.push_back(interpolator.Interpolate(t));
                res.points.back().time_from_start = ros::Duration(t) + req->points.front().time_from_start;
            }

            prev_t = t;
        }
        res.points.push_back(req->points.back());
    }

protected:
    float time_period_;

    // Topic version
    ros::Publisher pub_traj_;
    ros::Publisher pub_pose_;
    ros::Subscriber sub_traj_;
    ros::Subscriber sub_pose_;

    // Service version
    ros::ServiceServer joint_traj_service_;
};

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "traj_plan");

    float period = 0.1;
    ros::NodeHandle nh_linear("~/linear");
    TrajectoryPlanner<LinearInterpolater> linear_traj_planner;
    linear_traj_planner.Init(nh_linear, period);

    ros::NodeHandle nh_spline("~/spline");
    TrajectoryPlanner<SplineInterpolater> spline_traj_planner;
    spline_traj_planner.Init(nh_spline, period);

    ros::spin();
    return 0;
}