/*********************************
 * Made by @ohilho
 * Special thanks to @ssw0536, @shinjinjae
 *********************************/
#ifndef KINEMATICS_DEMO_SO3_HPP
#define KINEMATICS_DEMO_SO3_HPP

#include <eigen3/Eigen/Dense>

namespace SO3
{
inline void hat(const Eigen::Vector3d &so3, Eigen::Matrix3d &so3_hat)
{
    // ROS_INFO_STREAM("Why you die.daskljfl;k");
    so3_hat = Eigen::Matrix3d::Zero();
    // ROS_INFO_STREAM("Why you die.daskljfl;k 0123012301230123");
    so3_hat(0, 1) = -so3(2);
    so3_hat(0, 2) = so3(1);
    so3_hat(1, 2) = -so3(0);
    so3_hat(1, 0) = so3(2);
    so3_hat(2, 1) = -so3(1);
    so3_hat(2, 2) = so3(0);
    // ROS_INFO_STREAM("Are you kididngklm  lmek asdfjlksa.");
}

inline void vee(const Eigen::Matrix3d &so3_hat, Eigen::Vector3d &so3)
{
    // normalize matrix.
    // this is property of skew symmetric matrix.
    Eigen::Matrix3d n = (so3_hat - so3_hat.transpose()) / 2;
    so3(0) = -so3_hat(1, 2);
    so3(1) = so3_hat(0, 2);
    so3(2) = -so3_hat(0, 1);
}

inline void exp(const Eigen::Vector3d &so3, Eigen::Matrix3d &SO3)
{
    // calculate theta
    const float &th = so3.norm();
    // check divide by zero
    if (FP_ZERO == std::fpclassify(th))
    {
        SO3 = Eigen::Matrix3d::Identity();
        return;
    }
    // make so3_hat
    Eigen::Matrix3d so3_hat;
    hat(so3, so3_hat);
    // do exp operation
    SO3 = Eigen::Matrix3d::Identity()
        + sin(th) * (so3_hat / th)
        + (1.0 - cos(th)) * (so3_hat * so3_hat / (th * th));
}

inline void log(const Eigen::Matrix3d &SO3, Eigen::Vector3d &so3)
{
    const float &trace = SO3.trace();
    const float &cos_th = (trace - 1.0) / 2.0;
    // cos(th) == 1.0 --> th == 0 --> sin(th) == 0.0
    if (FP_ZERO == std::fpclassify(cos_th - 1.0))
    {
        so3 = Eigen::Vector3d::Zero();
        return;
    }
    // cos(th) == - 1.0 --> th == PI --> sin(th) == 0.0
    // modern robotics. page 87. eq 3.58
    else if (FP_ZERO == std::fpclassify(cos_th + 1.0))
    {
        const float &a = 1.0 / std::sqrt(2.0 * (1 + SO3(2, 2)));
        so3(0) = a * SO3(0, 2);
        so3(1) = a * SO3(1, 2);
        so3(2) = a * (SO3(2, 2) + 1.0);
        return;
    }
    const float &th = std::acos(cos_th);
    vee((SO3 - SO3.transpose()) * th / (2.0 * std::sin(th)), so3);
}
}

#endif /* KINEMATICS_DEMO_SO3_HPP */