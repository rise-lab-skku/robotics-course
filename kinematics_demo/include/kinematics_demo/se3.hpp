/*********************************
 * Made by @ohilho
 * Special thanks to @ssw0536, @shinjinjae
 *********************************/
#ifndef KINEMATICS_DEMO_SE3_HPP
#define KINEMATICS_DEMO_SE3_HPP

#include <eigen3/Eigen/Dense>
#include "kinematics_demo/so3.hpp"

// http://ingmec.ual.es/~jlblanco/papers/jlblanco2010geometry3D_techrep.pdf
namespace SE3
{
void hat(const Eigen::VectorXd &se3, Eigen::Matrix4d &se3_hat)
{
    // Eigen::VectorXf t= se3.block(0,0,3,1);
    // Eigen::VectorXf w= se3.block(3,0,3,1);

    // make so3_hat
    Eigen::Matrix3d so3_hat;
    SO3::hat(se3.block(3, 0, 3, 1), so3_hat);

    // set values
    se3_hat = Eigen::Matrix4d::Zero();
    se3_hat.block(0, 0, 3, 3) = so3_hat;
    se3_hat.block(0, 3, 3, 1) = se3.block(0, 0, 3, 1);
}

void vee(const Eigen::Matrix4d &se3_hat, Eigen::VectorXd &se3)
{
    // set translation
    se3.block(0, 0, 3, 1) = se3_hat.block(0, 3, 3, 1);

    // make so3
    Eigen::Vector3d so3;
    SO3::vee(se3_hat.block(0, 0, 3, 3), so3);

    // set orientation
    se3.block(0, 3, 3, 1) = so3;
}

void exp(const Eigen::VectorXd &se3, Eigen::Matrix4d &SE3)
{
    // make so3_hat
    Eigen::Matrix3d so3_hat;
    SO3::hat(se3.block(3, 0, 3, 1), so3_hat);

    float th = se3.norm();

    // th == 0 , which means no motion. --> Identity.
    if (FP_ZERO == std::fpclassify(th))
    {
        SE3 = Eigen::Matrix4d::Identity();
        return;
    }

    // make V
    const float &th_sq = th * th;
    const float &th_cb = th_sq * th;
    Eigen::Matrix3d V = Eigen::Matrix3d::Identity() +
                        so3_hat * (1.0 - std::cos(th)) / (th_sq) +
                        so3_hat * so3_hat * (th - std::sin(th)) / (th_cb);

    // make SO3
    Eigen::Matrix3d SO3;
    SO3::exp(se3.block(3, 0, 3, 1), SO3);
    SE3.block(0, 0, 3, 3) = SO3;
    SE3.block(0, 3, 3, 1) = V * se3.block(0, 0, 3, 1);
}

void log(const Eigen::Matrix4d &SE3, Eigen::VectorXd &se3)
{
    // make so3
    Eigen::Vector3d so3;
    SO3::log(SE3.block(0, 0, 3, 3), so3);
    const float &th = so3.norm();

    // ROS_INFO_STREAM("th: " << th);

    // th == 0 , which means no rotation
    // modern robotics. page 106, 3.3.3.2, algorithm case (a)
    if (FP_ZERO == std::fpclassify(th))
    {
        se3.block(0, 0, 3, 1) = SE3.block(0, 3, 3, 1).normalized();
        se3.block(3, 0, 3, 1) = Eigen::Vector3d::Zero();
        ROS_INFO_STREAM("se3: " << se3.transpose());
        return;
    }
    // ROS_INFO_STREAM("bvefore hasdktjakls");

    // make so3_hat
    Eigen::Matrix3d so3_hat;
    SO3::hat(so3, so3_hat);
    // ROS_INFO_STREAM("so3_hat: \n" << so3_hat);

    //make V_inv
    Eigen::MatrixXd V_inv = Eigen::Matrix3d::Identity() -
                            so3_hat * 0.5 +
                            so3_hat * so3_hat * (1.0 - (th * std::cos(0.5 * th) / (2.0 * std::sin(0.5 * th)))) / (th * th);
    // ROS_INFO_STREAM("V_inv: \n" << V_inv);

    //set se3
    se3.block(0, 0, 3, 1) = V_inv * SE3.block(0, 3, 3, 1);
    se3.block(3, 0, 3, 1) = so3;
    // ROS_INFO_STREAM("__ se3: " << se3.transpose());
}

void adjoint(const Eigen::Matrix4d &SE3, Eigen::MatrixXd &adj)
{
    adj.resize(6, 6);
    Eigen::Matrix3d p_hat;
    SO3::hat(SE3.block(0, 3, 3, 1), p_hat);
    adj.block(0, 0, 3, 3) = SE3.block(0, 0, 3, 3);
    adj.block(3, 0, 3, 3) = p_hat * SE3.block(0, 0, 3, 3);
    adj.block(0, 3, 3, 3) = Eigen::MatrixXd::Zero(3, 3);
    adj.block(3, 3, 3, 3) = SE3.block(0, 0, 3, 3);
}
}

#endif //KINEMATICS_DEMO_SE3_HPP