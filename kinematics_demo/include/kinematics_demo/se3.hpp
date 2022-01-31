/*********************************
 * Made by @ohilho
 * Special thanks to @ssw0536, @shinjinjae
 *********************************/

#include <eigen3/Eigen/Dense>

namespace SE3
{
void hat_se3(const Eigen::VectorXf &se3, Eigen::MatrixXf &se3_hat)
{
    // Eigen::VectorXf t= se3.block(0,0,3,1);
    // Eigen::VectorXf w= se3.block(3,0,3,1);

    // make so3_hat
    Eigen::MatrixXf so3_hat;
    hat_so3(se3.block(3, 0, 3, 1), so3_hat);

    // set values
    se3_hat = Eigen::MatrixXf::Zero(4, 4);
    se3_hat.block(0, 0, 3, 3) = so3_hat;
    se3_hat.block(0, 3, 3, 1) = se3.block(0, 0, 3, 1);
}

void vee_se3(const Eigen::MatrixXf &se3_hat, Eigen::VectorXf &se3)
{
    // set translation
    se3.block(0, 0, 3, 1) = se3_hat.block(0, 3, 3, 1);

    // make so3
    Eigen::VectorXf so3;
    vee_so3(se3_hat.block(0, 0, 3, 3), so3);

    // set orientation
    se3.block(0, 3, 3, 1) = so3;
}

void exp_se3(const Eigen::VectorXf &se3, Eigen::MatrixXf &SE3)
{
    // make so3_hat
    Eigen::MatrixXf so3_hat;
    hat_so3(se3.block(3, 0, 3, 1), so3_hat);

    float th = se3.norm();

    // th == 0 , which means no motion. --> Identity.
    if (FP_ZERO == std::fpclassify(th))
    {
        SE3 = Eigen::MatrixXf::Identity(4, 4);
        return;
    }

    // make V
    const float &th_sq = th * th;
    const float &th_cb = th_sq * th;
    Eigen::MatrixXf V = Eigen::MatrixXf::Identity(3, 3) +
                        so3_hat * (1.0 - std::cos(th)) / (th_sq) +
                        so3_hat * so3_hat * (th - std::sin(th)) / (th_cb);

    // make SO3
    Eigen::MatrixXf SO3;
    exp_so3(se3.block(3, 0, 3, 1), SO3);
    SE3.block(0, 0, 3, 3) = SO3;
    SE3.block(0, 3, 3, 1) = V * se3.block(0, 0, 3, 1);
}

void log_se3(const Eigen::MatrixXf &SE3, Eigen::VectorXf &se3)
{
    // make so3
    Eigen::VectorXf so3;
    log_so3(SE3.block(0, 0, 3, 3), so3);
    const float &th = so3.norm();

    // th == 0 , which means no rotation
    // modern robotics. page 106, 3.3.3.2, algorithm case (a)
    if (FP_ZERO == std::fpclassify(th))
    {
        se3.block(0, 0, 3, 1) = SE3.block(0, 3, 3, 1).normalized();
        se3.block(3, 0, 3, 1) = Eigen::VectorXf::Zero(3);
        return;
    }

    // make so3_hat
    Eigen::MatrixXf so3_hat;
    hat_so3(SE3.block(0, 0, 3, 3), so3_hat);

    //make V_inv
    Eigen::MatrixXf V_inv = Eigen::MatrixXf::Identity(3, 3) -
                            so3_hat * 0.5 +
                            so3_hat * so3_hat * (1.0 - (th * std::cos(0.5 * th) / (2.0 * std::sin(0.5 * th)))) / (th * th);

    //set se3
    se3.block(0, 0, 3, 1) = V_inv * SE3.block(0, 3, 3, 1);
    se3.block(3, 0, 3, 1) = so3;
}
}
