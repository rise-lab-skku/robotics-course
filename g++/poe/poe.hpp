#include <eigen3/Eigen/Dense>

using namespace Eigen;

namespace Exponential
{
    // 3x3 skew-symmetric matrix of cross product
    Matrix3d cross_product_matrix(Vector3d w)
    {
        Matrix3d w_hat;
        w_hat <<    0, -w(2),  w(1),
                 w(2),     0, -w(0),
                -w(1),  w(0),     0;
        return w_hat;
    }

    // Rotation matrix via Rodrigues' formula
    Matrix3d rotation_matrix(Vector3d Sw, double theta)
    {
        Matrix3d wx = cross_product_matrix(Sw);
        Matrix3d rodrigues =
            Matrix3d::Identity() + sin(theta)*wx + (1-cos(theta))*wx*wx;
        return rodrigues;
    }

    // 3x1 translation vector
    Vector3d translation_vector(Vector3d Sw, Vector3d Sv, double theta)
    {
        if (Sw.isZero())
            return Sv * theta;
        Matrix3d wx = cross_product_matrix(Sw);
        Matrix3d G =
            Matrix3d::Identity()*theta + (1-cos(theta))*wx + (theta-sin(theta))*wx*wx;
        return G * Sv;
    }

    // Tranformation matrix from matrix exponential
    Matrix4d matrix_form(Vector3d Sw, Vector3d Sv, double theta)
    {
        Matrix4d T = Matrix4d::Identity();
        T.block<3, 3>(0, 0) = rotation_matrix(Sw, theta);
        T.block<3, 1>(0, 3) = translation_vector(Sw, Sv, theta);
        return T;
    }
}
