#include <eigen3/Eigen/Dense>

static Eigen::Matrix4d DH(double alpha, double a, double d, double theta)
{
    double cosT = cos(theta), sinT = sin(theta);
    double cosA = cos(alpha), sinA = sin(alpha);
    Eigen::Matrix4d T;
    T <<      cosT,     -sinT,     0,       a,
         sinT*cosA, cosT*cosA, -sinA, -sinA*d,
         sinT*sinA, cosT*sinA,  cosA,  cosA*d,
                 0,         0,     0,       1;
    return T;
};