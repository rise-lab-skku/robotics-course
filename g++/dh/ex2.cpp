#include <iostream>
#include "dh.hpp"

using namespace std;

int main(int argc, char* argv[]) {
    int num_inputs = 4;
    if (argc != (num_inputs + 1))
    {
        cout << "Invalid number of arguments. expected " << num_inputs << endl;
        return -1;
    }
    double theta1 = atof(argv[1]);
    double theta2 = atof(argv[2]);
    double theta3 = atof(argv[3]);
    double d = atof(argv[4]);
    double L = 1;

    Eigen::Matrix4d T01 = DH(0, 0, 0, theta1);
    Eigen::Matrix4d T12 = DH(0, L, 0, theta2);
    Eigen::Matrix4d T23 = DH(0, L, 0, theta3);
    Eigen::Matrix4d T34 = DH(EIGEN_PI, 0, d, 0);

    Eigen::Matrix4d T04 = T01 * T12 * T23 * T34;
    std::cout << T04 << std::endl;
    return 0;
}