#include <iostream>
#include "dh.hpp"

using namespace std;

int main(int argc, char* argv[]) {
    int num_inputs = 3;
    if (argc != (num_inputs + 1))
    {
        cout << "Invalid number of arguments. expected " << num_inputs << endl;
        return -1;
    }

    double theta1 = atof(argv[1]);
    double theta2 = atof(argv[2]);
    double theta3 = atof(argv[3]);
    double L = 1;

    Eigen::Matrix4d T01 = DH(0, 0, 0, theta1);
    Eigen::Matrix4d T12 = DH(0, L, 0, theta2);
    Eigen::Matrix4d T23 = DH(0, L, 0, theta3);

    Eigen::Matrix4d T03 = T01 * T12 * T23;
    std::cout << T03 << std::endl;
    return 0;
}