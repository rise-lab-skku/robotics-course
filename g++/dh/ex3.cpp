#include <iostream>
#include <vector>
#include "dh.hpp"

using namespace std;

int main(int argc, char* argv[]) {
    int num_inputs = 6;
    if (argc != (num_inputs + 1))
    {
        cout << "Invalid number of arguments. expected " << num_inputs << endl;
        return -1;
    }
    vector<double> theta;
    for (int i=1; i<=num_inputs; i++)
        theta.push_back(atof(argv[i]));

    double a2 = 1;
    double a3 = 1;
    double d3 = 0.2;
    double d4 = 0.2;

    Eigen::Matrix4d T06 =
        DH(          0,  0,  0, theta[0]) *  // T01
        DH(-EIGEN_PI/2,  0,  0, theta[1]) *  // T12
        DH(          0, a2, d3, theta[2]) *  // T23
        DH(-EIGEN_PI/2, a3, d4, theta[3]) *  // T34
        DH( EIGEN_PI/2,  0,  0, theta[4]) *  // T45
        DH(-EIGEN_PI/2,  0,  0, theta[5]);   // T56

    std::cout << T06 << std::endl;
    return 0;
}