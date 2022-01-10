// g++ -I /path/to/eigen/ my_program.cpp -o my_program
// g++ my_program.cpp -o my_program
// sudo apt install libeigen3-dev

#include <iostream>
#include <eigen3/Eigen/Dense>


using namespace std;


void print(Eigen::Matrix3d A)
{
    cout << A << endl;
}

int main()
{
    Eigen::Matrix3d matA;
    matA << 1, 2, 3,
            4, 5, 6,
            7, 8, 9;
    Eigen::Matrix3d matB;
    matB << 1, 2, 3,
            1, 2, 3,
            1, 2, 3;

    Eigen::Matrix3d matC = matA * matB;
    print(matC);
    /* Result:
        6 12 18
        15 30 45
        24 48 72
    */
    return 0;
}