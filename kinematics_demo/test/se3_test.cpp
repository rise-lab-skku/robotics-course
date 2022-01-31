// Bring in my package's API, which is what I'm testing
#include "kinematics_demo/se3.hpp"

// Bring in gtest
#include <gtest/gtest.h>
#include <cstdlib>
#include <tf2_eigen/tf2_eigen.h>
#include <tf/tf.h>

double randRad()
{
    return (double)rand() / (double)RAND_MAX * M_PI;
}

// Declare a test
TEST(TestSuite, testCase1)
{
    tf2::Quaternion q;

    // for (int i = 0; i < 100; i++)
    // {
    //     q.setRPY(randRad(), randRad(), randRad());
    //     Eigen::Matrix3d R;
    //     tf2::convert(q, R);
    //     Eigen::Vector3d so3;
    //     SO3::log(R, so3);
    //     Eigen::Matrix3d R_exp;
    //     SO3::exp(so3, R_exp);
    //     EXPECT_TRUE(R.isApprox(R_exp, 1e-6));
    // }
    // <test things here, calling EXPECT_* and/or ASSERT_* macros as needed>
}

// Declare another test
// TEST(TestSuite, testCase2)
// {
// <test things here, calling EXPECT_* and/or ASSERT_* macros as needed>
// }

// Run all the tests that were declared with TEST()
int main(int argc, char **argv){
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "tester");
  ros::NodeHandle nh;
  return RUN_ALL_TESTS();
}