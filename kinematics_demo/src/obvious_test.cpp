#include <gtest/gtest.h>
// Declare a test
TEST(obvious, obvious)
{
    EXPECT_TRUE(true);
    EXPECT_FALSE(false);
    EXPECT_EQ(true, true);
    EXPECT_NE(true, false);
    EXPECT_NEAR(0.0, 0.0, 1.0);
    EXPECT_GT(1, 0);
    EXPECT_LT(0, 1);
}
