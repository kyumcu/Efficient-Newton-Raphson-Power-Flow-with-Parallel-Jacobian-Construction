#include <gtest/gtest.h>

int
Add(int a, int b)
{
  return a + b;
}
TEST(AdditionTest, HandlesPositiveNumbers)
{
  EXPECT_EQ(3, Add(1, 2));
}

TEST(AdditionTest, HandlesNegativeNumbers)
{
  EXPECT_EQ(-1, Add(-1, 0));
}

int
main(int argc, char** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  ::testing::GTEST_FLAG(break_on_failure) = true;
  //::testing::GTEST_FLAG(filter) = "MatpowerTests.case2868rte:MatpowerTests.case9";
  //://"MatpowerTests.case6515rte:MatpowerTests.case1354pegase";

  return RUN_ALL_TESTS();
}