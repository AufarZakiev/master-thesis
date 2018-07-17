#include <gtest/gtest.h>
#include <headers/dakai_algo.h>

TEST(VectorDistanceTest, ShouldPass)
{
  Eigen::Vector2d v1;
  Eigen::Vector2d v2;
  v1 << 1.0, 1.0;
  v2 << 1, 1;
  ASSERT_EQ(0, vector_distance(v1, v2));
  v1 << 3, 3;
  v2 << 2, 2;
  ASSERT_EQ(sqrt(2), vector_distance(v1, v2));
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
