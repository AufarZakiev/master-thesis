#include <gtest/gtest.h>
#include <headers/dakai_algo.h>

TEST(VectorDistanceTest, ShouldPass)
{
  Eigen::Vector2d v1;
  Eigen::Vector2d v2;
  v1 << 1.0, 1.0;
  v2 << 1, 1;
  ASSERT_EQ(0, getVectorDistance(v1, v2));
  v1 << 3, 3;
  v2 << 2, 2;
  ASSERT_EQ(sqrt(2), getVectorDistance(v1, v2));
}

TEST(VectorLengthTest, ShouldPass)
{
  Eigen::Vector2d v;
  v << 3, 3;
  ASSERT_EQ(sqrt(18), getVectorLength(v));
  v << 3, 4;
  ASSERT_EQ(5, getVectorLength(v));
  v << -3, -4;
  ASSERT_EQ(5, getVectorLength(v));
}

TEST(isObjectOnLineSegmentTest, ShouldPass)
{
  Eigen::Vector2d v1;
  Eigen::Vector2d v2;
  Eigen::Vector2d o;
  v1 << 3, 3;
  v2 << 2, 2;
  o << 2.5, 2.5;
  ASSERT_EQ(true, isObjectOnLineSegment(o, v1, v2));
  v1 << -3, 3;
  v2 << -2, 2;
  o << -2.5, 2.5;
  ASSERT_EQ(true, isObjectOnLineSegment(o, v1, v2));
  v1 << -4, -3;
  v2 << -1, -1;
  o << -2.5, -2;
  ASSERT_EQ(true, isObjectOnLineSegment(o, v1, v2));
  v1 << 3, 3;
  v2 << 2, 2;
  o << 1, 1;
  ASSERT_EQ(false, isObjectOnLineSegment(o, v1, v2));
  v1 << -3, 3;
  v2 << -2, 2;
  o << -4, 4;
  ASSERT_EQ(false, isObjectOnLineSegment(o, v1, v2));
  v1 << -4, -3;
  v2 << -1, -1;
  o << -2, -2;
  ASSERT_EQ(false, isObjectOnLineSegment(o, v1, v2));
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
