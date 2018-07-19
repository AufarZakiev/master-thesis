#include <gtest/gtest.h>
#include <headers/dakai_algo.h>

TEST(VectorDistanceTest, ShouldPass)
{
  Eigen::Vector2d v1;
  Eigen::Vector2d v2;
  v1 << 1.0, 1.0;
  v2 << 1, 1;
  EXPECT_EQ(0, getVectorDistance(v1, v2));
  v1 << 3, 3;
  v2 << 2, 2;
  EXPECT_EQ(sqrt(2), getVectorDistance(v1, v2));
}

TEST(VectorLengthTest, ShouldPass)
{
  Eigen::Vector2d v;
  v << 3, 3;
  EXPECT_EQ(sqrt(18), getVectorLength(v));
  v << 3, 4;
  EXPECT_EQ(5, getVectorLength(v));
  v << -3, -4;
  EXPECT_EQ(5, getVectorLength(v));
}

TEST(isObjectOnLineSegmentTest, ShouldPass)
{
  Eigen::Vector2d v1;
  Eigen::Vector2d v2;
  Eigen::Vector2d o;
  v1 << 3, 3;
  v2 << 2, 2;
  o << 2.5, 2.5;
  EXPECT_EQ(true, isObjectOnLineSegment(o, v1, v2));
  v1 << -3, 3;
  v2 << -2, 2;
  o << -2.5, 2.5;
  EXPECT_EQ(true, isObjectOnLineSegment(o, v1, v2));
  v1 << -4, -3;
  v2 << -1, -1;
  o << -2.5, -2;
  EXPECT_EQ(true, isObjectOnLineSegment(o, v1, v2));
  v1 << 3, 3;
  v2 << 2, 2;
  o << 1, 1;
  EXPECT_EQ(false, isObjectOnLineSegment(o, v1, v2));
  v1 << -3, 3;
  v2 << -2, 2;
  o << -4, 4;
  EXPECT_EQ(false, isObjectOnLineSegment(o, v1, v2));
  v1 << -4, -3;
  v2 << -1, -1;
  o << -2, -2;
  EXPECT_EQ(false, isObjectOnLineSegment(o, v1, v2));
}

TEST(isObjectInDSpaceTest, NonBorderCases)
{
  Eigen::Vector2d v1;
  Eigen::Vector2d v2;
  Eigen::Vector2d o;
  v1 << 3, 3;
  v2 << 2, 2;
  o << 2.5, 2.5;
  EXPECT_EQ(true, isObjectInDSpace(o, v1, v2));
  v1 << -3, 3;
  v2 << -2, 2;
  o << -2.5, 2.5;
  EXPECT_EQ(true, isObjectInDSpace(o, v1, v2));
  v1 << -4, -3;
  v2 << -1, -1;
  o << -2.5, -2;
  EXPECT_EQ(true, isObjectInDSpace(o, v1, v2));
  v1 << -3, -3;
  v2 << -1, -1;
  o << -3, -1;
  EXPECT_EQ(true, isObjectInDSpace(o, v1, v2));
  o << -4, 0;
  EXPECT_EQ(true, isObjectInDSpace(o, v1, v2));
  o << -5, 1;
  EXPECT_EQ(true, isObjectInDSpace(o, v1, v2));
  o << -2.5, -0.5;
  EXPECT_EQ(true, isObjectInDSpace(o, v1, v2));
  v1 << -1, -1;
  v2 << 1, 1;
  o << 2, -2;
  EXPECT_EQ(true, isObjectInDSpace(o, v1, v2));
  v1 << 3, 3;
  v2 << 2, 2;
  o << 1, 1;
  EXPECT_EQ(false, isObjectInDSpace(o, v1, v2));
  v1 << -3, 3;
  v2 << -2, 2;
  o << -4, 4;
  EXPECT_EQ(false, isObjectInDSpace(o, v1, v2));
  v1 << -4, -3;
  v2 << -1, -1;
  o << -2, -2;
  EXPECT_EQ(true, isObjectInDSpace(o, v1, v2));
  o << -4, -4;
  EXPECT_EQ(false, isObjectInDSpace(o, v1, v2));
}

TEST(isObjectInDSpaceTest, BorderCases)
{
  Eigen::Vector2d v1;
  Eigen::Vector2d v2;
  Eigen::Vector2d o;
  v1 << 1, 1;
  v2 << 3, 3;
  o << 5, 1;
  EXPECT_EQ(false, isObjectInDSpace(o, v1, v2));
  v1 << 1, 1;
  v2 << 3, 3;
  o << 2, 0;
  EXPECT_EQ(false, isObjectInDSpace(o, v1, v2));
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
