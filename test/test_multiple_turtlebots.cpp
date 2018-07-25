#include <gtest/gtest.h>
#include "../include/headers/DakaiAlgo.h"

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

TEST(getProjectionPhiTest, ShouldPass)
{
  Vector_t p;
  Vector_t q;
  Vector_t a;
  p << 2, 3;
  q << 3, -2;
  EXPECT_EQ(p, getProjectionPhi(p, q));
  p << 4, 7;
  q << 8, 0;
  a << 0, 7;
  EXPECT_EQ(a, getProjectionPhi(p, q));
}

TEST(isVectorInGraphTest, ShouldPass)
{
  RigidGraph rg;
  Position_t v1;
  Position_t v2;
  v1 << 0, 0;
  v2 << 1, 1;
  RigidObject r1(v1);
  RigidObject r2(v2);

  RigidObjectDesc r1_d = boost::add_vertex(r1, rg);
  RigidObjectDesc r2_d = boost::add_vertex(r2, rg);
  EXPECT_EQ(false, isVectorInGraph(r1, r2, rg));
  boost::add_edge(r1_d, r2_d, rg);
  EXPECT_EQ(true, isVectorInGraph(r1, r2, rg));
  boost::remove_edge(r1_d, r2_d, rg);
}

TEST(angleBetweenVectorsInRadiansTest, ShouldPass)
{
  Vector_t v1;
  Vector_t v2;
  v1 << 1, 0;
  v2 << 0, 1;
  EXPECT_EQ(M_PI / 2, angleBetweenVectorsInRadians(v1, v2));
  EXPECT_EQ(-M_PI / 2, angleBetweenVectorsInRadians(v2, v1));
  v1 << 1, 0;
  v2 << 1, 1;
  EXPECT_EQ(M_PI / 4, angleBetweenVectorsInRadians(v1, v2));
  EXPECT_EQ(-M_PI / 4, angleBetweenVectorsInRadians(v2, v1));
  v1 << 1, 0;
  v2 << -1, 0;
  EXPECT_EQ(M_PI, angleBetweenVectorsInRadians(v1, v2));
  EXPECT_EQ(-M_PI, angleBetweenVectorsInRadians(v2, v1));
}

TEST(isObjectInTSetTest, ShouldPass)
{
  Variables& v = Variables::getInstance();
  v.setParam("edge_deletion_distance", 10.0f);

  RigidGraph rg;
  Position_t v1;
  Position_t v2;
  Position_t v3;
  v1 << 0, 0;
  v2 << 1, 0;
  v3 << 0.5, 0.5;
  RigidObject i(v1);
  RigidObject j(v2);
  RigidObject m(v3);

  RigidObjectDesc i_d = boost::add_vertex(i, rg);
  RigidObjectDesc j_d = boost::add_vertex(j, rg);
  RigidObjectDesc m_d = boost::add_vertex(m, rg);

  boost::add_edge(i_d, j_d, rg);
  boost::add_edge(j_d, m_d, rg);
  boost::add_edge(m_d, i_d, rg);

  EXPECT_EQ(true, isObjectInTSet(i, j, m, rg, v));

  boost::remove_edge(i_d, j_d, rg);
  EXPECT_EQ(false, isObjectInTSet(i, j, m, rg, v));

  boost::add_edge(i_d, j_d, rg);
  EXPECT_EQ(true, isObjectInTSet(i, j, m, rg, v));

  v3 << -0.5, -0.5;
  m.setPosition(v3);
  EXPECT_EQ(false, isObjectInTSet(i, j, m, rg, v));
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
