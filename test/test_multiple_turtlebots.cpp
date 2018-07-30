#include <gtest/gtest.h>
#include "../include/headers/DakaiAlgo.h"
#include "../include/headers/matplotlibcpp.h"  // uses this library https://github.com/lava/matplotlib-cpp to draw plots

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

  RigidObject r1(v1);
  RigidObject r2(v2);
  RigidObject ro(o);

  EXPECT_EQ(true, isObjectOnLineSegment(ro, r1, r2));

  v1 << -3, 3;
  v2 << -2, 2;
  o << -2.5, 2.5;
  r1.setPosition(v1);
  r2.setPosition(v2);
  ro.setPosition(o);
  EXPECT_EQ(true, isObjectOnLineSegment(ro, r1, r2));

  v1 << -4, -3;
  v2 << -1, -1;
  o << -2.5, -2;
  r1.setPosition(v1);
  r2.setPosition(v2);
  ro.setPosition(o);
  EXPECT_EQ(true, isObjectOnLineSegment(ro, r1, r2));

  v1 << 3, 3;
  v2 << 2, 2;
  o << 1, 1;
  r1.setPosition(v1);
  r2.setPosition(v2);
  ro.setPosition(o);
  EXPECT_EQ(false, isObjectOnLineSegment(ro, r1, r2));

  v1 << -3, 3;
  v2 << -2, 2;
  o << -4, 4;
  r1.setPosition(v1);
  r2.setPosition(v2);
  ro.setPosition(o);
  EXPECT_EQ(false, isObjectOnLineSegment(ro, r1, r2));

  v1 << -4, -3;
  v2 << -1, -1;
  o << -2, -2;
  r1.setPosition(v1);
  r2.setPosition(v2);
  ro.setPosition(o);
  EXPECT_EQ(false, isObjectOnLineSegment(ro, r1, r2));
}

TEST(isObjectInDSpaceTest, NonBorderCases)
{
  Eigen::Vector2d v1;
  Eigen::Vector2d v2;
  Eigen::Vector2d o;
  v1 << 3, 3;
  v2 << 2, 2;
  o << 2.5, 2.5;

  RigidObject r1(v1);
  RigidObject r2(v2);
  RigidObject ro(o);

  EXPECT_EQ(true, isObjectInDSpace(ro, r1, r2));

  v1 << -3, 3;
  v2 << -2, 2;
  o << -2.5, 2.5;
  r1.setPosition(v1);
  r2.setPosition(v2);
  ro.setPosition(o);
  EXPECT_EQ(true, isObjectInDSpace(ro, r1, r2));

  v1 << -4, -3;
  v2 << -1, -1;
  o << -2.5, -2;
  r1.setPosition(v1);
  r2.setPosition(v2);
  ro.setPosition(o);
  EXPECT_EQ(true, isObjectInDSpace(ro, r1, r2));

  v1 << -3, -3;
  v2 << -1, -1;
  o << -3, -1;
  r1.setPosition(v1);
  r2.setPosition(v2);
  ro.setPosition(o);
  EXPECT_EQ(true, isObjectInDSpace(ro, r1, r2));

  o << -4, 0;
  ro.setPosition(o);
  EXPECT_EQ(true, isObjectInDSpace(ro, r1, r2));

  o << -5, 1;
  ro.setPosition(o);
  EXPECT_EQ(true, isObjectInDSpace(ro, r1, r2));

  o << -2.5, -0.5;
  ro.setPosition(o);
  EXPECT_EQ(true, isObjectInDSpace(ro, r1, r2));

  v1 << -1, -1;
  v2 << 1, 1;
  o << 2, -2;
  r1.setPosition(v1);
  r2.setPosition(v2);
  ro.setPosition(o);
  EXPECT_EQ(true, isObjectInDSpace(ro, r1, r2));

  v1 << 3, 3;
  v2 << 2, 2;
  o << 1, 1;
  r1.setPosition(v1);
  r2.setPosition(v2);
  ro.setPosition(o);
  EXPECT_EQ(false, isObjectInDSpace(ro, r1, r2));

  v1 << -3, 3;
  v2 << -2, 2;
  o << -4, 4;
  r1.setPosition(v1);
  r2.setPosition(v2);
  ro.setPosition(o);
  EXPECT_EQ(false, isObjectInDSpace(ro, r1, r2));

  v1 << -4, -3;
  v2 << -1, -1;
  o << -2, -2;
  r1.setPosition(v1);
  r2.setPosition(v2);
  ro.setPosition(o);
  EXPECT_EQ(true, isObjectInDSpace(ro, r1, r2));

  o << -4, -4;
  ro.setPosition(o);
  EXPECT_EQ(false, isObjectInDSpace(ro, r1, r2));
}

TEST(isObjectInDSpaceTest, BorderCases)
{
  Eigen::Vector2d v1;
  Eigen::Vector2d v2;
  Eigen::Vector2d o;
  v1 << 1, 1;
  v2 << 3, 3;
  o << 5, 1;

  RigidObject r1(v1);
  RigidObject r2(v2);
  RigidObject ro(o);

  EXPECT_EQ(false, isObjectInDSpace(ro, r1, r2));
  v1 << 1, 1;
  v2 << 3, 3;
  o << 2, 0;
  r1.setPosition(v1);
  r2.setPosition(v2);
  ro.setPosition(o);
  EXPECT_EQ(false, isObjectInDSpace(ro, r1, r2));
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

  v3 << 0.5, -0.5;
  m.setPosition(v3);
  EXPECT_EQ(false, isObjectInTSet(i, j, m, rg, v));
}

TEST(isObjectInDashedTSetTest, ShouldPass)
{
  Variables& v = Variables::getInstance();
  v.setParam("neighbourhood_distance", 13.0f);
  v.setParam("robots_avoidance_distance", 10.0f);

  RigidGraph rg;
  Position_t v1;
  Position_t v2;
  Position_t v3;
  v1 << 0, 0;
  v2 << 10, 0;
  v3 << 5, 12;
  RigidObject m(v1);
  RigidObject i(v2);
  RigidObject j(v3);

  RigidObjectDesc i_d = boost::add_vertex(i, rg);
  RigidObjectDesc j_d = boost::add_vertex(j, rg);
  RigidObjectDesc m_d = boost::add_vertex(m, rg);

  boost::add_edge(i_d, j_d, rg);
  boost::add_edge(j_d, m_d, rg);
  boost::add_edge(m_d, i_d, rg);

  EXPECT_EQ(true, isObjectInDashedTSet(i, j, m, rg, v));

  boost::remove_edge(i_d, j_d, rg);
  EXPECT_EQ(false, isObjectInDashedTSet(i, j, m, rg, v));

  boost::add_edge(i_d, j_d, rg);
  EXPECT_EQ(true, isObjectInDashedTSet(i, j, m, rg, v));

  v3 << 5, -12;
  m.setPosition(v3);
  EXPECT_EQ(false, isObjectInDashedTSet(i, j, m, rg, v));

  v3 << 5, -10;
  m.setPosition(v3);
  EXPECT_EQ(false, isObjectInDashedTSet(i, j, m, rg, v));
}

TEST(isEdgePreservedTest, ShouldPass)
{
  Variables& v = Variables::getInstance();
  v.setParam("neighbourhood_distance", 13.0f);
  v.setParam("robots_avoidance_distance", 1.0f);
  v.setParam("edge_deletion_distance", 5.0f);

  RigidGraph rg;
  Position_t v1, v2, v3, v4, v5, v6;
  v1 << 0, 0;
  v2 << 1, 0;
  v3 << 0.5, 0.5;
  v4 << 0.5, -0.5;
  v5 << -1, 0;
  v6 << -0.5, 0.5;
  Robot r1(v1);
  Robot r2(v2);
  Robot r3(v3);
  Robot r4(v4);
  Robot r5(v5);
  Robot r6(v6);

  RigidObjectDesc r1_d = boost::add_vertex(r1, rg);
  RigidObjectDesc r2_d = boost::add_vertex(r2, rg);
  RigidObjectDesc r3_d = boost::add_vertex(r3, rg);

  boost::add_edge(r1_d, r2_d, rg);
  boost::add_edge(r2_d, r3_d, rg);
  boost::add_edge(r3_d, r1_d, rg);
  EXPECT_EQ(false, isEdgePreserved(r1, r2, rg, v));

  boost::add_vertex(r4, rg);
  boost::add_vertex(r5, rg);
  boost::add_vertex(r6, rg);
  EXPECT_EQ(false, isEdgePreserved(r1, r2, rg, v));

  boost::remove_edge(r3_d, r1_d, rg);
  EXPECT_EQ(true, isEdgePreserved(r1, r2, rg, v));

  boost::remove_vertex(r3_d, rg);
  v3 << 6, 0;
  r3.setPosition(v3);
  r3_d = boost::add_vertex(r3, rg);
  boost::add_edge(r3_d, r1_d, rg);
  EXPECT_EQ(true, isEdgePreserved(r1, r2, rg, v));
}

TEST(partialDerivativeTest, ShouldPass)
{
  Variables& v = Variables::getInstance();
  double x = M_PI / 2;
  double EQUALITY_CASE;
  v.getParam("equality_case", EQUALITY_CASE);
  EXPECT_NEAR(partialDerivative(x, sin, v), 0, EQUALITY_CASE);
  EXPECT_NEAR(partialDerivative(x, cos, v), -1, EQUALITY_CASE);
  x = M_PI;
  EXPECT_NEAR(partialDerivative(x, sin, v), -1, EQUALITY_CASE);
  EXPECT_NEAR(partialDerivative(x, cos, v), 0, EQUALITY_CASE);
  x = 2;
  EXPECT_NEAR(partialDerivative(x, [](double x) { return x * x * x; }, v), 12, EQUALITY_CASE);
}

TEST(fullDerivativeTest, ShouldPass)
{
  Variables& v = Variables::getInstance();
  double EQUALITY_CASE;
  v.getParam("equality_case", EQUALITY_CASE);
  Position_t p;
  p << 1, 1;
  EXPECT_NEAR(fullDerivative(p, [](double x, double y) { return x * y; }, v), 2, EQUALITY_CASE);
  p << 2, 2;
  EXPECT_NEAR(fullDerivative(p, [](double x, double y) { return x * x * y * y; }, v), 32, EQUALITY_CASE);
}

TEST(partialInterrobotCollisionPotentialTest, ShouldPass)
{
  namespace plt = matplotlibcpp;

  Variables& v = Variables::getInstance();
  v.setParam("neighbourhood_distance", 1.0f);
  v.setParam("robots_avoidance_distance", 0.3f);
  v.setParam("desired_distance", 0.65f);
  double EQUALITY_CASE;
  v.getParam("equality_case", EQUALITY_CASE);
  // Prepare data.
  size_t n = 20000;
  std::vector<double> x(n), y(n);
  for (size_t i = 0; i < n; ++i)
  {
    x.at(i) = 0.3 + 0.7 * double(i) / (n - 1);
    y.at(i) = partialInterrobotCollisionPotential(0.3 + 0.7 * double(i) / (n - 1), v);
  }

  EXPECT_NEAR(y[0], 10, EQUALITY_CASE);
  EXPECT_NEAR(y[n / 2], 0, EQUALITY_CASE);
  EXPECT_NEAR(y[n - 1], 10, EQUALITY_CASE);

  // Set the size of output image = 1200x780 pixels
  plt::figure_size(1200, 780);
  // Plot line from given x and y data. Color is selected automatically.
  plt::named_plot("Phi Interrobot", x, y);
  // Set x-axis to interval [0,1000000]
  plt::xlim(0.25, 1.05);
  plt::ylim(-1, 12);
  // Enable legend.
  plt::legend();
  // Save the image (file format is determined by the extension)
  plt::save("./interrobot_potential.png");
}

TEST(partialObstacleCollisionPotentialTest, ShouldPass)
{
  namespace plt = matplotlibcpp;

  Variables& v = Variables::getInstance();
  v.setParam("obstacle_care_distance", 0.2f);
  v.setParam("obstacles_avoidance_distance", 0.1f);
  v.setParam("small_positive_constant", 0.1f);
  double EQUALITY_CASE;
  v.getParam("equality_case", EQUALITY_CASE);
  // Prepare data.
  size_t n = 20000;
  std::vector<double> x(n), y(n);
  for (size_t i = 0; i < n; ++i)
  {
    x.at(i) = 0.1 + 0.4 * double(i) / (n - 1);
    y.at(i) = partialObstacleCollisionPotential(0.1 + 0.4 * double(i) / (n - 1), v);
  }

  EXPECT_GT(y[0], 8);
  EXPECT_NEAR(y[n - 1], 0, EQUALITY_CASE);

  // Set the size of output image = 1200x780 pixels
  plt::figure_size(1200, 780);
  // Plot line from given x and y data. Color is selected automatically.
  plt::named_plot("Phi Obstacle", x, y);
  // Set x-axis to interval [0,1000000]
  plt::xlim(0.0, 0.5);
  plt::ylim(0, 10);
  // Enable legend.
  plt::legend();
  // Save the image (file format is determined by the extension)
  plt::save("./obstacle_potential.png");
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
