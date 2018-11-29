#include <gtest/gtest.h>
#include "../include/headers/matplotlibcpp.h"  // uses this library https://github.com/lava/matplotlib-cpp to draw plots

#include "../include/headers/classes.h"
#include "../include/headers/helper_functions.h"
#include "../include/headers/geometric_functions.h"

TEST(closestDetectedObstacleTest, ShouldPass)
{
  Variables& v = Variables::getInstance();

  Vector_t v1, v2, v3;
  v1 << 10.0, 10.0;
  v2 << 5.0, 5.0;
  Obstacle o1(v1);
  Obstacle o2(v2);
  ObstacleGraph og;
  boost::add_vertex(o1, og);
  boost::add_vertex(o2, og);

  v3 << 10.0, 6.0;
  Robot r(v3);
  Obstacle o = closestDetectedObstacle(r, og, v).value();
  EXPECT_EQ(o1.getPosition(), o.getPosition());

  v3 << 0.0, 0.0;
  r.setPosition(v3);
  o = closestDetectedObstacle(r, og, v).value();
  EXPECT_EQ(o2.getPosition(), o.getPosition());

  v3 << 8.0, 8.0;
  r.setPosition(v3);
  o = closestDetectedObstacle(r, og, v).value();
  EXPECT_EQ(o1.getPosition(), o.getPosition());
}

TEST(separateNeighbourRobotsBehindAndFront, ShouldPass)
{
  Vector_t v1(5, 5), v2(6, 6), v3(10, 5), v4(3, 3), v5(0, 2);
  Robot r1(v1, Vector_t(1, 1)), r2(v2), r3(v3), r4(v4), r5(v5);
  RobotGraph rg, behind, front;

  boost::add_vertex(r2, rg);
  boost::add_vertex(r3, rg);
  boost::add_vertex(r4, rg);
  boost::add_vertex(r5, rg);

  std::pair p = separateNeighbourRobotsBehindAndFront(r1, rg);
  behind = p.first;
  front = p.second;
  EXPECT_EQ(findVertexInGraph(r1, behind).second, false);
  EXPECT_EQ(findVertexInGraph(r1, front).second, false);
  EXPECT_EQ(findVertexInGraph(r2, front).second, true);
  EXPECT_EQ(findVertexInGraph(r3, front).second, true);
  EXPECT_EQ(findVertexInGraph(r4, behind).second, true);
  EXPECT_EQ(findVertexInGraph(r5, behind).second, true);

  r1.setSpeedDirection(Vector_t(-1, -1));
  p = separateNeighbourRobotsBehindAndFront(r1, rg);
  behind = p.first;
  front = p.second;
  EXPECT_EQ(findVertexInGraph(r1, behind).second, false);
  EXPECT_EQ(findVertexInGraph(r1, front).second, false);
  EXPECT_EQ(findVertexInGraph(r2, front).second, false);
  EXPECT_EQ(findVertexInGraph(r3, front).second, false);
  EXPECT_EQ(findVertexInGraph(r4, behind).second, false);
  EXPECT_EQ(findVertexInGraph(r5, behind).second, false);

  r1.setSpeedDirection(Vector_t(1, -1));
  p = separateNeighbourRobotsBehindAndFront(r1, rg);
  behind = p.first;
  front = p.second;
  EXPECT_EQ(findVertexInGraph(r1, behind).second, false);
  EXPECT_EQ(findVertexInGraph(r1, front).second, false);
  EXPECT_EQ(findVertexInGraph(r2, front).second, false);
  EXPECT_EQ(findVertexInGraph(r3, front).second, true);
  EXPECT_EQ(findVertexInGraph(r4, behind).second, true);
  EXPECT_EQ(findVertexInGraph(r5, behind).second, true);
}

TEST(closingObstaclesInDSpace, ShouldPass)
{
  Robot r1(Position_t(5, 5), Vector_t(1, 1)), r2(Position_t(9, 5));
  Obstacle o1(Position_t(8, 8)), o2(Position_t(10, 5)), o3(Position_t(3, 0));

  ObstacleGraph og;
  boost::add_vertex(o1,og);
  boost::add_vertex(o2,og);
  boost::add_vertex(o3,og);

  auto closing_og = closingObstaclesInDSpace(r1,r2,og);
  EXPECT_EQ(findVertexInGraph(o1,closing_og).second, true);
  EXPECT_EQ(findVertexInGraph(o2,closing_og).second, false);
  EXPECT_EQ(findVertexInGraph(o3,closing_og).second, false);
}

TEST(closestObstacleToLOSinDSpaceAtFront, ShouldPass)
{
  Robot r1(Position_t(5, 5), Vector_t(1, 1)), r2(Position_t(9, 5));
  Obstacle o1(Position_t(8, 8)), o2(Position_t(10, 5)), o3(Position_t(3, 0)), o4(Position_t(6,4));

  ObstacleGraph og;
  auto closest = closestObstacleToLOSinDSpaceAtFront(r1,r2,og);
  EXPECT_EQ((bool)closest, false);

  boost::add_vertex(o1,og);
  boost::add_vertex(o2,og);
  boost::add_vertex(o3,og);
  boost::add_vertex(o4,og);

  closest = closestObstacleToLOSinDSpaceAtFront(r1,r2,og);
  EXPECT_EQ(closest.value().getPosition(), o4.getPosition());

  closest = closestObstacleToLOSinDSpaceAtFront(r2,r1,og);
  EXPECT_EQ(closest.value().getPosition(), o4.getPosition());

  r2.setPosition(Position_t(11,5));
  closest = closestObstacleToLOSinDSpaceAtFront(r2,r1,og);
  EXPECT_EQ(closest.value().getPosition(), o2.getPosition());
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
