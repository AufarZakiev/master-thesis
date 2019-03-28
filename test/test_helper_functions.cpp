#include <gtest/gtest.h>
#include "../include/headers/matplotlibcpp.h"  // uses this library https://github.com/lava/matplotlib-cpp to draw plots

#include "../include/headers/classes.h"
#include "../include/headers/helper_functions.h"
#include "../include/headers/geometric_functions.h"
#include "../include/headers/field_functions.h"

TEST(closestDetectedObstacleTest, ShouldPass)
{
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
  Obstacle o = closestDetectedObstacle(r, og).value();
  EXPECT_EQ(o1.getObstacleID(), o.getObstacleID());

  v3 << 0.0, 0.0;
  r.setPosition(v3);
  o = closestDetectedObstacle(r, og).value();
  EXPECT_EQ(o2.getObstacleID(), o.getObstacleID());

  v3 << 8.0, 8.0;
  r.setPosition(v3);
  o = closestDetectedObstacle(r, og).value();
  EXPECT_EQ(o1.getObstacleID(), o.getObstacleID());
}

TEST(separateNeighbourRobotsBehindAndFrontTest, ShouldPass)
{
  Robot r1(Position_t(5, 5), Vector_t(1, 1)), r2(Position_t(6, 6)), r3(Position_t(10, 5)), r4(Position_t(3, 3)),
      r5(Position_t(0, 2));
  RobotGraph rg, behind, front;

  boost::add_vertex(r2, rg);
  boost::add_vertex(r3, rg);
  boost::add_vertex(r4, rg);
  boost::add_vertex(r5, rg);

  std::pair p = separateNeighbourRobotsBehindAndFront(r1, rg);
  behind = p.first;
  front = p.second;
  EXPECT_EQ((bool)findRobotInGraph(r1, behind), false);
  EXPECT_EQ((bool)findRobotInGraph(r1, front), false);
  EXPECT_EQ((bool)findRobotInGraph(r2, front), true);
  EXPECT_EQ((bool)findRobotInGraph(r3, front), true);
  EXPECT_EQ((bool)findRobotInGraph(r4, behind), true);
  EXPECT_EQ((bool)findRobotInGraph(r5, behind), true);

  r1.setSpeedDirection(Vector_t(-1, -1));
  p = separateNeighbourRobotsBehindAndFront(r1, rg);
  behind = p.first;
  front = p.second;
  EXPECT_EQ((bool)findRobotInGraph(r1, behind), false);
  EXPECT_EQ((bool)findRobotInGraph(r1, front), false);
  EXPECT_EQ((bool)findRobotInGraph(r2, front), false);
  EXPECT_EQ((bool)findRobotInGraph(r3, front), false);
  EXPECT_EQ((bool)findRobotInGraph(r4, behind), false);
  EXPECT_EQ((bool)findRobotInGraph(r5, behind), false);

  r1.setSpeedDirection(Vector_t(1, -1));
  p = separateNeighbourRobotsBehindAndFront(r1, rg);
  behind = p.first;
  front = p.second;
  EXPECT_EQ((bool)findRobotInGraph(r1, behind), false);
  EXPECT_EQ((bool)findRobotInGraph(r1, front), false);
  EXPECT_EQ((bool)findRobotInGraph(r2, front), false);
  EXPECT_EQ((bool)findRobotInGraph(r3, front), true);
  EXPECT_EQ((bool)findRobotInGraph(r4, behind), true);
  EXPECT_EQ((bool)findRobotInGraph(r5, behind), true);
}

TEST(closingObstaclesInDSpaceTest, ShouldPass)
{
  Robot r1(Position_t(5, 5), Vector_t(1, 1)), r2(Position_t(9, 5));
  Obstacle o1(Position_t(8, 8)), o2(Position_t(10, 5)), o3(Position_t(3, 0));

  ObstacleGraph og;
  boost::add_vertex(o1, og);
  boost::add_vertex(o2, og);
  boost::add_vertex(o3, og);

  auto closing_og = closingObstaclesInDSpace(r1, r2, og);
  EXPECT_EQ((bool)findObstacleInGraph(o1, closing_og), true);
  EXPECT_EQ((bool)findObstacleInGraph(o2, closing_og), false);
  EXPECT_EQ((bool)findObstacleInGraph(o3, closing_og), false);
}

TEST(closestObstacleToLOSinDSpaceAtFrontTest, ShouldPass)
{
  Robot r1(Position_t(5, 5), Vector_t(1, 1)), r2(Position_t(9, 5));
  Obstacle o1(Position_t(8, 8)), o2(Position_t(10, 5)), o3(Position_t(3, 0)), o4(Position_t(6, 4));

  ObstacleGraph og;
  auto closest = closestObstacleToLOSinDSpaceAtFront(r1, r2, og);
  EXPECT_EQ((bool)closest, false);

  boost::add_vertex(o1, og);
  boost::add_vertex(o2, og);
  boost::add_vertex(o3, og);
  boost::add_vertex(o4, og);

  closest = closestObstacleToLOSinDSpaceAtFront(r1, r2, og);
  EXPECT_EQ(closest.value().getObstacleID(), o4.getObstacleID());

  closest = closestObstacleToLOSinDSpaceAtFront(r2, r1, og);
  EXPECT_EQ(closest.value().getObstacleID(), o4.getObstacleID());

  r2.setPosition(Position_t(11, 5));
  closest = closestObstacleToLOSinDSpaceAtFront(r2, r1, og);
  EXPECT_EQ(closest.value().getObstacleID(), o2.getObstacleID());
}

TEST(printPlotWithArrowsTest, ShouldPass)
{
  Variables v = Variables();
  v.setParam("robots_avoidance_distance", 1.0);
  v.setParam("desired_distance", 3.0);
  v.setParam("neighbourhood_distance", 4.0);
  v.setParam("k1", 10);
  v.setParam("k2", 10);

  v.setParam("los_clearance_care_distance", 0.4);
  v.setParam("los_clearance_distance", 0.2);
  v.setParam("small_positive_constant", 0.2);
  Robot r1(Position_t(1, 1), Vector_t(10, 10));
  Robot r2(Position_t(4, 4), Vector_t(4, -4));
  Robot r3(Position_t(10, 6), Vector_t(-2, 2));
  Robot r4(Position_t(8, 5), Vector_t(-1, -5));

  RobotGraph rg;
  boost::add_vertex(r1, rg);

  Obstacle o1(Position_t(3, 3));

  ObstacleGraph og;
  boost::add_vertex(o1, og);

  printPlotWithArrows("LOS arrows test.png", "LOS", 30, 30, 1, rg, std::function(&LOSPreservePotential), rg, og, v);

  boost::add_vertex(r2, rg);
  boost::add_vertex(r3, rg);
  boost::add_vertex(r4, rg);

  //  printPlotWithArrows("Interrobot arrows test.png", "Interrobot", 30, 30, 1, rg,
  //                      std::function(&interrobotCollisionPotential), rg, og, v);
  //  printPlotWithArrows("Interrobot arrows test_0_90.png", "Interrobot", 0, 90, 1, rg,
  //                      std::function(&interrobotCollisionPotential), rg, og, v);
  printPlotWithArrows("Arrows test_cohesion_0_90.png", "Cohesion", 0, 90, 1, rg, std::function(&cohesionPotential), rg,
                      og, v);
}

TEST(getNeighboursTest, ShouldPass)
{
  Variables v = Variables();
  v.setParam("neighbourhood_distance", 2.0);

  Robot r1(Position_t(5.0, 5.0));
  Robot r2(Position_t(4.0, 4.0));
  Robot r3(Position_t(1.0, 3.0));

  RobotGraph rg;
  boost::add_vertex(r2, rg);
  boost::add_vertex(r3, rg);
  auto neighbours = getNeighbourRobots(r1, rg, v);
  EXPECT_EQ((bool)findRobotInGraph(r2, neighbours), true);
  EXPECT_EQ((bool)findRobotInGraph(r3, neighbours), false);
}

TEST(getNeighboursPreservedTest, ShouldPass)
{
  Variables v = Variables();
  v.setParam("neighbourhood_distance", 4.0);
  v.setParam("robots_avoidance_distance", 1.0);
  v.setParam("edge_deletion_distance", 3.0 * sin(M_PI / 3));

  Robot r1(Position_t(3.0, 5.0));
  Robot r2(Position_t(2.0, 4.0));
  Robot r3(Position_t(4.0, 4.0));

  RobotGraph rg;
  boost::add_vertex(r2, rg);
  boost::add_vertex(r3, rg);
  auto neighbours_preserved = getNeighbourPreservedRobots(r1, rg, v);
  EXPECT_EQ((bool)findRobotInGraph(r2, neighbours_preserved), true);
  EXPECT_EQ((bool)findRobotInGraph(r3, neighbours_preserved), true);
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
