#include <gtest/gtest.h>
#include "../include/headers/matplotlibcpp.h"  // uses this library https://github.com/lava/matplotlib-cpp to draw plots

#include "../include/headers/classes.h"
#include "../include/headers/helper_functions.h"
#include "../include/headers/geometric_functions.h"
#include "../include/headers/field_functions.h"

TEST(closestDetectedObstacleTest, ShouldPass) {
  Variables &v = Variables::getInstance();

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

TEST(separateNeighbourRobotsBehindAndFrontTest, ShouldPass) {
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

TEST(closingObstaclesInDSpaceTest, ShouldPass) {
  Robot r1(Position_t(5, 5), Vector_t(1, 1)), r2(Position_t(9, 5));
  Obstacle o1(Position_t(8, 8)), o2(Position_t(10, 5)), o3(Position_t(3, 0));

  ObstacleGraph og;
  boost::add_vertex(o1, og);
  boost::add_vertex(o2, og);
  boost::add_vertex(o3, og);

  auto closing_og = closingObstaclesInDSpace(r1, r2, og);
  EXPECT_EQ(findVertexInGraph(o1, closing_og).second, true);
  EXPECT_EQ(findVertexInGraph(o2, closing_og).second, false);
  EXPECT_EQ(findVertexInGraph(o3, closing_og).second, false);
}

TEST(closestObstacleToLOSinDSpaceAtFrontTest, ShouldPass) {
  Robot r1(Position_t(5, 5), Vector_t(1, 1)), r2(Position_t(9, 5));
  Obstacle o1(Position_t(8, 8)), o2(Position_t(10, 5)), o3(Position_t(3, 0)), o4(Position_t(6, 4));

  ObstacleGraph og;
  auto closest = closestObstacleToLOSinDSpaceAtFront(r1, r2, og);
  EXPECT_EQ((bool) closest, false);

  boost::add_vertex(o1, og);
  boost::add_vertex(o2, og);
  boost::add_vertex(o3, og);
  boost::add_vertex(o4, og);

  closest = closestObstacleToLOSinDSpaceAtFront(r1, r2, og);
  EXPECT_EQ(closest.value().getPosition(), o4.getPosition());

  closest = closestObstacleToLOSinDSpaceAtFront(r2, r1, og);
  EXPECT_EQ(closest.value().getPosition(), o4.getPosition());

  r2.setPosition(Position_t(11, 5));
  closest = closestObstacleToLOSinDSpaceAtFront(r2, r1, og);
  EXPECT_EQ(closest.value().getPosition(), o2.getPosition());
}

TEST(printPlotWithArrowsTest, ShouldPass) {
  Variables &v = Variables::getInstance();
  v.setParam("robots_avoidance_distance", 1.0);
  v.setParam("desired_distance", 3.0);
  v.setParam("neighbourhood_distance", 4.0);
  v.setParam("k1", 10);
  v.setParam("k2", 10);

  v.setParam("los_clearance_care_distance", 0.4);
  v.setParam("los_clearance_distance", 0.2);
  v.setParam("small_positive_constant", 0.2);
  Robot r1(Vector_t(1, 1), Vector_t(10, 10));
  Robot r2(Vector_t(4, 4), Vector_t(4, -4));
  Robot r3(Vector_t(10, 6), Vector_t(-2, 2));
  Robot r4(Vector_t(8, 5), Vector_t(-1, -5));

  RobotGraph rg;
  boost::add_vertex(r1, rg);

  Obstacle o1(Vector_t(3, 3));

  ObstacleGraph og;
  boost::add_vertex(o1, og);

  printPlotWithArrows("LOS arrows test.png", "LOS", 30, 30, {r1, r2, r3, r4},
                      std::function(&LOSPreservePotential), rg, og, v);

  boost::add_vertex(r2, rg);
  boost::add_vertex(r3, rg);
  boost::add_vertex(r4, rg);

  printPlotWithArrows("Interrobot arrows test.png", "Interrobot", 30, 30, {r1, r2, r3, r4},
                      std::function(&interrobotCollisionPotential), rg, v);
  printPlotWithArrows("Interrobot arrows test_0_90.png", "Interrobot", 0, 90, {r1, r2, r3, r4},
                      std::function(&interrobotCollisionPotential), rg, v);
  printPlotWithArrows("Arrows test_cohesion_0_90.png", "Cohesion", 0, 90, {r1, r2, r3, r4},
                      std::function(&cohesionPotential), rg, v);
}

TEST(getNeighboursTest, ShouldPass) {
  Variables &v = Variables::getInstance();
  v.setParam("neighbourhood_distance", 2.0);

  Robot r1(Vector_t(5.0, 5.0));
  Robot r2(Vector_t(4.0, 4.0));
  Robot r3(Vector_t(1.0, 3.0));

  RobotGraph rg;
  boost::add_vertex(r2, rg);
  boost::add_vertex(r3, rg);
  auto neighbours = getNeighbourRobots(r1, rg, v);
  EXPECT_EQ(findVertexInGraph(r2, neighbours).second, true);
  EXPECT_EQ(findVertexInGraph(r3, neighbours).second, false);
}

TEST(getNeighboursPreservedTest, ShouldPass) {
  Variables &v = Variables::getInstance();
  v.setParam("neighbourhood_distance", 2.0);

  Robot r1(Vector_t(3.0, 5.0));
  Robot r2(Vector_t(2.0, 4.0));
  Robot r3(Vector_t(4.0, 4.0));

  RobotGraph rg;
  boost::add_vertex(r2, rg);
  boost::add_vertex(r3, rg);
  auto neighbours_preserved = getNeighbourPreservedRobots(r1, rg, v);
  EXPECT_EQ(findVertexInGraph(r2, neighbours_preserved).second, true);
  EXPECT_EQ(findVertexInGraph(r3, neighbours_preserved).second, true);
}

TEST(potentialGradient, ShouldPass) {
  Variables &v = Variables::getInstance();
  v.setParam("robots_avoidance_distance", 2.0);
  v.setParam("obstacles_avoidance_distance", 1.5);
  v.setParam("los_clearance_distance", 0.2);
  v.setParam("los_clearance_care_distance", 0.4);
  v.setParam("neighbourhood_distance", 5.0);
  v.setParam("edge_deletion_distance", -1.0);
  v.setParam("obstacle_care_distance", 3.0);
  v.setParam("desired_distance", 3.5);
  v.setParam("k1", 10);
  v.setParam("k2", 10);
  v.setParam("c1", 0.0);
  v.setParam("c2", 0.0);
  v.setParam("c3", 0.0);
  v.setParam("c4", 1000.0);

  Robot r1(Vector_t(5.0, 5.0));
  Robot r2(Vector_t(10.0, 5.0));
  Robot r3(Vector_t(7.5, 7.5));
  RobotGraph rg;
  boost::add_vertex(r1, rg);
  boost::add_vertex(r2, rg);
  boost::add_vertex(r3, rg);

  Obstacle o1(Vector_t(7.5, 6.0));
  Obstacle o2(Vector_t(7.5, 9.5));
  Obstacle o3(Vector_t(15.0, 5.0));
  Obstacle o4(Vector_t(6.0, 6.0));
  ObstacleGraph og;
  boost::add_vertex(o1, og);
//  boost::add_vertex(o2, og);
//  boost::add_vertex(o3,og);
//  boost::add_vertex(o4, og);

  r1.setSpeedDirection(gradientPotential(r1.getPosition(), overallPotential, v, rg, og));
  r2.setSpeedDirection(gradientPotential(r2.getPosition(), overallPotential, v, rg, og));
  r3.setSpeedDirection(gradientPotential(r3.getPosition(), overallPotential, v, rg, og));

  printPlotWithArrows("Overall potential gradient.png", "Overall potentials and gradient", 30, 60, {r1, r2, r3},
                      std::function(&overallPotential),
                      rg, og, v);
  printPlotWithArrows("Overall potential gradient_0_90.png", "Overall potentials and gradient", 0, 90, {r1, r2, r3},
                      std::function(&overallPotential), rg, og, v);
}

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
