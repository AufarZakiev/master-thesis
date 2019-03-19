#include <gtest/gtest.h>
#include "../include/headers/speed_constraints.h"
#include "../include/headers/field_functions.h"

TEST(maximumDistanceConstraint, ShouldPass)
{
  Variables v = Variables();
  v.setParam("neighbourhood_distance", 3);
  Robot r0(Position_t(5, 5)), r1(Position_t(3, 3)), r2(Position_t(3, 4));
  r0.setSpeedDirection(Vector_t(1, 1));
  RobotGraph rg;
  boost::add_vertex(r1, rg);
  boost::add_vertex(r2, rg);
  auto value = maximumDistanceConstraint(r0, rg, v);
  EXPECT_LT(value, 0.09);
  EXPECT_GT(value, 0.085);
}

TEST(maximumDistanceConstraint2, ShouldPass)
{
  Variables v = Variables();
  double EQUALITY_CASE;
  v.getParam("equality_case", EQUALITY_CASE);
  Robot r0(Position_t(5, 5)), r1(Position_t(7.5, 0.5)), r2(Position_t(3, 4)), r3(Position_t(8, 6)),
      r4(Position_t(7, 4));
  r0.setSpeedDirection(Vector_t(1, 1));
  RobotGraph rg;
  boost::add_vertex(r1, rg);
  boost::add_vertex(r2, rg);
  boost::add_vertex(r3, rg);
  EXPECT_NEAR(sqrt(8), maximumDistanceConstraint2(r0, rg), EQUALITY_CASE);
  boost::add_vertex(r4, rg);
  EXPECT_NEAR(sqrt(2) / 2, maximumDistanceConstraint2(r0, rg), EQUALITY_CASE);
}

TEST(interrobotAvoidanceConstraint, ShouldPass)
{
  Variables v = Variables();
  double EQUALITY_CASE;
  v.getParam("equality_case", EQUALITY_CASE);
  v.setParam("robots_avoidance_distance", 3);
  Robot r0(Position_t(5, 5)), r1(Position_t(10, 5)), r2(Position_t(8, 7));
  r0.setSpeedDirection(Vector_t(1, 1));
  RobotGraph rg;
  boost::add_vertex(r1, rg);
  boost::add_vertex(r2, rg);
  EXPECT_NEAR(0.5 * (sqrt(13) - 3.0), interrobotAvoidanceConstraint(r0, rg, v), EQUALITY_CASE);
}

TEST(obstacleAvoidanceConstraint, ShouldPass)
{
  Variables v = Variables();
  v.setParam("obstacles_avoidance_distance", 1);
  Robot r0(Position_t(5, 5));
  r0.setSpeedDirection(Vector_t(1, 1));

  Obstacle o1(Position_t(6, 10)), o2(Position_t(10, 7));
  o1.setRadius(2);
  o2.setRadius(2);

  ObstacleGraph og;
  boost::add_vertex(o1, og);
  boost::add_vertex(o2, og);

  EXPECT_LT(obstacleAvoidanceConstraint(r0, og, v, 0), 2.9);
  EXPECT_GT(obstacleAvoidanceConstraint(r0, og, v, 0), 2.8);
}

TEST(LOSPreservationConstraint, ShouldPass)
{
  Variables v = Variables();
  v.setParam("los_clearance_distance", 0.5);
  double EQUALITY_CASE;
  v.getParam("equality_case", EQUALITY_CASE);

  Robot r1(Position_t(2, 2)), r2(Position_t(4, 4)), r3(Position_t(7, 4));
  r2.setSpeedDirection(Vector_t(-1, 1));

  RobotGraph rg;
  boost::add_vertex(r1, rg);
  boost::add_vertex(r3, rg);

  Obstacle o1(Position_t(2, 3)), o2(Position_t(6, 5));
  o1.setRadius(0);
  o2.setRadius(0);

  ObstacleGraph og;
  boost::add_vertex(o1, og);
  boost::add_vertex(o2, og);

  EXPECT_NEAR(LOSPreservationConstraint(r2, og, v, rg), sqrt(2.0) / 2 - 1.0 / 2, EQUALITY_CASE);
  r2.setSpeedDirection(Vector_t(1, -1));
  EXPECT_NEAR(LOSPreservationConstraint(r2, og, v, rg), std::numeric_limits<double>::max(), EQUALITY_CASE);
}

TEST(getConstrainedSpeedTest, ShouldPass)
{
  Variables v = Variables();
  v.setParam("robots_avoidance_distance", 1.5);
  v.setParam("obstacles_avoidance_distance", 1.0);
  v.setParam("los_clearance_distance", 0.2);
  v.setParam("los_clearance_care_distance", 0.4);
  v.setParam("neighbourhood_distance", 5.0);
  v.setParam("edge_deletion_distance", 0.1);
  v.setParam("obstacle_care_distance", 3.0);
  v.setParam("desired_distance", 3.5);
  v.setParam("sensing_distance", 10.0);
  v.setParam("robot_max_speed", 0.1);
  v.setParam("k1", 10);
  v.setParam("k2", 10);
  v.setParam("c1", 0.0);
  v.setParam("c2", 0.0);
  v.setParam("c3", 0.0);
  v.setParam("c4", 10.0);

  ValidatedVariables vv(v);

  Robot r1(Vector_t(5.0, 3.0));
  Robot r2(Vector_t(10.0, 3.0));
  Robot r3(Vector_t(7.5, 14.0));
  RobotGraph rg;
  boost::add_vertex(r1, rg);
  boost::add_vertex(r2, rg);
  boost::add_vertex(r3, rg);

  ObstacleGraph og;

  ValidatedGraphs vg(rg, og, v);

  r1.setSpeedDirection(getConstrainedDirectedSpeed(r1, vg, vv));

  r2.setSpeedDirection(getConstrainedDirectedSpeed(r2, vg, vv));

  r3.setSpeedDirection(getConstrainedDirectedSpeed(r3, vg, vv));

  printPlotWithArrows("ConstrainedSpeedTest.png", "ConstrainedSpeedTest", 30, 60, 1, { r1, r2, r3 },
                      std::function(&overallPotential), rg, og, v);
  printPlotWithArrows("ConstrainedSpeedTest_0_90.png", "ConstrainedSpeedTest", 0, 90, 1, { r1, r2, r3 },
                      std::function(&overallPotential), rg, og, v);
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
