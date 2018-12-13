#include <gtest/gtest.h>
#include "../include/headers/speed_constraints.h"

TEST(maximumDistanceConstraint, ShouldPass)
{
  Variables& v = Variables::getInstance();
  v.setParam("neighbourhood_distance", 3);
  Robot r0, r1, r2;
  r0.setPosition(Position_t(5, 5));
  r1.setPosition(Position_t(3, 3));
  r2.setPosition(Position_t(3, 4));
  r1.setSpeedDirection(Vector_t(1, 1));
  RobotGraph rg;
  boost::add_vertex(r1, rg);
  boost::add_vertex(r2, rg);
  auto value = maximumDistanceConstraint(r0, rg, v);
  EXPECT_LT(value, 0.086);
  EXPECT_GT(value, 0.085);
}

TEST(maximumDistanceConstraint2, ShouldPass)
{
  Variables& v = Variables::getInstance();
  double EQUALITY_CASE;
  v.getParam("equality_case", EQUALITY_CASE);
  Robot r0, r1, r2, r3, r4;
  r0.setPosition(Vector_t(5, 5));
  r0.setSpeedDirection(Vector_t(1, 1));
  r1.setPosition(Position_t(7.5, 0.5));
  r2.setPosition(Position_t(3, 4));
  r3.setPosition(Position_t(8, 6));
  r4.setPosition(Position_t(7, 4));
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
  Variables& v = Variables::getInstance();
  double EQUALITY_CASE;
  v.getParam("equality_case", EQUALITY_CASE);
  v.setParam("robots_avoidance_distance", 3);
  Robot r0, r1, r2;
  r0.setPosition(Position_t(5, 5));
  r0.setSpeedDirection(Vector_t(1, 1));
  r1.setPosition(Position_t(10, 5));
  r2.setPosition(Position_t(8, 7));
  RobotGraph rg;
  boost::add_vertex(r1, rg);
  boost::add_vertex(r2, rg);
  EXPECT_NEAR(0.5 * (sqrt(13) - 3.0), interrobotAvoidanceConstraint(r0, rg, v), EQUALITY_CASE);
}

TEST(obstacleAvoidanceConstraint, ShouldPass)
{
  Variables& v = Variables::getInstance();
  v.setParam("obstacles_avoidance_distance", 1);
  Robot r0;
  r0.setPosition(Position_t(5, 5));
  r0.setSpeedDirection(Vector_t(1, 1));

  Obstacle o1, o2;
  o1.setPosition(Position_t(6, 10));
  o2.setPosition(Position_t(10, 7));
  o1.setRadius(2);
  o2.setRadius(2);

  ObstacleGraph og;
  boost::add_vertex(o1, og);
  boost::add_vertex(o2, og);

  EXPECT_LT(obstacleAvoidanceConstraint(r0, og, v, 0), 2.9);
  EXPECT_GT(obstacleAvoidanceConstraint(r0, og, v, 0), 2.8);
}

TEST(LOSUnitPreservationConstraint, ShouldPass)
{
  Variables& v = Variables::getInstance();
  v.setParam("los_clearance_distance", 0.5);
  double EQUALITY_CASE;
  v.getParam("equality_case", EQUALITY_CASE);

  Robot r1, r2, r3;
  r1.setPosition(Position_t(2, 2));
  r2.setPosition(Position_t(4, 4));
  r2.setSpeedDirection(Vector_t(-1, 1));
  r3.setPosition(Position_t(7, 4));

  RobotGraph rg;
  boost::add_vertex(r1, rg);
  boost::add_vertex(r3, rg);

  Obstacle o1, o2;
  o1.setPosition(Position_t(2, 3));
  o2.setPosition(Position_t(6, 5));
  o1.setRadius(0);
  o2.setRadius(0);

  ObstacleGraph og;
  auto o1_desc = boost::add_vertex(o1, og);

  EXPECT_NEAR(LOSUnitPreservationConstraint(r2, r1, og, v, rg), sqrt(2.0) / 2 - 1.0 / 2, EQUALITY_CASE);

  boost::remove_vertex(o1_desc, og);
  boost::add_vertex(o2, og);

  //EXPECT_NEAR(LOSUnitPreservationConstraint(r2, r3, og, v, rg), sqrt(2.0) / 2 - 1.0 / 2, EQUALITY_CASE); TODO: wtf with shit
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
