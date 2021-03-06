#include <gtest/gtest.h>
#include "headers/classes.h"
#include "headers/field_functions.h"

TEST(CohesionPotentialTest_4_robots, ShouldPass)
{
  Variables v = Variables();
  v.setParam("neighbourhood_distance", 1.0f);

  Robot r1(Position_t(5, 5), 0);
  Robot r2(Position_t(10, 10), 1);
  Robot r3(Position_t(10, 5), 2);
  Robot r4(Position_t(5, 10), 3);
  RobotGraph rg;
  boost::add_vertex(r1, rg);
  boost::add_vertex(r2, rg);
  boost::add_vertex(r3, rg);
  boost::add_vertex(r4, rg);

  ObstacleGraph og;

  printPlot("Cohesion_field_4_robots_uniform.png", "Cohesion field 4 robots", 0, 90, std::function(&cohesionPotential),
            rg, og, v);
}

TEST(CohesionPotentialTest_2_robots_offset, ShouldPass)
{
  Variables v = Variables();
  v.setParam("neighbourhood_distance", 1.0f);

  Robot r1(Position_t(0, 0), 0);
  Robot r2(Position_t(10, 10), 1);
  RobotGraph rg;
  boost::add_vertex(r1, rg);
  boost::add_vertex(r2, rg);

  ObstacleGraph og;

  printPlot("Cohesion_field_2_robots_cohesive_offset.png", "Cohesion field 2 robots", 0, 90,
            std::function(&cohesionPotential), rg, og, v);
}

TEST(CohesionPotentialTest_2_robots_cohesive, ShouldPass)
{
  Variables v = Variables();
  v.setParam("neighbourhood_distance", 20.0f);

  Robot r1(Position_t(0, 0), 0);
  Robot r2(Position_t(10, 10), 1);
  RobotGraph rg;
  boost::add_vertex(r1, rg);
  boost::add_vertex(r2, rg);

  ObstacleGraph og;

  printPlot("Cohesion_field_2_robots_cohesiv_neighbourhood_max.png", "Cohesion field 2 robots cohesive", 60, 30,
            std::function(&cohesionPotential), rg, og, v);
}

TEST(InterrobotPotentialTest_2_robots, ShouldPass)
{
  Variables v = Variables();
  v.setParam("robots_avoidance_distance", 1.0);
  v.setParam("desired_distance", 3.0);
  v.setParam("neighbourhood_distance", 4.0);
  v.setParam("k1", 10);
  v.setParam("k2", 10);

  Robot r1(Position_t(5.5, 5.5), 0);
  Robot r2(Position_t(9.5, 9.5), 1);
  RobotGraph rg;
  boost::add_vertex(r1, rg);
  boost::add_vertex(r2, rg);

  ObstacleGraph og;

  //  printPlot("Interrobot_field_2_robots.png", "Interrobot field 2 robots", 60, 30,
  //            std::function(&interrobotCollisionPotential), rg, og, v);
}

TEST(ObstaclePotentialTest_one_obstacle, ShouldPass)
{
  Variables v = Variables();
  v.setParam("obstacle_care_distance", 3.0);
  v.setParam("obstacles_avoidance_distance", 1.5);
  v.setParam("small_positive_constant", 0.2);

  Obstacle o1(Position_t(6.0, 6.0));
  ObstacleGraph og;
  boost::add_vertex(o1, og);

  //  printPlot("Obstacle_collision_fields.png", "Obstacle collision field", 60, 30,
  //            std::function(&obstacleCollisionPotential), og, v);
}

TEST(ObstaclePotentialTest_two_obstacle, ShouldPass)
{
  Variables v = Variables();
  v.setParam("obstacle_care_distance", 3.0);
  v.setParam("obstacles_avoidance_distance", 1.5);
  v.setParam("small_positive_constant", 0.2);

  Obstacle o1(Position_t(10.0, 10.0));
  Obstacle o2(Position_t(5.0, 5.0));
  ObstacleGraph og;
  boost::add_vertex(o1, og);
  boost::add_vertex(o2, og);

  //  printPlot("Two_obstacle_collision_fields.png", "Two obstacle collision field", 60, 30,
  //            std::function(&obstacleCollisionPotential), og, v);
}

TEST(ObstaclePotentialTest_with_radius, ShouldPass)
{
  Variables v = Variables();
  v.setParam("obstacle_care_distance", 3.0);
  v.setParam("obstacles_avoidance_distance", 1.5);
  v.setParam("small_positive_constant", 0.2);

  Obstacle o1(Position_t(10.0, 10.0), 2.0);
  ObstacleGraph og;
  boost::add_vertex(o1, og);

  //  printPlot("Obstacle_with_radius.png", "Obstacle with radius", 60, 30, std::function(&obstacleCollisionPotential),
  //  og,
  //            v);
  //  printPlot("Obstacle_with_radius_0_90.png", "Obstacle with radius", 0, 90,
  //  std::function(&obstacleCollisionPotential),
  //            og, v);
}

TEST(ObstaclePotentialTest_two_obstacle_interfere, ShouldPass)
{
  Variables v = Variables();
  v.setParam("obstacle_care_distance", 3.0);
  v.setParam("obstacles_avoidance_distance", 1.5);
  v.setParam("small_positive_constant", 0.2);

  Obstacle o1(Position_t(6.0, 6.0));
  Obstacle o2(Position_t(5.0, 5.0));
  ObstacleGraph og;
  boost::add_vertex(o1, og);
  boost::add_vertex(o2, og);

  //  printPlot("Two_obstacle_collision_fields_interfere.png", "Two obstacle collision field interfere", 60, 30,
  //            std::function(&obstacleCollisionPotential), og, v);
  //  printPlot("Two_obstacle_collision_fields_interfere_0_90.png", "Two obstacle collision field interfere", 0, 90,
  //            std::function(&obstacleCollisionPotential), og, v);
}

TEST(LOSPotentialTest_obstacle, ShouldPass)
{
  Variables v = Variables();
  v.setParam("los_clearance_care_distance", 6.5);
  v.setParam("los_clearance_distance", 0.5);
  v.setParam("small_positive_constant", 0.2);

  Obstacle o1(Position_t(7.5, 7.5), 1.5);
  ObstacleGraph og;
  boost::add_vertex(o1, og);

  Robot r1(Position_t(9.6, 6.0), 0);
  Robot r2(Position_t(9.6, 9.0), 1);
  RobotGraph rg;
  //boost::add_vertex(r1, rg);
  boost::add_vertex(r2, rg);

  printPlot("LOS fields.png", "LOS preservation field", 45, 115, std::function(&LOSPreservePotential), rg, og, v);
  printPlot("LOS fields_0_90.png", "LOS preservation field", 0, 90, std::function(&LOSPreservePotential), rg, og, v);
}

TEST(overallPotentialTest, ShouldPass)
{
  Variables v = Variables();
  v.setParam("los_clearance_care_distance", 0.4);
  v.setParam("los_clearance_distance", 0.2);
  v.setParam("small_positive_constant", 0.2);
  v.setParam("robots_avoidance_distance", 2.0);
  v.setParam("desired_distance", 3.5);
  v.setParam("neighbourhood_distance", 15.0);
  v.setParam("obstacle_care_distance", 3.0);
  v.setParam("obstacles_avoidance_distance", 1.5);
  v.setParam("edge_deletion_distance", 1.3);
  v.setParam("k1", 10);
  v.setParam("k2", 10);
  v.setParam("c1", 0.0);
  v.setParam("c2", 0.0);
  v.setParam("c3", 1.0);
  v.setParam("c4", 0.0);

  Robot r1(Position_t(5.0, 5.0), 0);
  Robot r2(Position_t(10.0, 5.0), 1);
  Robot r3(Position_t(7.5, 7.5), 2);
  RobotGraph rg;
  auto r1_desc = boost::add_vertex(r1, rg);
  auto r2_desc = boost::add_vertex(r2, rg);

  Obstacle o1(Position_t(7.5, 6.0), 1.0);
  Obstacle o2(Position_t(7.5, 9.5), 1.0);
  Obstacle o3(Position_t(15.0, 5.0), 1.0);
  Obstacle o4(Position_t(6.0, 6.0), 1.0);
  ObstacleGraph og;
  boost::add_vertex(o1, og);

  // rg[r1_desc].setSpeedDirection(gradientPotentialOnly(r1, rg, og, v));
  // rg[r2_desc].setSpeedDirection(gradientPotentialOnly(r2, rg, og, v));
  //  boost::add_vertex(o2, og);
  //  boost::add_vertex(o3,og);
  //  boost::add_vertex(o4, og);

  printPlotWithArrows("Overall potentials.png", "Overall potentials", 30, 60, 1, rg, std::function(&overallPotential),
                      rg, og, v);
  printPlotWithArrows("Overall potentials_0_90.png", "Overall potentials", 0, 90, 1, rg,
                      std::function(&overallPotential), rg, og, v);
}

TEST(potentialGradient, ShouldPass)
{
  Variables v = Variables();
  v.setParam("robots_avoidance_distance", 2.0);
  v.setParam("obstacles_avoidance_distance", 1.0);
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
  v.setParam("c4", 10.0);

  Robot r1(Position_t(5.0, 4.0), 0);
  Robot r2(Position_t(10.0, 4.0), 1);
  Robot r3(Position_t(7.5, 10.0), 2);
  RobotGraph rg;
  auto r1_desc = boost::add_vertex(r1, rg);
  auto r2_desc = boost::add_vertex(r2, rg);
  auto r3_desc = boost::add_vertex(r3, rg);

  ObstacleGraph og;

  //  r1.setSpeedDirection(gradientPotential(r1.getPosition(), overallPotential, v, rg, og));
  //
  //  r2.setSpeedDirection(gradientPotential(r2.getPosition(), overallPotential, v, rg, og));
  //
  //  r3.setSpeedDirection(gradientPotential(r3.getPosition(), overallPotential, v, rg, og));
  // rg[r1_desc].setSpeedDirection(gradientPotential(r1.getPosition(), overallPotential, v, rg, og));
  // rg[r2_desc].setSpeedDirection(gradientPotential(r2.getPosition(), overallPotential, v, rg, og));
  // rg[r3_desc].setSpeedDirection(gradientPotential(r3.getPosition(), overallPotential, v, rg, og));

  printPlotWithArrows("Overall potential gradient.png", "Overall potentials and gradient", 30, 60, 0.1, rg,
                      std::function(&overallPotential), rg, og, v);
  printPlotWithArrows("Overall potential gradient_0_90.png", "Overall potentials and gradient", 0, 90, 0.1, rg,
                      std::function(&overallPotential), rg, og, v);
}

TEST(potentialGradient2, ShouldPass)
{
  Variables v = Variables();
  v.setParam("robots_avoidance_distance", 2.0);
  v.setParam("obstacles_avoidance_distance", 1.0);
  v.setParam("los_clearance_distance", 0.2);
  v.setParam("los_clearance_care_distance", 0.4);
  v.setParam("neighbourhood_distance", 15.0);
  v.setParam("edge_deletion_distance", -1.0);
  v.setParam("obstacle_care_distance", 3.0);
  v.setParam("desired_distance", 3.5);
  v.setParam("k1", 10);
  v.setParam("k2", 10);
  v.setParam("c1", 1.0);
  v.setParam("c2", 1.0);
  v.setParam("c3", 1.0);
  v.setParam("c4", 1.0);

  Robot r1(Position_t(5.0, 5.0), 0);
  Robot r2(Position_t(10.0, 5.0), 1);
  Robot r3(Position_t(7.5, 9.5), 2);
  RobotGraph rg;
  auto r1_desc = boost::add_vertex(r1, rg);
  auto r2_desc = boost::add_vertex(r2, rg);
  auto r3_desc = boost::add_vertex(r3, rg);

  Obstacle o1(Position_t(7.5, 6.0));
  Obstacle o2(Position_t(7.5, 9.5));
  Obstacle o3(Position_t(15.0, 5.0));
  Obstacle o4(Position_t(6.0, 6.0));
  ObstacleGraph og;
  boost::add_vertex(o1, og);
  //  boost::add_vertex(o2, og);
  //  boost::add_vertex(o3,og);
  //  boost::add_vertex(o4, og);

  //  r1.setSpeedDirection(gradientPotential(r1.getPosition(), overallPotential, v, rg, og));
  //
  //  r2.setSpeedDirection(gradientPotential(r2.getPosition(), overallPotential, v, rg, og));
  //
  //  r3.setSpeedDirection(gradientPotential(r3.getPosition(), overallPotential, v, rg, og));
  // rg[r1_desc].setSpeedDirection(gradientPotential(r1.getPosition(), overallPotential, v, rg, og));
  // rg[r2_desc].setSpeedDirection(gradientPotential(r2.getPosition(), overallPotential, v, rg, og));
  // rg[r3_desc].setSpeedDirection(gradientPotential(r3.getPosition(), overallPotential, v, rg, og));

  printPlotWithArrows("Overall potential gradient2.png", "Overall potentials and gradient", 30, 60, 15, rg,
                      std::function(&overallPotential), rg, og, v);
  printPlotWithArrows("Overall potential gradient2_0_90.png", "Overall potentials and gradient", 0, 90, 15, rg,
                      std::function(&overallPotential), rg, og, v);
}

TEST(potentialGradient3, ShouldPass)
{
  Variables v = Variables();
  v.setParam("robots_avoidance_distance", 2.0);
  v.setParam("obstacles_avoidance_distance", 1.0);
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
  v.setParam("c4", 10.0);

  Robot r1(Position_t(5.0, 3.0), 0);
  Robot r2(Position_t(10.0, 3.0), 1);
  Robot r3(Position_t(7.5, 14.0), 2);
  RobotGraph rg;
  auto r1_desc = boost::add_vertex(r1, rg);
  auto r2_desc = boost::add_vertex(r2, rg);
  auto r3_desc = boost::add_vertex(r3, rg);

  ObstacleGraph og;

  //  r1.setSpeedDirection(gradientPotential(r1.getPosition(), overallPotential, v, rg, og));
  //  r2.setSpeedDirection(gradientPotential(r2.getPosition(), overallPotential, v, rg, og));
  //  r3.setSpeedDirection(gradientPotential(r3.getPosition(), overallPotential, v, rg, og));
  //  rg[r1_desc].setSpeedDirection(gradientPotential(r1.getPosition(), overallPotential, v, rg, og));
  //  rg[r2_desc].setSpeedDirection(gradientPotential(r2.getPosition(), overallPotential, v, rg, og));
  //  rg[r3_desc].setSpeedDirection(gradientPotential(r3.getPosition(), overallPotential, v, rg, og));

  printPlotWithArrows("Overall potential gradient3.png", "Overall potentials and gradient 3", 30, 60, 0.1, rg,
                      std::function(&overallPotential), rg, og, v);
  printPlotWithArrows("Overall potential gradient3_0_90.png", "Overall potentials and gradient 3", 0, 90, 0.1, rg,
                      std::function(&overallPotential), rg, og, v);
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
