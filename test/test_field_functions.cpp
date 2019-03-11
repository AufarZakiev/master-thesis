#include <gtest/gtest.h>
#include "../include/headers/classes.h"
#include "../include/headers/field_functions.h"

TEST(CohesionPotentialTest_4_robots, ShouldPass)
{
  Variables& v = Variables::getInstance();
  v.setParam("neighbourhood_distance", 1.0f);

  Robot r1(Vector_t(5, 5));
  Robot r2(Vector_t(10, 10));
  Robot r3(Vector_t(10, 5));
  Robot r4(Vector_t(5, 10));
  RobotGraph rg;
  boost::add_vertex(r1, rg);
  boost::add_vertex(r2, rg);
  boost::add_vertex(r3, rg);
  boost::add_vertex(r4, rg);

  printPlot("Cohesion_field_4_robots_uniform.png", "Cohesion field 4 robots", 0, 90, std::function(&cohesionPotential),
            rg, v);
}

TEST(CohesionPotentialTest_2_robots_offset, ShouldPass)
{
  Variables& v = Variables::getInstance();
  v.setParam("neighbourhood_distance", 1.0f);

  Robot r1(Vector_t(0, 0));
  Robot r2(Vector_t(10, 10));
  RobotGraph rg;
  boost::add_vertex(r1, rg);
  boost::add_vertex(r2, rg);

  printPlot("Cohesion_field_2_robots_cohesive_offset.png", "Cohesion field 2 robots", 0, 90,
            std::function(&cohesionPotential), rg, v);
}

TEST(CohesionPotentialTest_2_robots_cohesive, ShouldPass)
{
  Variables& v = Variables::getInstance();
  v.setParam("neighbourhood_distance", 20.0f);

  Robot r1(Vector_t(0, 0));
  Robot r2(Vector_t(10, 10));
  RobotGraph rg;
  boost::add_vertex(r1, rg);
  boost::add_vertex(r2, rg);

  printPlot("Cohesion_field_2_robots_cohesiv_neighbourhood_max.png", "Cohesion field 2 robots cohesive", 60, 30,
            std::function(&cohesionPotential), rg, v);
}

TEST(InterrobotPotentialTest_2_robots, ShouldPass)
{
  Variables& v = Variables::getInstance();
  v.setParam("robots_avoidance_distance", 1.0);
  v.setParam("desired_distance", 3.0);
  v.setParam("neighbourhood_distance", 4.0);
  v.setParam("k1", 10);
  v.setParam("k2", 10);

  Robot r1(Vector_t(5.5, 5.5));
  Robot r2(Vector_t(9.5, 9.5));
  RobotGraph rg;
  boost::add_vertex(r1, rg);
  boost::add_vertex(r2, rg);

  printPlot("Interrobot_field_2_robots.png", "Interrobot field 2 robots", 60, 30,
            std::function(&interrobotCollisionPotential), rg, v);
}

TEST(ObstaclePotentialTest_one_obstacle, ShouldPass)
{
  Variables& v = Variables::getInstance();
  v.setParam("obstacle_care_distance", 3.0);
  v.setParam("obstacles_avoidance_distance", 1.5);
  v.setParam("small_positive_constant", 0.2);

  Obstacle o1(Vector_t(6.0, 6.0));
  ObstacleGraph og;
  boost::add_vertex(o1, og);

  printPlot("Obstacle_collision_fields.png", "Obstacle collision field", 60, 30,
            std::function(&obstacleCollisionPotential), og, v);
}

TEST(ObstaclePotentialTest_two_obstacle, ShouldPass)
{
  Variables& v = Variables::getInstance();
  v.setParam("obstacle_care_distance", 3.0);
  v.setParam("obstacles_avoidance_distance", 1.5);
  v.setParam("small_positive_constant", 0.2);

  Obstacle o1(Vector_t(10.0, 10.0));
  Obstacle o2(Vector_t(5.0, 5.0));
  ObstacleGraph og;
  boost::add_vertex(o1, og);
  boost::add_vertex(o2, og);

  printPlot("Two_obstacle_collision_fields.png", "Two obstacle collision field", 60, 30,
            std::function(&obstacleCollisionPotential), og, v);
}

TEST(ObstaclePotentialTest_two_obstacle_interfere, ShouldPass)
{
  Variables& v = Variables::getInstance();
  v.setParam("obstacle_care_distance", 3.0);
  v.setParam("obstacles_avoidance_distance", 1.5);
  v.setParam("small_positive_constant", 0.2);

  Obstacle o1(Vector_t(6.0, 6.0));
  Obstacle o2(Vector_t(5.0, 5.0));
  ObstacleGraph og;
  boost::add_vertex(o1, og);
  boost::add_vertex(o2, og);

  printPlot("Two_obstacle_collision_fields_interfere.png", "Two obstacle collision field interfere", 60, 30,
            std::function(&obstacleCollisionPotential), og, v);
  printPlot("Two_obstacle_collision_fields_interfere_0_90.png", "Two obstacle collision field interfere", 0, 90,
            std::function(&obstacleCollisionPotential), og, v);
}

TEST(LOSPotentialTest_obstacle, ShouldPass)
{
  Variables& v = Variables::getInstance();
  v.setParam("los_clearance_care_distance", 0.4);
  v.setParam("los_clearance_distance", 0.2);
  v.setParam("small_positive_constant", 0.2);
  v.setParam("obstacle_care_distance", 3.0);
  v.setParam("obstacles_avoidance_distance", 1.5);

  Obstacle o1(Vector_t(6.0, 6.0));
  ObstacleGraph og;
  boost::add_vertex(o1, og);

  Robot r1(Vector_t(5.0, 5.0));
  Robot r2(Vector_t(10.0, 10.0));
  RobotGraph rg;
  auto r1_desc = boost::add_vertex(r1, rg);

  printPlot("LOS fields.png", "LOS preservation field", 45, 25, std::function(&LOSPreservePotential), rg, og, v);
  printPlot("LOS fields_0_90.png", "LOS preservation field", 0, 90, std::function(&LOSPreservePotential), rg, og, v);
  boost::remove_vertex(r1_desc, rg);
  boost::add_vertex(r2, rg);
  printPlot("LOS fields 2 robots.png", "LOS preservation field", 45, 25, std::function(&LOSPreservePotential), rg, og,
            v);
}

TEST(overallPotentialTest, ShouldPass)
{
  Variables& v = Variables::getInstance();
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
  v.setParam("c1", 1.0);
  v.setParam("c2", 1.0);
  v.setParam("c3", 1.0);
  v.setParam("c4", 1.0);

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

  printPlotWithArrows("Overall potentials.png", "Overall potentials", 30, 60, { r1,r2,r3 }, std::function(&overallPotential),
                      rg, og, v);
  printPlotWithArrows("Overall potentials_0_90.png", "Overall potentials", 0, 90, { r1,r2,r3 },
                      std::function(&overallPotential), rg, og, v);
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
