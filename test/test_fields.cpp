#include <gtest/gtest.h>
#include "../include/headers/DakaiAlgo.h"

TEST(CohesionPotentialTest_4_robots, ShouldPass)
{
  Variables& v = Variables::getInstance();
  v.setParam("neighbourhood_distance", 1.0f);

  Vector_t v1, v2, v3, v4;
  v1 << 5, 5;
  v2 << 10, 10;
  v3 << 10, 5;
  v4 << 5, 10;
  Robot r1(v1);
  Robot r2(v2);
  Robot r3(v3);
  Robot r4(v4);
  RigidGraph rg;
  boost::add_vertex(r1, rg);
  boost::add_vertex(r2, rg);
  boost::add_vertex(r3, rg);
  boost::add_vertex(r4, rg);

  std::vector<std::vector<std::tuple<double, double, double>>> frame(60);
  for (int i = 0; i < 60; i++)
  {
    frame[i].resize(60);
    for (int j = 0; j < 60; j++)
    {
      Vector_t temp;
      temp << i / 4.0, j / 4.0;
      RigidObject point(temp);
      frame[i][j] = std::make_tuple(temp(0, 0), temp(1, 0), cohesionPotential(point, rg, v));
    }
  }

  printPlot(frame, "Cohesion_field_4_robots_uniform.png", "Cohesion field 4 robots", 0, 90);
}

TEST(CohesionPotentialTest_2_robots_offset, ShouldPass)
{
  Variables& v = Variables::getInstance();
  v.setParam("neighbourhood_distance", 1.0f);

  Vector_t v1, v2;
  v1 << 0, 0;
  v2 << 10, 10;
  Robot r1(v1);
  Robot r2(v2);
  RigidGraph rg;
  boost::add_vertex(r1, rg);
  boost::add_vertex(r2, rg);

  std::vector<std::vector<std::tuple<double, double, double>>> frame(60);
  for (int i = 0; i < 60; i++)
  {
    frame[i].resize(60);
    for (int j = 0; j < 60; j++)
    {
      Vector_t temp;
      temp << i / 4.0, j / 4.0;
      RigidObject point(temp);
      frame[i][j] = std::make_tuple(temp(0, 0), temp(1, 0), cohesionPotential(point, rg, v));
    }
  }

  printPlot(frame, "Cohesion_field_2_robots_cohesive_offset.png", "Cohesion field 2 robots", 0, 90);
}

TEST(CohesionPotentialTest_2_robots_cohesive, ShouldPass)
{
  Variables& v = Variables::getInstance();
  v.setParam("neighbourhood_distance", 20.0f);

  Vector_t v1, v2;
  v1 << 0, 0;
  v2 << 10, 10;
  Robot r1(v1);
  Robot r2(v2);
  RigidGraph rg;
  boost::add_vertex(r1, rg);
  boost::add_vertex(r2, rg);

  std::vector<std::vector<std::tuple<double, double, double>>> frame(60);
  for (int i = 0; i < 60; i++)
  {
    frame[i].resize(60);
    for (int j = 0; j < 60; j++)
    {
      Vector_t temp;
      temp << i / 4.0, j / 4.0;
      RigidObject point(temp);
      frame[i][j] = std::make_tuple(temp(0, 0), temp(1, 0), cohesionPotential(point, rg, v));
    }
  }

  printPlot(frame, "Cohesion_field_2_robots_cohesiv_neighbourhood_max.png", "Cohesion field 2 robots cohesive", 60, 30);
}

TEST(InterrobotPotentialTest_2_robots, ShouldPass)
{
  Variables& v = Variables::getInstance();
  v.setParam("robots_avoidance_distance", 1.0);
  v.setParam("desired_distance", 3.0);
  v.setParam("neighbourhood_distance", 4.0);
  v.setParam("k1", 10);
  v.setParam("k2", 10);

  Vector_t v1, v2;
  v1 << 5.5, 5.5;
  v2 << 9.5, 9.5;
  Robot r1(v1);
  Robot r2(v2);
  RigidGraph rg;
  boost::add_vertex(r1, rg);
  boost::add_vertex(r2, rg);

  std::vector<std::vector<std::tuple<double, double, double>>> frame(60);
  for (int i = 0; i < 60; i++)
  {
    frame[i].resize(60);
    for (int j = 0; j < 60; j++)
    {
      Vector_t temp;
      temp << i / 4.0, j / 4.0;
      RigidObject point(temp);
      frame[i][j] = std::make_tuple(temp(0, 0), temp(1, 0), interrobotCollisionPotential(point, rg, v));
    }
  }

  printPlot(frame, "Interrobot_field_2_robots.png", "Interrobot field 2 robots", 60, 30);
}

TEST(ObstaclePotentialTest_one_obstacle, ShouldPass)
{
  Variables& v = Variables::getInstance();
  v.setParam("obstacle_care_distance", 3.0);
  v.setParam("obstacles_avoidance_distance", 1.5);
  v.setParam("small_positive_constant", 0.2);

  Vector_t v1;
  v1 << 6.0, 6.0;
  Obstacle o1(v1);
  ObstacleGraph og;
  boost::add_vertex(o1, og);

  std::vector<std::vector<std::tuple<double, double, double>>> frame(60);
  for (int i = 0; i < 60; i++)
  {
    frame[i].resize(60);
    for (int j = 0; j < 60; j++)
    {
      Vector_t temp;
      temp << i / 4.0, j / 4.0;
      RigidObject point(temp);
      frame[i][j] = std::make_tuple(temp(0, 0), temp(1, 0), obstacleCollisionPotential(point, og, v));
    }
  }

  printPlot(frame, "Obstacle_collision_fields.png", "Obstacle collision field", 60, 30);
}

TEST(ObstaclePotentialTest_two_obstacle, ShouldPass)
{
  Variables& v = Variables::getInstance();
  v.setParam("obstacle_care_distance", 3.0);
  v.setParam("obstacles_avoidance_distance", 1.5);
  v.setParam("small_positive_constant", 0.2);

  Vector_t v1, v2;
  v1 << 10.0, 10.0;
  v2 << 5.0, 5.0;
  Obstacle o1(v1);
  Obstacle o2(v2);
  ObstacleGraph og;
  boost::add_vertex(o1, og);
  boost::add_vertex(o2, og);

  std::vector<std::vector<std::tuple<double, double, double>>> frame(60);
  for (int i = 0; i < 60; i++)
  {
    frame[i].resize(60);
    for (int j = 0; j < 60; j++)
    {
      Vector_t temp;
      temp << i / 4.0, j / 4.0;
      RigidObject point(temp);
      frame[i][j] = std::make_tuple(temp(0, 0), temp(1, 0), obstacleCollisionPotential(point, og, v));
    }
  }

  printPlot(frame, "Two_obstacle_collision_fields.png", "Two obstacle collision field", 60, 30);
}

TEST(ObstaclePotentialTest_two_obstacle_interfere, ShouldPass)
{
  Variables& v = Variables::getInstance();
  v.setParam("obstacle_care_distance", 3.0);
  v.setParam("obstacles_avoidance_distance", 1.5);
  v.setParam("small_positive_constant", 0.2);

  Vector_t v1, v2;
  v1 << 6.0, 6.0;
  v2 << 5.0, 5.0;
  Obstacle o1(v1);
  Obstacle o2(v2);
  ObstacleGraph og;
  boost::add_vertex(o1, og);
  boost::add_vertex(o2, og);

  std::vector<std::vector<std::tuple<double, double, double>>> frame(60);
  for (int i = 0; i < 60; i++)
  {
    frame[i].resize(60);
    for (int j = 0; j < 60; j++)
    {
      Vector_t temp;
      temp << i / 4.0, j / 4.0;
      RigidObject point(temp);
      frame[i][j] = std::make_tuple(temp(0, 0), temp(1, 0), obstacleCollisionPotential(point, og, v));
    }
  }

  printPlot(frame, "Two_obstacle_collision_fields_interfere.png", "Two obstacle collision field interfere", 60, 30);
  printPlot(frame, "Two_obstacle_collision_fields_interfere_0_90.png", "Two obstacle collision field interfere", 0, 90);
}

TEST(LOSPotentialTest_obstacle, ShouldPass)
{
  Variables& v = Variables::getInstance();
  v.setParam("los_clearance_care_distance", 0.4);
  v.setParam("los_clearance_distance", 0.2);
  v.setParam("small_positive_constant", 0.2);

  Vector_t v1, v2;
  v1 << 6.0, 6.0;
  v2 << 5.0, 5.0;

  Obstacle o1(v1);
  ObstacleGraph og;
  boost::add_vertex(o1, og);

  Robot r1(v2);
  RobotGraph rg;
  boost::add_vertex(r1, rg);

  std::vector<std::vector<std::tuple<double, double, double>>> frame(120);

  for (int i = 0; i < 120; i++)
  {
    frame[i].resize(120);
    for (int j = 0; j < 120; j++)
    {
      Vector_t temp;
      temp << i / 8.0, j / 8.0;
      Robot point(temp);
      frame[i][j] = std::make_tuple(temp(0, 0), temp(1, 0), LOSPreservePotential(point, rg, og, v));
    }
  }

  printPlot(frame, "LOS fields.png", "LOS preservation field", 45, 25);
  printPlot(frame, "LOS fields_0_90.png", "LOS preservation field", 0, 90);
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
