#include <gtest/gtest.h>
#include "../include/headers/speed_constraints.h"
#include "../include/headers/field_functions.h"
#include <boost/filesystem.hpp>
#include <chrono>

TEST(animatedLeaderTest2, ShouldPass)
{
  Variables v = Variables();
  v.setParam("robots_avoidance_distance", 3.0);
  v.setParam("obstacles_avoidance_distance", 1.7);
  v.setParam("los_clearance_distance", 0.5);
  v.setParam("los_clearance_care_distance", 1.0);
  v.setParam("neighbourhood_distance", 10.0);
  v.setParam("edge_deletion_distance", 2.5);
  v.setParam("obstacle_care_distance", 2.0);
  v.setParam("desired_distance", 7.0);
  v.setParam("sensing_distance", 20.0);
  v.setParam("robot_max_speed", 0.1);
  v.setParam("k1", 10);
  v.setParam("k2", 10);
  v.setParam("c1", 0.5);
  v.setParam("c2", 0.01);
  v.setParam("c3", 0.01);
  v.setParam("c4", 10.0);

  ValidatedVariables vv(v);

  Robot r1(Vector_t(8.0, 8.0), 0);
  Robot r2(Vector_t(0.0, 0.0), 1);
  Robot r3(Vector_t(3.0, 5.0), 2);
  Robot r4(Vector_t(7.0, 4.0), 3);
  Robot r5(Vector_t(1.0, 8.0), 4);
  auto rg = std::make_unique<RobotGraph>();
  auto r1_desc = boost::add_vertex(r1, *rg);
  boost::add_vertex(r2, *rg);
  boost::add_vertex(r3, *rg);
  boost::add_vertex(r4, *rg);
  boost::add_vertex(r5, *rg);

  auto og = std::make_unique<ObstacleGraph>();

  ValidatedGraphs vg(std::move(rg), std::move(og), vv);

  auto leaderV = Vector_t(sqrt(2), sqrt(2));
  boost::filesystem::create_directories("leaderAnimation2");

  for (int i = 0; i < 1200; i++)
  {
    std::cout << "\tIteration " << i << " starts." << std::endl;
    auto start = std::chrono::system_clock::now();

    vg.tick(r1_desc, leaderV, vv);

    auto end = std::chrono::system_clock::now();
    int elapsed_seconds = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();
    std::cout << "\tIteration " << i << " ends in " << elapsed_seconds << std::endl;

    if (i % 100 == 0)
    {
      printPlotWithArrows("leaderAnimation2/leaderAnimation2_0_90_" + std::to_string(i) + ".png",
                          "leaderAnimationTest2", 0, 90, 1, vg.getRobotGraph(), std::function(&overallPotential),
                          vg.getRobotGraph(), vg.getObstacleGraph(), v);
    }
  }
}

TEST(animatedLeaderTest3, ShouldPass)
{
  Variables v = Variables();
  v.setParam("robots_avoidance_distance", 3.0);
  v.setParam("obstacles_avoidance_distance", 1.7);
  v.setParam("los_clearance_distance", 0.5);
  v.setParam("los_clearance_care_distance", 1.0);
  v.setParam("neighbourhood_distance", 10.0);
  v.setParam("edge_deletion_distance", 2.5);
  v.setParam("obstacle_care_distance", 2.0);
  v.setParam("desired_distance", 7.0);
  v.setParam("sensing_distance", 20.0);
  v.setParam("robot_max_speed", 0.1);
  v.setParam("k1", 10);
  v.setParam("k2", 10);
  v.setParam("c1", 0.5);
  v.setParam("c2", 0.01);
  v.setParam("c3", 0.01);
  v.setParam("c4", 10.0);

  ValidatedVariables vv(v);

  Robot r1(Position_t(18.0, 18.0), 0);
  Robot r2(Position_t(16.0, 12.0), 1);
  Robot r3(Position_t(2.0, 2.0), 2);
  Robot r4(Position_t(8.0, 8.0), 3);
  Robot r5(Position_t(5.0, 12.0), 4);
  Robot r6(Position_t(8.0, 0.0), 6);

  auto rg = std::make_unique<RobotGraph>();
  auto r1_desc = boost::add_vertex(r1, *rg);
  boost::add_vertex(r2, *rg);
  boost::add_vertex(r3, *rg);
  boost::add_vertex(r4, *rg);
  boost::add_vertex(r5, *rg);
  boost::add_vertex(r6, *rg);

  auto og = std::make_unique<ObstacleGraph>();

  ValidatedGraphs vg(std::move(rg), std::move(og), vv);

  auto leaderV = Vector_t(sqrt(2), sqrt(2));

  boost::filesystem::create_directories("leaderAnimation3");

  for (int i = 0; i < 1200; i++)
  {
    std::cout << "\tIteration " << i << " starts." << std::endl;
    auto start = std::chrono::system_clock::now();

    vg.tick(r1_desc, leaderV, vv);

    auto end = std::chrono::system_clock::now();
    int elapsed_seconds = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();
    std::cout << "\tIteration " << i << " ends in " << elapsed_seconds << std::endl;

    if (i % 100 == 0)
    {
      printPlotWithArrows("leaderAnimation3/leaderAnimation3_0_90_" + std::to_string(i) + ".png",
                          "leaderAnimationTest3", 0, 90, 1, vg.getRobotGraph(), std::function(&overallPotential),
                          vg.getRobotGraph(), vg.getObstacleGraph(), v);
    }
  }
}

TEST(animatedWideCorridorTest_1, ShouldPass)
{
  Variables v = Variables();
  v.setParam("robots_avoidance_distance", 3.0);
  v.setParam("obstacles_avoidance_distance", 1.7);
  v.setParam("los_clearance_distance", 0.5);
  v.setParam("los_clearance_care_distance", 1.0);
  v.setParam("neighbourhood_distance", 10.0);
  v.setParam("edge_deletion_distance", 2.5);
  v.setParam("obstacle_care_distance", 2.0);
  v.setParam("desired_distance", 7.0);
  v.setParam("sensing_distance", 20.0);
  v.setParam("robot_max_speed", 0.1);
  v.setParam("k1", 10);
  v.setParam("k2", 10);
  v.setParam("c1", 0.5);
  v.setParam("c2", 0.1);
  v.setParam("c3", 0.01);
  v.setParam("c4", 10.0);

  ValidatedVariables vv(v);

  Robot r1(Position_t(13.0, 13.0), 0);
  Robot r2(Position_t(3.0, 3.0), 1);
  Robot r3(Position_t(6.0, 8.0), 2);
  Robot r4(Position_t(11.0, 3.0), 3);
  Robot r5(Position_t(1.0, 8.0), 4);
  Robot r6(Position_t(8.0, 0.0), 6);

  auto rg = std::make_unique<RobotGraph>();
  auto r1_desc = boost::add_vertex(r1, *rg);
  boost::add_vertex(r2, *rg);
  boost::add_vertex(r3, *rg);
  boost::add_vertex(r4, *rg);

  auto og = std::make_unique<ObstacleGraph>();
  for (int i = 5; i < 21; i += 1)
  {
    boost::add_vertex(Obstacle(Position_t(8 + i, 18 + i), 0.1), *og);
    boost::add_vertex(Obstacle(Position_t(18 + i, 8 + i), 0.1), *og);
  }

  ValidatedGraphs vg(std::move(rg), std::move(og), vv);

  auto leaderV = Vector_t(sqrt(2), sqrt(2));
  boost::filesystem::create_directories("obstacleAnimation1");

  for (int i = 0; i < 1200; i++)
  {
    std::cout << "\tIteration " << i << " starts." << std::endl;
    auto start = std::chrono::system_clock::now();

    vg.tick(r1_desc, leaderV, vv);

    auto end = std::chrono::system_clock::now();
    int elapsed_seconds = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();
    std::cout << "\tIteration " << i << " ends in " << elapsed_seconds << std::endl;

    if (i % 100 == 0)
    {
      printPlotWithArrows("obstacleAnimation1/obstacleAnimation1_0_90_" + std::to_string(i) + ".png",
                          "obstacleAnimationTest1", 0, 90, 1, vg.getRobotGraph(), std::function(&overallPotential),
                          vg.getRobotGraph(), vg.getObstacleGraph(), v);
    }
  }
}

TEST(animatedNarrowCorridorTest_2, ShouldPass)
{
  Variables v = Variables();
  v.setParam("robots_avoidance_distance", 3.0);
  v.setParam("obstacles_avoidance_distance", 1.7);
  v.setParam("los_clearance_distance", 0.5);
  v.setParam("los_clearance_care_distance", 1.0);
  v.setParam("neighbourhood_distance", 10.0);
  v.setParam("edge_deletion_distance", 2.5);
  v.setParam("obstacle_care_distance", 2.0);
  v.setParam("desired_distance", 7.0);
  v.setParam("sensing_distance", 20.0);
  v.setParam("robot_max_speed", 0.1);
  v.setParam("k1", 10);
  v.setParam("k2", 10);
  v.setParam("c1", 0.5);
  v.setParam("c2", 0.1);
  v.setParam("c3", 0.01);
  v.setParam("c4", 10.0);

  ValidatedVariables vv(v);

  Robot r1(Position_t(13.0, 13.0), 0);
  Robot r2(Position_t(3.0, 3.0), 1);
  Robot r3(Position_t(6.0, 8.0), 2);
  Robot r4(Position_t(11.0, 3.0), 3);
  Robot r5(Position_t(1.0, 8.0), 4);
  Robot r6(Position_t(8.0, 0.0), 6);

  auto rg = std::make_unique<RobotGraph>();
  auto r1_desc = boost::add_vertex(r1, *rg);
  boost::add_vertex(r2, *rg);
  boost::add_vertex(r3, *rg);
  boost::add_vertex(r4, *rg);

  auto og = std::make_unique<ObstacleGraph>();
  for (int i = 5; i < 21; i += 1)
  {
    boost::add_vertex(Obstacle(Position_t(10 + i, 16 + i), 0.1), *og);
    boost::add_vertex(Obstacle(Position_t(16 + i, 10 + i), 0.1), *og);
  }

  ValidatedGraphs vg(std::move(rg), std::move(og), vv);

  auto leaderV = Vector_t(sqrt(2), sqrt(2));
  boost::filesystem::create_directories("obstacleAnimation2");

  for (int i = 0; i < 1200; i++)
  {
    std::cout << "\tIteration " << i << " starts." << std::endl;
    auto start = std::chrono::system_clock::now();

    vg.tick(r1_desc, leaderV, vv);

    auto end = std::chrono::system_clock::now();
    int elapsed_seconds = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();
    std::cout << "\tIteration " << i << " ends in " << elapsed_seconds << std::endl;

    if (i % 100 == 0)
    {
      printPlotWithArrows("obstacleAnimation2/obstacleAnimation2_0_90_" + std::to_string(i) + ".png",
                          "obstacleAnimationTest2", 0, 90, 1, vg.getRobotGraph(), std::function(&overallPotential),
                          vg.getRobotGraph(), vg.getObstacleGraph(), v);
    }
  }
}

TEST(animatedNarrowCorridorTestWithBigObstacles_3, ShouldPass)
{
  Variables v = Variables();
  v.setParam("robots_avoidance_distance", 3.0);
  v.setParam("obstacles_avoidance_distance", 1.7);
  v.setParam("los_clearance_distance", 0.5);
  v.setParam("los_clearance_care_distance", 1.0);
  v.setParam("neighbourhood_distance", 10.0);
  v.setParam("edge_deletion_distance", 2.5);
  v.setParam("obstacle_care_distance", 2.0);
  v.setParam("desired_distance", 7.0);
  v.setParam("sensing_distance", 20.0);
  v.setParam("robot_max_speed", 0.1);
  v.setParam("k1", 10);
  v.setParam("k2", 10);
  v.setParam("c1", 0.5);
  v.setParam("c2", 0.1);
  v.setParam("c3", 0.01);
  v.setParam("c4", 10.0);

  ValidatedVariables vv(v);

  Robot r1(Position_t(13.0, 13.0), 0);
  Robot r2(Position_t(3.0, 3.0), 1);
  Robot r3(Position_t(6.0, 8.0), 2);
  Robot r4(Position_t(11.0, 3.0), 3);
  Robot r5(Position_t(1.0, 8.0), 4);
  Robot r6(Position_t(8.0, 0.0), 6);

  auto rg = std::make_unique<RobotGraph>();
  auto r1_desc = boost::add_vertex(r1, *rg);
  boost::add_vertex(r2, *rg);
  boost::add_vertex(r3, *rg);
  boost::add_vertex(r4, *rg);

  auto og = std::make_unique<ObstacleGraph>();
  for (int i = 5; i < 21; i += 1)
  {
    boost::add_vertex(Obstacle(Position_t(8 + i, 18 + i), 5.0), *og);
    boost::add_vertex(Obstacle(Position_t(18 + i, 8 + i), 5.0), *og);
  }

  ValidatedGraphs vg(std::move(rg), std::move(og), vv);

  auto leaderV = Vector_t(sqrt(2), sqrt(2));
  boost::filesystem::create_directories("obstacleAnimation3");

  for (int i = 0; i < 1200; i++)
  {
    std::cout << "\tIteration " << i << " starts." << std::endl;
    auto start = std::chrono::system_clock::now();

    vg.tick(r1_desc, leaderV, vv);

    auto end = std::chrono::system_clock::now();
    int elapsed_seconds = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();
    std::cout << "\tIteration " << i << " ends in " << elapsed_seconds << std::endl;

    if ((i > 335 && i < 345) || i % 100 == 0)
    {
      printPlotWithArrows("obstacleAnimation3/obstacleAnimation3_0_90_" + std::to_string(i) + ".png",
                          "obstacleAnimationTest3", 0, 90, 1, vg.getRobotGraph(), std::function(&overallPotential),
                          vg.getRobotGraph(), vg.getObstacleGraph(), v);
    }
  }
}

TEST(animatedNarrowLongCorridorTestWithBigObstacles_4, ShouldPass)
{
  Variables v = Variables();
  v.setParam("robots_avoidance_distance", 3.0);
  v.setParam("obstacles_avoidance_distance", 1.7);
  v.setParam("los_clearance_distance", 0.5);
  v.setParam("los_clearance_care_distance", 1.0);
  v.setParam("neighbourhood_distance", 10.0);
  v.setParam("edge_deletion_distance", 2.5);
  v.setParam("obstacle_care_distance", 2.0);
  v.setParam("desired_distance", 7.0);
  v.setParam("sensing_distance", 20.0);
  v.setParam("robot_max_speed", 0.1);
  v.setParam("k1", 10);
  v.setParam("k2", 10);
  v.setParam("c1", 0.5);
  v.setParam("c2", 0.01);
  v.setParam("c3", 0.01);
  v.setParam("c4", 10.0);

  ValidatedVariables vv(v);

  Robot r1(Position_t(13.0, 13.0), 0);
  Robot r2(Position_t(3.0, 3.0), 1);
  Robot r3(Position_t(6.0, 8.0), 2);
  Robot r4(Position_t(11.0, 3.0), 3);
  Robot r5(Position_t(1.0, 8.0), 4);
  Robot r6(Position_t(8.0, 0.0), 6);

  auto rg = std::make_unique<RobotGraph>();
  auto r1_desc = boost::add_vertex(r1, *rg);
  boost::add_vertex(r2, *rg);
  boost::add_vertex(r3, *rg);
  boost::add_vertex(r4, *rg);

  auto og = std::make_unique<ObstacleGraph>();
  for (int i = 15; i < 41; i += 1)
  {
    boost::add_vertex(Obstacle(Position_t(5 + i, 21 + i), 9.0), *og);
    boost::add_vertex(Obstacle(Position_t(21 + i, 5 + i), 9.0), *og);
  }

  ValidatedGraphs vg(std::move(rg), std::move(og), vv);

  auto leaderV = Vector_t(sqrt(2), sqrt(2));
  boost::filesystem::create_directories("obstacleAnimation4");

  for (int i = 0; i < 1200; i++)
  {
    std::cout << "\tIteration " << i << " starts." << std::endl;
    auto start = std::chrono::system_clock::now();

    vg.tick(r1_desc, leaderV, vv);

    auto end = std::chrono::system_clock::now();
    int elapsed_seconds = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();
    std::cout << "\tIteration " << i << " ends in " << elapsed_seconds << std::endl;

    if (i % 100 == 0)
    {
      printPlotWithArrows("obstacleAnimation4/obstacleAnimation4_0_90_" + std::to_string(i) + ".png",
                          "obstacleAnimationTest4", 0, 90, 1, vg.getRobotGraph(), std::function(&overallPotential),
                          vg.getRobotGraph(), vg.getObstacleGraph(), v);
    }
  }
}

TEST(animatedNarrowVeryLongCorridorTestWithBigObstacles_5, ShouldPass)
{
  Variables v = Variables();
  v.setParam("robots_avoidance_distance", 3.0);
  v.setParam("obstacles_avoidance_distance", 1.7);
  v.setParam("los_clearance_distance", 0.5);
  v.setParam("los_clearance_care_distance", 1.0);
  v.setParam("neighbourhood_distance", 10.0);
  v.setParam("edge_deletion_distance", 2.5);
  v.setParam("obstacle_care_distance", 2.0);
  v.setParam("desired_distance", 7.0);
  v.setParam("sensing_distance", 20.0);
  v.setParam("robot_max_speed", 0.1);
  v.setParam("k1", 10);
  v.setParam("k2", 10);
  v.setParam("c1", 0.5);
  v.setParam("c2", 0.1);
  v.setParam("c3", 0.01);
  v.setParam("c4", 10.0);

  ValidatedVariables vv(v);

  Robot r1(Position_t(13.0, 13.0), 0);
  Robot r2(Position_t(3.0, 3.0), 1);
  Robot r3(Position_t(6.0, 8.0), 2);
  Robot r4(Position_t(11.0, 3.0), 3);
  Robot r5(Position_t(1.0, 8.0), 4);
  Robot r6(Position_t(8.0, 0.0), 6);

  auto rg = std::make_unique<RobotGraph>();
  auto r1_desc = boost::add_vertex(r1, *rg);
  boost::add_vertex(r2, *rg);
  boost::add_vertex(r3, *rg);
  boost::add_vertex(r4, *rg);

  auto og = std::make_unique<ObstacleGraph>();
  for (int i = 15; i < 41; i += 1)
  {
    boost::add_vertex(Obstacle(Position_t(5 + i, 21 + i), 9.0), *og);
    boost::add_vertex(Obstacle(Position_t(21 + i, 5 + i), 9.0), *og);
  }

  ValidatedGraphs vg(std::move(rg), std::move(og), vv);

  auto leaderV = Vector_t(sqrt(2), sqrt(2));
  boost::filesystem::create_directories("obstacleAnimation5");

  for (int i = 0; i < 1200; i++)
  {
    std::cout << "\tIteration " << i << " starts." << std::endl;
    auto start = std::chrono::system_clock::now();

    vg.tick(r1_desc, leaderV, vv);

    auto end = std::chrono::system_clock::now();
    int elapsed_seconds = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();
    std::cout << "\tIteration " << i << " ends in " << elapsed_seconds << std::endl;

    if (i % 100 == 0)
    {
      printPlotWithArrows("obstacleAnimation5/obstacleAnimation5_0_90_" + std::to_string(i) + ".png",
                          "obstacleAnimationTest5", 0, 90, 1, vg.getRobotGraph(), std::function(&overallPotential),
                          vg.getRobotGraph(), vg.getObstacleGraph(), v);
    }
  }
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
