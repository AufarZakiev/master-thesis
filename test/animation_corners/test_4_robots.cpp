#include <gtest/gtest.h>
#include "../../include/headers/speed_constraints.h"
#include "../../include/headers/field_functions.h"
#include <boost/filesystem.hpp>
#include <chrono>

TEST(corners_4_robots_1, ShouldPass)
{
  Variables v = Variables();
  v.setParam("robots_avoidance_distance", 3.0);
  v.setParam("obstacles_avoidance_distance", 1.7);
  v.setParam("los_clearance_distance", 0.1);
  v.setParam("los_clearance_care_distance", 6.5);
  v.setParam("neighbourhood_distance", 10.0);
  v.setParam("edge_deletion_distance", 2.5);
  v.setParam("obstacle_care_distance", 6.5);
  v.setParam("desired_distance", 6.5);
  v.setParam("sensing_distance", 10.5);
  v.setParam("robot_max_speed", 0.1);
  v.setParam("k1", 10);
  v.setParam("k2", 10);
  v.setParam("c1", 2.5);
  v.setParam("c2", 1.25);
  v.setParam("c3", 0.1);
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
    boost::add_vertex(Obstacle(Position_t(8 + i, 18 + i), 3.0), *og);
    boost::add_vertex(Obstacle(Position_t(18 + i, 8 + i), 3.0), *og);
  }
  for (int i = 21; i < 31; i += 1)
  {
    boost::add_vertex(Obstacle(Position_t(18 + i, 8 + i), 3.0), *og);
  }

  for (int i = 0; i < 10; i += 1)
  {
    boost::add_vertex(Obstacle(Position_t(49 - i, 39 + i), 3.0), *og);
  }
  for (int i = 10; i < 26; i += 1)
  {
    boost::add_vertex(Obstacle(Position_t(39 - i, 29 + i), 3.0), *og);
    boost::add_vertex(Obstacle(Position_t(49 - i, 39 + i), 3.0), *og);
  }

  ValidatedGraphs vg(std::move(rg), std::move(og), vv);

  auto leaderV = Vector_t(sqrt(2), sqrt(2));
  boost::filesystem::create_directories("corners_4_robots_1");

  for (int i = 0; i <= 1200; i++)
  {
    if (vg.getRobotGraph()[r1_desc].getPosition()(1, 0) > 39)
    {
      leaderV = Vector_t(-sqrt(2), sqrt(2));
    }
    std::cout << "\tIteration " << i << " starts." << std::endl;
    auto start = std::chrono::system_clock::now();

    vg.tick(r1_desc, leaderV, vv);

    auto end = std::chrono::system_clock::now();
    int elapsed_seconds = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();
    std::cout << "\tIteration " << i << " ends in " << elapsed_seconds << std::endl;

    if (i % 200 == 0)
    {
      printPlotWithArrows("corners_4_robots_1/corners_4_robots_1_" + std::to_string(i) + ".png",
                          "corners_4_robots_1", 0, 90, 1, vg.getRobotGraph(), std::function(&overallPotential),
                          vg.getRobotGraph(), vg.getObstacleGraph(), v);
    }
  }
}

TEST(corners_4_robots_2, ShouldPass)
{
  Variables v = Variables();
  v.setParam("robots_avoidance_distance", 3.0);
  v.setParam("obstacles_avoidance_distance", 1.7);
  v.setParam("los_clearance_distance", 0.1);
  v.setParam("los_clearance_care_distance", 6.5);
  v.setParam("neighbourhood_distance", 10.0);
  v.setParam("edge_deletion_distance", 2.5);
  v.setParam("obstacle_care_distance", 6.5);
  v.setParam("desired_distance", 6.5);
  v.setParam("sensing_distance", 10.5);
  v.setParam("robot_max_speed", 0.1);
  v.setParam("k1", 10);
  v.setParam("k2", 10);
  v.setParam("c1", 2.5);
  v.setParam("c2", 1.25);
  v.setParam("c3", 0.1);
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
    boost::add_vertex(Obstacle(Position_t(8 + i, 18 + i), 4.5), *og);
    boost::add_vertex(Obstacle(Position_t(18 + i, 8 + i), 4.5), *og);
  }
  for (int i = 21; i < 31; i += 1)
  {
    boost::add_vertex(Obstacle(Position_t(18 + i, 8 + i), 4.5), *og);
  }

  for (int i = 0; i < 16; i += 1)
  {
    boost::add_vertex(Obstacle(Position_t(29 - i, 39 + i), 4.5), *og);
    boost::add_vertex(Obstacle(Position_t(49 - i, 39 + i), 4.5), *og);
  }
  for (int i = 0; i < 10; i += 1)
  {
    boost::add_vertex(Obstacle(Position_t(34 - i, 54 + i), 4.5), *og);
  }

  ValidatedGraphs vg(std::move(rg), std::move(og), vv);

  auto leaderV = Vector_t(sqrt(2), sqrt(2));
  boost::filesystem::create_directories("corners_4_robots_2");

  for (int i = 0; i <= 1200; i++)
  {
    if (vg.getRobotGraph()[r1_desc].getPosition()(1, 0) > 39)
    {
      leaderV = Vector_t(-sqrt(2), sqrt(2));
    }
    std::cout << "\tIteration " << i << " starts." << std::endl;
    auto start = std::chrono::system_clock::now();

    vg.tick(r1_desc, leaderV, vv);

    auto end = std::chrono::system_clock::now();
    int elapsed_seconds = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();
    std::cout << "\tIteration " << i << " ends in " << elapsed_seconds << std::endl;

    if (i % 200 == 0)
    {
      printPlotWithArrows("corners_4_robots_2/corners_4_robots_2_" + std::to_string(i) + ".png",
                          "corners_4_robots_2", 0, 90, 1, vg.getRobotGraph(), std::function(&overallPotential),
                          vg.getRobotGraph(), vg.getObstacleGraph(), v);
    }
  }
}

TEST(corners_4_robots_3, ShouldPass)
{
  Variables v = Variables();
  v.setParam("robots_avoidance_distance", 3.0);
  v.setParam("obstacles_avoidance_distance", 1.7);
  v.setParam("los_clearance_distance", 0.1);
  v.setParam("los_clearance_care_distance", 6.5);
  v.setParam("neighbourhood_distance", 10.0);
  v.setParam("edge_deletion_distance", 2.5);
  v.setParam("obstacle_care_distance", 6.5);
  v.setParam("desired_distance", 6.5);
  v.setParam("sensing_distance", 20.0);
  v.setParam("robot_max_speed", 0.1);
  v.setParam("k1", 10);
  v.setParam("k2", 10);
  v.setParam("c1", 2.5);
  v.setParam("c2", 1.25);
  v.setParam("c3", 0.1);
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
    boost::add_vertex(Obstacle(Position_t(8 + i, 18 + i), 4.5), *og);
    boost::add_vertex(Obstacle(Position_t(18 + i, 8 + i), 4.5), *og);
  }
  for (int i = 21; i < 31; i += 1)
  {
    boost::add_vertex(Obstacle(Position_t(18 + i, 8 + i), 4.5), *og);
  }
  for (int i = 0; i < 16; i += 1)
  {
    boost::add_vertex(Obstacle(Position_t(29 - i, 39 + i), 4.5), *og);
    boost::add_vertex(Obstacle(Position_t(49 - i, 39 + i), 4.5), *og);
  }
  for (int i = 0; i < 16; i += 1)
  {
    boost::add_vertex(Obstacle(Position_t(14 + i, 54 + i), 4.5), *og);
    boost::add_vertex(Obstacle(Position_t(34 + i, 54 + i), 4.5), *og);
  }

  ValidatedGraphs vg(std::move(rg), std::move(og), vv);

  auto leaderV = Vector_t(sqrt(2), sqrt(2));
  boost::filesystem::create_directories("corners_4_robots_3");

  for (int i = 0; i < 1500; i++)
  {
    if (vg.getRobotGraph()[r1_desc].getPosition()(1, 0) > 39)
    {
      leaderV = Vector_t(-sqrt(2), sqrt(2));
    }

    if (vg.getRobotGraph()[r1_desc].getPosition()(1, 0) > 54)
    {
      leaderV = Vector_t(sqrt(2), sqrt(2));
    }
    std::cout << "\tIteration " << i << " starts." << std::endl;
    auto start = std::chrono::system_clock::now();

    vg.tick(r1_desc, leaderV, vv);

    auto end = std::chrono::system_clock::now();
    int elapsed_seconds = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();
    std::cout << "\tIteration " << i << " ends in " << elapsed_seconds << std::endl;

    if (i % 200 == 0)
    {
      printPlotWithArrows("corners_4_robots_3/corners_4_robots_3_" + std::to_string(i) + ".png",
                          "corners_4_robots_3", 0, 90, 1, vg.getRobotGraph(), std::function(&overallPotential),
                          vg.getRobotGraph(), vg.getObstacleGraph(), v);
    }
  }
}

TEST(corners_4_robots_4, ShouldPass)
{
  Variables v = Variables();
  v.setParam("robots_avoidance_distance", 3.0);
  v.setParam("obstacles_avoidance_distance", 1.7);
  v.setParam("los_clearance_distance", 0.1);
  v.setParam("los_clearance_care_distance", 6.5);
  v.setParam("neighbourhood_distance", 10.0);
  v.setParam("edge_deletion_distance", 2.5);
  v.setParam("obstacle_care_distance", 6.5);
  v.setParam("desired_distance", 6.5);
  v.setParam("sensing_distance", 20.0);
  v.setParam("robot_max_speed", 0.1);
  v.setParam("k1", 10);
  v.setParam("k2", 10);
  v.setParam("c1", 2.5);
  v.setParam("c2", 1.25);
  v.setParam("c3", 0.1);
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
    boost::add_vertex(Obstacle(Position_t(8 + i, 18 + i), 4.5), *og);
    boost::add_vertex(Obstacle(Position_t(18 + i, 8 + i), 4.5), *og);
  }
  for (int i = 21; i < 31; i += 1)
  {
    boost::add_vertex(Obstacle(Position_t(18 + i, 8 + i), 4.5), *og);
  }
  for (int i = 0; i < 16; i += 1)
  {
    boost::add_vertex(Obstacle(Position_t(29 - i, 39 + i), 4.5), *og);
    boost::add_vertex(Obstacle(Position_t(49 - i, 39 + i), 4.5), *og);
  }
  for (int i = 0; i < 16; i += 1)
  {
    boost::add_vertex(Obstacle(Position_t(14, 54 + i), 4.5), *og);
  }
  for (int i = 0; i < 30; i += 1)
  {
    boost::add_vertex(Obstacle(Position_t(14 + i * sqrt(2), 69), 4.5), *og);
    boost::add_vertex(Obstacle(Position_t(34 + i * sqrt(2), 54), 4.5), *og);
  }

  ValidatedGraphs vg(std::move(rg), std::move(og), vv);

  auto leaderV = Vector_t(sqrt(2), sqrt(2));
  boost::filesystem::create_directories("corners_4_robots_4");

  for (int i = 0; i < 2500; i++)
  {
    if (vg.getRobotGraph()[r1_desc].getPosition()(1, 0) > 39)
    {
      leaderV = Vector_t(-sqrt(2), sqrt(2));
    }

    if (vg.getRobotGraph()[r1_desc].getPosition()(1, 0) > 54)
    {
      leaderV = Vector_t(0, 1);
    }

    if (vg.getRobotGraph()[r1_desc].getPosition()(1, 0) > 61)
    {
      leaderV = Vector_t(1, 0);
    }

    std::cout << "\tIteration " << i << " starts." << std::endl;
    auto start = std::chrono::system_clock::now();

    vg.tick(r1_desc, leaderV, vv);

    auto end = std::chrono::system_clock::now();
    int elapsed_seconds = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();
    std::cout << "\tIteration " << i << " ends in " << elapsed_seconds << std::endl;

    if (i % 200 == 0)
    {
      printPlotWithArrows("corners_4_robots_4/corners_4_robots_4_" + std::to_string(i) + ".png",
                          "corners_4_robots_4", 0, 90, 1, vg.getRobotGraph(), std::function(&overallPotential),
                          vg.getRobotGraph(), vg.getObstacleGraph(), v);
    }
  }
}

TEST(corners_4_robots_5, ShouldPass)
{
  Variables v = Variables();
  v.setParam("robots_avoidance_distance", 3.0);
  v.setParam("obstacles_avoidance_distance", 1.7);
  v.setParam("los_clearance_distance", 0.1);
  v.setParam("los_clearance_care_distance", 6.5);
  v.setParam("neighbourhood_distance", 10.0);
  v.setParam("edge_deletion_distance", 2.5);
  v.setParam("obstacle_care_distance", 6.5);
  v.setParam("desired_distance", 6.5);
  v.setParam("sensing_distance", 20.0);
  v.setParam("robot_max_speed", 0.1);
  v.setParam("k1", 10);
  v.setParam("k2", 10);
  v.setParam("c1", 2.5);
  v.setParam("c2", 1.25);
  v.setParam("c3", 0.1);
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
    boost::add_vertex(Obstacle(Position_t(8 + i, 18 + i), 3.0), *og);
    boost::add_vertex(Obstacle(Position_t(18 + i, 8 + i), 3.0), *og);
  }
  for (int i = 21; i < 31; i += 1)
  {
    boost::add_vertex(Obstacle(Position_t(18 + i, 8 + i), 3.0), *og);
  }
  for (int i = 0; i < 16; i += 1)
  {
    boost::add_vertex(Obstacle(Position_t(29 - i, 39 + i), 3.0), *og);
    boost::add_vertex(Obstacle(Position_t(49 - i, 39 + i), 3.0), *og);
  }
  for (int i = 0; i < 16; i += 1)
  {
    boost::add_vertex(Obstacle(Position_t(14, 54 + i), 5.0), *og);
  }
  for (int i = 0; i < 30; i += 1)
  {
    boost::add_vertex(Obstacle(Position_t(14 + i * sqrt(2), 69), 5.0), *og);
    boost::add_vertex(Obstacle(Position_t(34 + i * sqrt(2), 54), 5.0), *og);
  }

  ValidatedGraphs vg(std::move(rg), std::move(og), vv);

  auto leaderV = Vector_t(sqrt(2), sqrt(2));
  boost::filesystem::create_directories("corners_4_robots_5");

  for (int i = 0; i < 2300; i++)
  {
    if (vg.getRobotGraph()[r1_desc].getPosition()(1, 0) > 39)
    {
      leaderV = Vector_t(-sqrt(2), sqrt(2));
    }

    if (vg.getRobotGraph()[r1_desc].getPosition()(1, 0) > 54)
    {
      leaderV = Vector_t(0, 1);
    }

    if (vg.getRobotGraph()[r1_desc].getPosition()(1, 0) > 61)
    {
      leaderV = Vector_t(1, 0);
    }

    std::cout << "\tIteration " << i << " starts." << std::endl;
    auto start = std::chrono::system_clock::now();

    vg.tick(r1_desc, leaderV, vv);

    auto end = std::chrono::system_clock::now();
    int elapsed_seconds = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();
    std::cout << "\tIteration " << i << " ends in " << elapsed_seconds << std::endl;

    if (i % 200 == 0)
    {
      printPlotWithArrows("corners_4_robots_5/corners_4_robots_5_" + std::to_string(i) + ".png",
                          "corners_4_robots_5", 0, 90, 1, vg.getRobotGraph(), std::function(&overallPotential),
                          vg.getRobotGraph(), vg.getObstacleGraph(), v);
    }
  }
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
