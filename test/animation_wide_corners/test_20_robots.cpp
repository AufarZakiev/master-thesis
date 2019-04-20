#include <gtest/gtest.h>
#include "../../include/headers/speed_constraints.h"
#include "../../include/headers/field_functions.h"
#include <boost/filesystem.hpp>
#include <chrono>

TEST(wide_corners_20_robots_1, ShouldPass)
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

  Robot r1(Position_t(28.0, 30.0), 0);
  Robot r2(Position_t(26.0, 11.0), 1);
  Robot r3(Position_t(31.0, 26.0), 2);
  Robot r4(Position_t(25.0, 24.0), 3);
  Robot r5(Position_t(25.0, 28.0), 4);
  Robot r6(Position_t(20.0, 23.0), 5);
  Robot r7(Position_t(25.0, 20.0), 6);
  Robot r8(Position_t(30.0, 20.0), 7);
  Robot r9(Position_t(22.5, 18.0), 8);
  Robot r10(Position_t(27.5, 18.0), 9);

  Robot r11(Position_t(18.0, 20.0), 10);
  Robot r12(Position_t(30.0, 12.5), 11);
  Robot r13(Position_t(19.0, 15.0), 12);
  Robot r14(Position_t(30.0, 16.0), 13);
  Robot r15(Position_t(25.0, 7.0), 14);
  Robot r16(Position_t(10.0, 13.0), 15);
  Robot r17(Position_t(15.0, 10.0), 16);
  Robot r18(Position_t(20.0, 10.0), 17);
  Robot r19(Position_t(12.5, 8.0), 18);
  Robot r20(Position_t(27.5, 5.0), 19);

  auto rg = std::make_unique<RobotGraph>();
  auto r1_desc = boost::add_vertex(r1, *rg);
  boost::add_vertex(r2, *rg);
  boost::add_vertex(r3, *rg);
  boost::add_vertex(r4, *rg);
  boost::add_vertex(r5, *rg);
  boost::add_vertex(r6, *rg);
  boost::add_vertex(r7, *rg);
  boost::add_vertex(r8, *rg);
  boost::add_vertex(r9, *rg);
  boost::add_vertex(r10, *rg);

  boost::add_vertex(r11, *rg);
  boost::add_vertex(r12, *rg);
  boost::add_vertex(r13, *rg);
  boost::add_vertex(r14, *rg);
  boost::add_vertex(r15, *rg);
  boost::add_vertex(r16, *rg);
  boost::add_vertex(r17, *rg);
  boost::add_vertex(r18, *rg);
  boost::add_vertex(r19, *rg);
  boost::add_vertex(r20, *rg);

  auto og = std::make_unique<ObstacleGraph>();
  for (int i = 0; i < 10; i += 1)
  {
    boost::add_vertex(Obstacle(Position_t(20 - i, 30 - i), 1.0), *og);
    boost::add_vertex(Obstacle(Position_t(35 + i, 30 - i), 1.0), *og);
  }
  for (int i = -10; i < 61; i += 1)
  {
    boost::add_vertex(Obstacle(Position_t(20, 40 + i), 1.0), *og);
    boost::add_vertex(Obstacle(Position_t(35, 40 + i), 1.0), *og);
  }
  for (int i = 0; i < 16; i += 1)
  {
    boost::add_vertex(Obstacle(Position_t(20, 100 + i), 1.0), *og);
  }
  for (int i = 0; i < 31; i += 1)
  {
    boost::add_vertex(Obstacle(Position_t(20 + i, 115), 1.0), *og);
    boost::add_vertex(Obstacle(Position_t(35 + i, 100), 1.0), *og);
  }
  for (int i = 0; i < 40; i += 1)
  {
    boost::add_vertex(Obstacle(Position_t(50, 115 + i), 1.0), *og);
    boost::add_vertex(Obstacle(Position_t(65, 100 + i), 1.0), *og);
  }

  ValidatedGraphs vg(std::move(rg), std::move(og), vv);

  auto leaderV = Vector_t(0, 1);
  boost::filesystem::create_directories("wide_corners_20_robots_1");

  for (int i = 0; i < 5000; i++)
  {
    if (vg.getRobotGraph()[r1_desc].getPosition()(1, 0) > 107)
    {
      if (vg.getRobotGraph()[r1_desc].getPosition()(0, 0) > 57)
      {
        leaderV = Vector_t(0, 1);
      }
      else
      {
        leaderV = Vector_t(1, 0);
      }
    }

    std::cout << "\tIteration " << i << " starts." << std::endl;
    auto start = std::chrono::system_clock::now();

    vg.tick(r1_desc, leaderV, vv);

    auto end = std::chrono::system_clock::now();
    int elapsed_seconds = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();
    std::cout << "\tIteration " << i << " ends in " << elapsed_seconds << std::endl;

    if (i % 50 == 0)
    {
      printWidePlotWithArrows("wide_corners_20_robots_1/wide_corners_20_robots_1_" + std::to_string(i) + ".png",
                              "wide_corners_20_robots_1", 0, 90, 1, vg.getRobotGraph(),
                              std::function(&overallPotential), vg.getRobotGraph(), vg.getObstacleGraph(), v);
    }
  }
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
