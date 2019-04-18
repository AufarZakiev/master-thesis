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

  Robot r1(Position_t(28.0, 28.0), 0);
  Robot r2(Position_t(18.0, 18.0), 1);
  Robot r3(Position_t(21.0, 23.0), 2);
  Robot r4(Position_t(26.0, 18.0), 3);
  Robot r5(Position_t(16.0, 23.0), 4);
  Robot r6(Position_t(23.0, 15.0), 5);
  Robot r7(Position_t(23.0, 30.0), 6);
  Robot r8(Position_t(30.0, 23.0), 7);
  Robot r9(Position_t(17.5, 15.0), 8);
  Robot r10(Position_t(15.0, 17.5), 9);

  Robot r11(Position_t(2.0, 2.0), 10);
  Robot r12(Position_t(12.0, 12.0), 11);
  Robot r13(Position_t(9.0, 7.0), 12);
  Robot r14(Position_t(4.0, 12.0), 13);
  Robot r15(Position_t(14.0, 7.0), 14);
  Robot r16(Position_t(7.0, 15.0), 15);
  Robot r17(Position_t(7.0, 0.0), 16);
  Robot r18(Position_t(0.0, 7.0), 17);
  Robot r19(Position_t(12.5, 15.0), 18);
  Robot r20(Position_t(15.0, 12.5), 19);

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
  for (int i = 20; i < 41; i += 1)
  {
    boost::add_vertex(Obstacle(Position_t(8 + i, 18 + i), 1.0), *og);
    boost::add_vertex(Obstacle(Position_t(18 + i, 8 + i), 1.0), *og);
  }
  for (int i = 41; i < 51; i += 1)
  {
    boost::add_vertex(Obstacle(Position_t(18 + i, 8 + i), 1.0), *og);
  }
  for (int i = 0; i < 16; i += 1)
  {
    boost::add_vertex(Obstacle(Position_t(49 - i, 59 + i), 1.0), *og);
    boost::add_vertex(Obstacle(Position_t(69 - i, 59 + i), 1.0), *og);
  }
  for (int i = 0; i < 40; i += 1)
  {
    boost::add_vertex(Obstacle(Position_t(34, 74 + i), 1.0), *og);
    boost::add_vertex(Obstacle(Position_t(54, 74 + i), 1.0), *og);
  }

  ValidatedGraphs vg(std::move(rg), std::move(og), vv);

  auto leaderV = Vector_t(sqrt(2), sqrt(2));
  boost::filesystem::create_directories("wide_corners_20_robots_1");

  for (int i = 0; i < 5000; i++)
  {
    if (vg.getRobotGraph()[r1_desc].getPosition()(1, 0) > 59)
    {
      leaderV = Vector_t(-sqrt(2), sqrt(2));
    }

    if (vg.getRobotGraph()[r1_desc].getPosition()(1, 0) > 74)
    {
      leaderV = Vector_t(0, 1);
    }

    std::cout << "\tIteration " << i << " starts." << std::endl;
    auto start = std::chrono::system_clock::now();

    vg.tick(r1_desc, leaderV, vv);

    auto end = std::chrono::system_clock::now();
    int elapsed_seconds = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();
    std::cout << "\tIteration " << i << " ends in " << elapsed_seconds << std::endl;

    if (i % 200 == 0)
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
