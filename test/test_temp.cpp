#include <gtest/gtest.h>
#include "../include/headers/speed_constraints.h"
#include "../include/headers/field_functions.h"
#include <boost/filesystem.hpp>
#include <chrono>
#include <thread>

TEST(animatedNarrowLongCorridorTestWithBigObstacles, ShouldPass)
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

  Robot r1(Position_t(13.0, 13.0));
  Robot r2(Position_t(3.0, 3.0));
  Robot r3(Position_t(6.0, 8.0));
  Robot r4(Position_t(11.0, 3.0));
  Robot r5(Position_t(1.0, 8.0));
  Robot r6(Position_t(8.0, 0.0));

  auto rg = std::make_unique<RobotGraph>();
  auto r1_desc = boost::add_vertex(r1, *rg);
  auto r2_desc = boost::add_vertex(r2, *rg);
  auto r3_desc = boost::add_vertex(r3, *rg);
  auto r4_desc = boost::add_vertex(r4, *rg);

  auto og = std::make_unique<ObstacleGraph>();
  boost::add_vertex(Obstacle(Position_t(21, 15), 0.15), *og);
  boost::add_vertex(Obstacle(Position_t(15, 19), 0.15), *og);
  boost::add_vertex(Obstacle(Position_t(23, 17), 0.15), *og);
  boost::add_vertex(Obstacle(Position_t(17, 21), 0.15), *og);
  boost::add_vertex(Obstacle(Position_t(25, 19), 0.15), *og);
  boost::add_vertex(Obstacle(Position_t(19, 23), 0.15), *og);
  boost::add_vertex(Obstacle(Position_t(27, 21), 0.15), *og);
  boost::add_vertex(Obstacle(Position_t(21, 25), 0.15), *og);
  boost::add_vertex(Obstacle(Position_t(29, 23), 0.15), *og);
  boost::add_vertex(Obstacle(Position_t(23, 27), 0.15), *og);
  boost::add_vertex(Obstacle(Position_t(31, 25), 0.15), *og);
  boost::add_vertex(Obstacle(Position_t(25, 29), 0.15), *og);

  ValidatedGraphs vg(std::move(rg), std::move(og), vv);

  auto leaderV = Vector_t(sqrt(2), sqrt(2));
  vg.getRobotGraph()[r1_desc].setSpeedDirection(leaderV *
                                                getConstrainedLeaderSpeed(vg.getRobotGraph()[r1_desc], vg, vv));
  vg.getRobotGraph()[r2_desc].setSpeedDirection(getConstrainedDirectedSpeed(vg.getRobotGraph()[r2_desc], vg, vv));
  vg.getRobotGraph()[r3_desc].setSpeedDirection(getConstrainedDirectedSpeed(vg.getRobotGraph()[r3_desc], vg, vv));
  vg.getRobotGraph()[r4_desc].setSpeedDirection(getConstrainedDirectedSpeed(vg.getRobotGraph()[r4_desc], vg, vv));

  boost::filesystem::create_directories("obstacleAnimation4");
  printPlotWithArrows("obstacleAnimation4/obstacleAnimation4_0_90_1.png", "obstacleAnimationtTest4", 0, 90, 1,
                      vg.getRobotGraph(), std::function(&overallPotential), vg.getRobotGraph(), vg.getObstacleGraph(),
                      v);

  for (int i = 2; i < 1200; i++)
  {
    auto start = std::chrono::system_clock::now();
    vg.getRobotGraph()[r1_desc].updatePosition();
    vg.getRobotGraph()[r2_desc].updatePosition();
    vg.getRobotGraph()[r3_desc].updatePosition();
    vg.getRobotGraph()[r4_desc].updatePosition();
    vg.leavePreservedEdges(vv);
    vg.getRobotGraph()[r1_desc].setSpeedDirection(leaderV *
                                                  getConstrainedLeaderSpeed(vg.getRobotGraph()[r1_desc], vg, vv));
    vg.getRobotGraph()[r2_desc].setSpeedDirection(getConstrainedDirectedSpeed(vg.getRobotGraph()[r2_desc], vg, vv));
    vg.getRobotGraph()[r3_desc].setSpeedDirection(getConstrainedDirectedSpeed(vg.getRobotGraph()[r3_desc], vg, vv));
    vg.getRobotGraph()[r4_desc].setSpeedDirection(getConstrainedDirectedSpeed(vg.getRobotGraph()[r4_desc], vg, vv));

    auto end = std::chrono::system_clock::now();
    int elapsed_seconds = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();
    std::cout << "Iteration " << i << ": " << elapsed_seconds << std::endl;

    if ((i >= 300 && i <= 500 && i % 10 == 0) || i % 100 == 0)
    {
      printPlotWithArrows("obstacleAnimation4/obstacleAnimation4_0_90_" + std::to_string(i) + ".png",
                          "obstacleAnimationTest4", 0, 90, 1, vg.getRobotGraph(), std::function(&overallPotential),
                          vg.getRobotGraph(), vg.getObstacleGraph(), v);
    }
  }
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
