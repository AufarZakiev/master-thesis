#include <gtest/gtest.h>
#include "../include/headers/speed_constraints.h"
#include "../include/headers/field_functions.h"
#include <boost/filesystem.hpp>
#include <chrono>
#include <thread>

TEST(animatedObstacleTest3, ShouldPass)
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
  v.setParam("c4", 30.0);

  ValidatedVariables vv(v);

  Robot r1(Position_t(8.0, 8.0));
  Robot r2(Position_t(0.0, 0.0));
  Robot r3(Position_t(3.0, 5.0));
  Robot r4(Position_t(8.0, 0.0));
  Robot r5(Position_t(1.0, 8.0));
  Robot r6(Position_t(8.0, 0.0));

  auto rg = std::make_unique<RobotGraph>();
  auto r1_desc = boost::add_vertex(r1, *rg);
  auto r2_desc = boost::add_vertex(r2, *rg);
  auto r3_desc = boost::add_vertex(r3, *rg);
  auto r4_desc = boost::add_vertex(r4, *rg);
  // auto r5_desc = boost::add_vertex(r5, *rg);
  // auto r6_desc = boost::add_vertex(r6, *rg);

  auto og = std::make_unique<ObstacleGraph>();
  boost::add_vertex(Obstacle(Position_t(15, 10), 0.7), *og);
  boost::add_vertex(Obstacle(Position_t(8, 15), 0.7), *og);
  boost::add_vertex(Obstacle(Position_t(17, 12), 0.7), *og);
  boost::add_vertex(Obstacle(Position_t(10, 17), 0.7), *og);
  boost::add_vertex(Obstacle(Position_t(19, 14), 0.7), *og);
  boost::add_vertex(Obstacle(Position_t(12, 19), 0.7), *og);
  boost::add_vertex(Obstacle(Position_t(21, 16), 0.7), *og);
  boost::add_vertex(Obstacle(Position_t(14, 21), 0.7), *og);
  boost::add_vertex(Obstacle(Position_t(23, 18), 0.7), *og);
  boost::add_vertex(Obstacle(Position_t(16, 23), 0.7), *og);

  //  boost::add_vertex(Obstacle(Position_t(20, 5), 1), *og);
  //  boost::add_vertex(Obstacle(Position_t(19, 6), 1), *og);
  //  boost::add_vertex(Obstacle(Position_t(18, 7), 1), *og);
  //  boost::add_vertex(Obstacle(Position_t(17, 8), 1), *og);

  ValidatedGraphs vg(std::move(rg), std::move(og), vv);

  auto leaderV = Vector_t(sqrt(2), sqrt(2));
  vg.getRobotGraph()[r1_desc].setSpeedDirection(leaderV *
                                                getConstrainedLeaderSpeed(vg.getRobotGraph()[r1_desc], vg, vv));
  vg.getRobotGraph()[r2_desc].setSpeedDirection(getConstrainedDirectedSpeed(vg.getRobotGraph()[r2_desc], vg, vv));
  vg.getRobotGraph()[r3_desc].setSpeedDirection(getConstrainedDirectedSpeed(vg.getRobotGraph()[r3_desc], vg, vv));
  vg.getRobotGraph()[r4_desc].setSpeedDirection(getConstrainedDirectedSpeed(vg.getRobotGraph()[r4_desc], vg, vv));
  //vg.getRobotGraph()[r5_desc].setSpeedDirection(getConstrainedDirectedSpeed(vg.getRobotGraph()[r5_desc], vg, vv));
  //vg.getRobotGraph()[r6_desc].setSpeedDirection(getConstrainedDirectedSpeed(vg.getRobotGraph()[r6_desc], vg, vv));

  boost::filesystem::create_directories("obstacleAnimation3");
  printPlotWithArrows("obstacleAnimation3/obstacleAnimation3_0_90_1.png", "obstacleAnimationtTest3", 0, 90, 1,
                      vg.getRobotGraph(), std::function(&overallPotential), vg.getRobotGraph(), vg.getObstacleGraph(),
                      v);

  for (int i = 2; i < 1000; i++)
  {
    auto start = std::chrono::system_clock::now();
    vg.getRobotGraph()[r1_desc].updatePosition();
    vg.getRobotGraph()[r2_desc].updatePosition();
    vg.getRobotGraph()[r3_desc].updatePosition();
    vg.getRobotGraph()[r4_desc].updatePosition();
    //vg.getRobotGraph()[r5_desc].updatePosition();
    //vg.getRobotGraph()[r6_desc].updatePosition();
    vg.getRobotGraph()[r1_desc].setSpeedDirection(leaderV *
                                                  getConstrainedLeaderSpeed(vg.getRobotGraph()[r1_desc], vg, vv));
    vg.getRobotGraph()[r2_desc].setSpeedDirection(getConstrainedDirectedSpeed(vg.getRobotGraph()[r2_desc], vg, vv));
    vg.getRobotGraph()[r3_desc].setSpeedDirection(getConstrainedDirectedSpeed(vg.getRobotGraph()[r3_desc], vg, vv));
    vg.getRobotGraph()[r4_desc].setSpeedDirection(getConstrainedDirectedSpeed(vg.getRobotGraph()[r4_desc], vg, vv));
    //vg.getRobotGraph()[r5_desc].setSpeedDirection(getConstrainedDirectedSpeed(vg.getRobotGraph()[r5_desc], vg, vv));
    //vg.getRobotGraph()[r6_desc].setSpeedDirection(getConstrainedDirectedSpeed(vg.getRobotGraph()[r6_desc], vg, vv));
    vg.leavePreservedEdges(vv);
    auto end = std::chrono::system_clock::now();
    int elapsed_seconds = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();
    std::cout << "Iteration " << i << ": " << elapsed_seconds << std::endl;

    if (i % 25 == 0)
    {
      // std::thread draw([&]() {
      printPlotWithArrows("obstacleAnimation3/obstacleAnimation3_0_90_" + std::to_string(i) + ".png",
                          "obstacleAnimationTest3", 0, 90, 1, vg.getRobotGraph(), std::function(&overallPotential),
                          vg.getRobotGraph(), vg.getObstacleGraph(), v);
      //});
      // draw.detach();
    }
  }
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
