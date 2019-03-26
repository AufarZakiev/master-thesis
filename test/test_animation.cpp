#include <gtest/gtest.h>
#include "../include/headers/speed_constraints.h"
#include "../include/headers/field_functions.h"
#include <boost/filesystem.hpp>
#include <chrono>
#include <thread>

TEST(animatedCohesionTest, ShouldPass)
{
  Variables v = Variables();
  v.setParam("robots_avoidance_distance", 3.0);
  v.setParam("obstacles_avoidance_distance", 1.7);
  v.setParam("los_clearance_distance", 0.5);
  v.setParam("los_clearance_care_distance", 1.0);
  v.setParam("neighbourhood_distance", 15.0);
  v.setParam("edge_deletion_distance", 2.5);
  v.setParam("obstacle_care_distance", 2.0);
  v.setParam("desired_distance", 7.0);
  v.setParam("sensing_distance", 20.0);
  v.setParam("robot_max_speed", 0.1);
  v.setParam("k1", 10);
  v.setParam("k2", 10);
  v.setParam("c1", 0.0);
  v.setParam("c2", 0.0);
  v.setParam("c3", 0.0);
  v.setParam("c4", 10.0);

  ValidatedVariables vv(v);

  Robot r1(Vector_t(5.0, 3.0));
  Robot r2(Vector_t(10.0, 3.0));
  Robot r3(Vector_t(7.5, 14.0));

  auto rg = std::make_unique<RobotGraph>();
  auto r1_desc = boost::add_vertex(r1, *rg);
  auto r2_desc = boost::add_vertex(r2, *rg);
  auto r3_desc = boost::add_vertex(r3, *rg);
  boost::add_edge(r1_desc, r2_desc, *rg);
  boost::add_edge(r2_desc, r3_desc, *rg);

  auto og = std::make_unique<ObstacleGraph>();

  ValidatedGraphs vg(std::move(rg), std::move(og), vv);

  vg.getRobotGraph()[r1_desc].setSpeedDirection(getConstrainedDirectedSpeed(vg.getRobotGraph()[r1_desc], vg, vv));
  vg.getRobotGraph()[r2_desc].setSpeedDirection(getConstrainedDirectedSpeed(vg.getRobotGraph()[r2_desc], vg, vv));
  vg.getRobotGraph()[r3_desc].setSpeedDirection(getConstrainedDirectedSpeed(vg.getRobotGraph()[r3_desc], vg, vv));

  boost::filesystem::create_directories("cohesionAnimation");
  printPlotWithArrows("cohesionAnimation/cohesionAnimationTest_0_90_1.png", "cohesionAnimationTest", 0, 90, 1,
                      vg.getRobotGraph(), std::function(&overallPotential), vg.getRobotGraph(), vg.getObstacleGraph(),
                      v);

  for (int i = 2; i < 6; i++)
  {
    vg.getRobotGraph()[r1_desc].updatePosition();
    vg.getRobotGraph()[r2_desc].updatePosition();
    vg.getRobotGraph()[r3_desc].updatePosition();
    vg.getRobotGraph()[r1_desc].setSpeedDirection(getConstrainedDirectedSpeed(vg.getRobotGraph()[r1_desc], vg, vv));
    vg.getRobotGraph()[r2_desc].setSpeedDirection(getConstrainedDirectedSpeed(vg.getRobotGraph()[r2_desc], vg, vv));
    vg.getRobotGraph()[r3_desc].setSpeedDirection(getConstrainedDirectedSpeed(vg.getRobotGraph()[r3_desc], vg, vv));

    printPlotWithArrows("cohesionAnimation/cohesionAnimationTest_0_90_" + std::to_string(i) + ".png",
                        "cohesionAnimationTest", 0, 90, 1, vg.getRobotGraph(), std::function(&overallPotential),
                        vg.getRobotGraph(), vg.getObstacleGraph(), v);
  }
}

TEST(animatedLeaderTest, ShouldPass)
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
  v.setParam("c1", 0.0);
  v.setParam("c2", 0.0);
  v.setParam("c3", 0.0);
  v.setParam("c4", 10.0);

  ValidatedVariables vv(v);

  Robot r1(Vector_t(5.0, 3.0));
  Robot r2(Vector_t(5.0, 10.0));
  Robot r3(Vector_t(0.0, 3.0));
  auto rg = std::make_unique<RobotGraph>();
  auto r1_desc = boost::add_vertex(r1, *rg);
  auto r2_desc = boost::add_vertex(r2, *rg);
  auto r3_desc = boost::add_vertex(r3, *rg);

  auto og = std::make_unique<ObstacleGraph>();

  ValidatedGraphs vg(std::move(rg), std::move(og), vv);

  auto leaderV = Vector_t(0.5, 0.5);
  vg.getRobotGraph()[r1_desc].setSpeedDirection(leaderV);
  vg.getRobotGraph()[r2_desc].setSpeedDirection(getConstrainedDirectedSpeed(vg.getRobotGraph()[r2_desc], vg, vv));
  vg.getRobotGraph()[r3_desc].setSpeedDirection(getConstrainedDirectedSpeed(vg.getRobotGraph()[r3_desc], vg, vv));

  boost::filesystem::create_directories("leaderAnimation");
  printPlotWithArrows("leaderAnimation/leaderAnimationTest_0_90_1.png", "leaderAnimationTest", 0, 90, 1,
                      vg.getRobotGraph(), std::function(&overallPotential), vg.getRobotGraph(), vg.getObstacleGraph(),
                      v);

  for (int i = 2; i < 12; i++)
  {
    vg.getRobotGraph()[r1_desc].updatePosition();
    vg.getRobotGraph()[r2_desc].updatePosition();
    vg.getRobotGraph()[r3_desc].updatePosition();
    vg.getRobotGraph()[r2_desc].setSpeedDirection(getConstrainedDirectedSpeed(vg.getRobotGraph()[r2_desc], vg, vv));
    vg.getRobotGraph()[r3_desc].setSpeedDirection(getConstrainedDirectedSpeed(vg.getRobotGraph()[r3_desc], vg, vv));

    printPlotWithArrows("leaderAnimation/leaderAnimationTest_0_90_" + std::to_string(i) + ".png", "leaderAnimationTest",
                        0, 90, 1, vg.getRobotGraph(), std::function(&overallPotential), vg.getRobotGraph(),
                        vg.getObstacleGraph(), v);
  }
}

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

  Robot r1(Vector_t(8.0, 8.0));
  Robot r2(Vector_t(0.0, 0.0));
  Robot r3(Vector_t(3.0, 5.0));
  Robot r4(Vector_t(7.0, 4.0));
  Robot r5(Vector_t(1.0, 8.0));
  auto rg = std::make_unique<RobotGraph>();
  auto r1_desc = boost::add_vertex(r1, *rg);
  auto r2_desc = boost::add_vertex(r2, *rg);
  auto r3_desc = boost::add_vertex(r3, *rg);
  auto r4_desc = boost::add_vertex(r4, *rg);
  auto r5_desc = boost::add_vertex(r5, *rg);

  auto og = std::make_unique<ObstacleGraph>();

  ValidatedGraphs vg(std::move(rg), std::move(og), vv);

  auto leaderV = Vector_t(sqrt(2), sqrt(2));
  vg.leavePreservedEdges(vv);
  vg.getRobotGraph()[r1_desc].setSpeedDirection(leaderV *
                                                getConstrainedLeaderSpeed(vg.getRobotGraph()[r1_desc], vg, vv));
  vg.getRobotGraph()[r2_desc].setSpeedDirection(getConstrainedDirectedSpeed(vg.getRobotGraph()[r2_desc], vg, vv));
  vg.getRobotGraph()[r3_desc].setSpeedDirection(getConstrainedDirectedSpeed(vg.getRobotGraph()[r3_desc], vg, vv));
  vg.getRobotGraph()[r4_desc].setSpeedDirection(getConstrainedDirectedSpeed(vg.getRobotGraph()[r4_desc], vg, vv));
  vg.getRobotGraph()[r5_desc].setSpeedDirection(getConstrainedDirectedSpeed(vg.getRobotGraph()[r5_desc], vg, vv));

  boost::filesystem::create_directories("leaderAnimation2");
  printPlotWithArrows("leaderAnimation2/leaderAnimationTest2_0_90_1.png", "leaderAnimationTest", 0, 90, 1,
                      vg.getRobotGraph(), std::function(&overallPotential), vg.getRobotGraph(), vg.getObstacleGraph(),
                      v);

  for (int i = 2; i < 80; i++)
  {
    auto start = std::chrono::system_clock::now();
    vg.getRobotGraph()[r1_desc].updatePosition();
    vg.getRobotGraph()[r2_desc].updatePosition();
    vg.getRobotGraph()[r3_desc].updatePosition();
    vg.getRobotGraph()[r4_desc].updatePosition();
    vg.getRobotGraph()[r5_desc].updatePosition();
    vg.getRobotGraph()[r1_desc].setSpeedDirection(leaderV *
                                                  getConstrainedLeaderSpeed(vg.getRobotGraph()[r1_desc], vg, vv));
    vg.getRobotGraph()[r2_desc].setSpeedDirection(getConstrainedDirectedSpeed(vg.getRobotGraph()[r2_desc], vg, vv));
    vg.getRobotGraph()[r3_desc].setSpeedDirection(getConstrainedDirectedSpeed(vg.getRobotGraph()[r3_desc], vg, vv));
    vg.getRobotGraph()[r4_desc].setSpeedDirection(getConstrainedDirectedSpeed(vg.getRobotGraph()[r4_desc], vg, vv));
    vg.getRobotGraph()[r5_desc].setSpeedDirection(getConstrainedDirectedSpeed(vg.getRobotGraph()[r5_desc], vg, vv));
    vg.leavePreservedEdges(vv);
    auto end = std::chrono::system_clock::now();
    int elapsed_seconds = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();
    std::cout << "Iteration " << i << ": " << elapsed_seconds << std::endl;

    if (i % 5 == 0)
    {
      printPlotWithArrows("leaderAnimation2/leaderAnimationTest2_0_90_" + std::to_string(i) + ".png",
                          "leaderAnimationTest", 0, 90, 1, vg.getRobotGraph(), std::function(&overallPotential),
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

  Robot r1(Position_t(8.0, 8.0));
  Robot r2(Position_t(0.0, 0.0));
  Robot r3(Position_t(3.0, 5.0));
  Robot r4(Position_t(7.0, 4.0));
  Robot r5(Position_t(1.0, 8.0));
  Robot r6(Position_t(8.0, 0.0));

  auto rg = std::make_unique<RobotGraph>();
  auto r1_desc = boost::add_vertex(r1, *rg);
  auto r2_desc = boost::add_vertex(r2, *rg);
  auto r3_desc = boost::add_vertex(r3, *rg);
  auto r4_desc = boost::add_vertex(r4, *rg);
  auto r5_desc = boost::add_vertex(r5, *rg);
  auto r6_desc = boost::add_vertex(r6, *rg);

  auto og = std::make_unique<ObstacleGraph>();

  ValidatedGraphs vg(std::move(rg), std::move(og), vv);

  auto leaderV = Vector_t(sqrt(2), sqrt(2));
  vg.getRobotGraph()[r1_desc].setSpeedDirection(leaderV *
                                                getConstrainedLeaderSpeed(vg.getRobotGraph()[r1_desc], vg, vv));
  vg.getRobotGraph()[r2_desc].setSpeedDirection(getConstrainedDirectedSpeed(vg.getRobotGraph()[r2_desc], vg, vv));
  vg.getRobotGraph()[r3_desc].setSpeedDirection(getConstrainedDirectedSpeed(vg.getRobotGraph()[r3_desc], vg, vv));
  vg.getRobotGraph()[r4_desc].setSpeedDirection(getConstrainedDirectedSpeed(vg.getRobotGraph()[r4_desc], vg, vv));
  vg.getRobotGraph()[r5_desc].setSpeedDirection(getConstrainedDirectedSpeed(vg.getRobotGraph()[r5_desc], vg, vv));
  vg.getRobotGraph()[r6_desc].setSpeedDirection(getConstrainedDirectedSpeed(vg.getRobotGraph()[r6_desc], vg, vv));

  boost::filesystem::create_directories("leaderAnimation3");
  printPlotWithArrows("leaderAnimation3/leaderAnimationTest3_0_90_1.png", "leaderAnimationTest", 0, 90, 1,
                      vg.getRobotGraph(), std::function(&overallPotential), vg.getRobotGraph(), vg.getObstacleGraph(),
                      v);

  for (int i = 2; i < 80; i++)
  {
    auto start = std::chrono::system_clock::now();
    vg.getRobotGraph()[r1_desc].updatePosition();
    vg.getRobotGraph()[r2_desc].updatePosition();
    vg.getRobotGraph()[r3_desc].updatePosition();
    vg.getRobotGraph()[r4_desc].updatePosition();
    vg.getRobotGraph()[r5_desc].updatePosition();
    vg.getRobotGraph()[r6_desc].updatePosition();
    vg.getRobotGraph()[r1_desc].setSpeedDirection(leaderV *
                                                  getConstrainedLeaderSpeed(vg.getRobotGraph()[r1_desc], vg, vv));
    vg.getRobotGraph()[r2_desc].setSpeedDirection(getConstrainedDirectedSpeed(vg.getRobotGraph()[r2_desc], vg, vv));
    vg.getRobotGraph()[r3_desc].setSpeedDirection(getConstrainedDirectedSpeed(vg.getRobotGraph()[r3_desc], vg, vv));
    vg.getRobotGraph()[r4_desc].setSpeedDirection(getConstrainedDirectedSpeed(vg.getRobotGraph()[r4_desc], vg, vv));
    vg.getRobotGraph()[r5_desc].setSpeedDirection(getConstrainedDirectedSpeed(vg.getRobotGraph()[r5_desc], vg, vv));
    vg.getRobotGraph()[r6_desc].setSpeedDirection(getConstrainedDirectedSpeed(vg.getRobotGraph()[r6_desc], vg, vv));
    vg.leavePreservedEdges(vv);
    auto end = std::chrono::system_clock::now();
    int elapsed_seconds = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();
    std::cout << "Iteration " << i << ": " << elapsed_seconds << std::endl;

    if (i % 5 == 0)
    {
      printPlotWithArrows("leaderAnimation3/leaderAnimationTest3_0_90_" + std::to_string(i) + ".png",
                          "leaderAnimationTest", 0, 90, 1, vg.getRobotGraph(), std::function(&overallPotential),
                          vg.getRobotGraph(), vg.getObstacleGraph(), v);
    }
  }
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
