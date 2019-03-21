#include <gtest/gtest.h>
#include "../include/headers/speed_constraints.h"
#include "../include/headers/field_functions.h"
#include <boost/filesystem.hpp>

TEST(animatedCohesionTest, ShouldPass)
{
  Variables v = Variables();
  v.setParam("robots_avoidance_distance", 1.5);
  v.setParam("obstacles_avoidance_distance", 1.0);
  v.setParam("los_clearance_distance", 0.2);
  v.setParam("los_clearance_care_distance", 0.4);
  v.setParam("neighbourhood_distance", 3.0);
  v.setParam("edge_deletion_distance", 0.1);
  v.setParam("obstacle_care_distance", 3.0);
  v.setParam("desired_distance", 2.0);
  v.setParam("sensing_distance", 10.0);
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

  ValidatedGraphs vg(std::move(rg), std::move(og), v);

  vg.getRobotGraph()[r1_desc].setSpeedDirection(getConstrainedDirectedSpeed(vg.getRobotGraph()[r1_desc], vg, vv));
  vg.getRobotGraph()[r2_desc].setSpeedDirection(getConstrainedDirectedSpeed(vg.getRobotGraph()[r2_desc], vg, vv));
  vg.getRobotGraph()[r3_desc].setSpeedDirection(getConstrainedDirectedSpeed(vg.getRobotGraph()[r3_desc], vg, vv));

  boost::filesystem::create_directories("cohesionAnimation");
  printPlotWithArrows("cohesionAnimation/cohesionAnimationTest_0_90_1.png", "cohesionAnimationTest", 0, 90, 1,
                      vg.getRobotGraph(), std::function(&overallPotential), vg.getRobotGraph(), vg.getObstacleGraph(),
                      v);

  vg.getRobotGraph()[r1_desc].updatePosition();
  vg.getRobotGraph()[r2_desc].updatePosition();
  vg.getRobotGraph()[r3_desc].updatePosition();
  vg.getRobotGraph()[r1_desc].setSpeedDirection(getConstrainedDirectedSpeed(vg.getRobotGraph()[r1_desc], vg, vv));
  vg.getRobotGraph()[r2_desc].setSpeedDirection(getConstrainedDirectedSpeed(vg.getRobotGraph()[r2_desc], vg, vv));
  vg.getRobotGraph()[r3_desc].setSpeedDirection(getConstrainedDirectedSpeed(vg.getRobotGraph()[r3_desc], vg, vv));

  printPlotWithArrows("cohesionAnimation/cohesionAnimationTest_0_90_2.png", "cohesionAnimationTest", 0, 90, 1,
                      vg.getRobotGraph(), std::function(&overallPotential), vg.getRobotGraph(), vg.getObstacleGraph(),
                      v);

  vg.getRobotGraph()[r1_desc].updatePosition();
  vg.getRobotGraph()[r2_desc].updatePosition();
  vg.getRobotGraph()[r3_desc].updatePosition();
  vg.getRobotGraph()[r1_desc].setSpeedDirection(getConstrainedDirectedSpeed(vg.getRobotGraph()[r1_desc], vg, vv));
  vg.getRobotGraph()[r2_desc].setSpeedDirection(getConstrainedDirectedSpeed(vg.getRobotGraph()[r2_desc], vg, vv));
  vg.getRobotGraph()[r3_desc].setSpeedDirection(getConstrainedDirectedSpeed(vg.getRobotGraph()[r3_desc], vg, vv));

  printPlotWithArrows("cohesionAnimation/cohesionAnimationTest_0_90_3.png", "cohesionAnimationTest", 0, 90, 1,
                      vg.getRobotGraph(), std::function(&overallPotential), vg.getRobotGraph(), vg.getObstacleGraph(),
                      v);

  vg.getRobotGraph()[r1_desc].updatePosition();
  vg.getRobotGraph()[r2_desc].updatePosition();
  vg.getRobotGraph()[r3_desc].updatePosition();
  vg.getRobotGraph()[r1_desc].setSpeedDirection(getConstrainedDirectedSpeed(vg.getRobotGraph()[r1_desc], vg, vv));
  vg.getRobotGraph()[r2_desc].setSpeedDirection(getConstrainedDirectedSpeed(vg.getRobotGraph()[r2_desc], vg, vv));
  vg.getRobotGraph()[r3_desc].setSpeedDirection(getConstrainedDirectedSpeed(vg.getRobotGraph()[r3_desc], vg, vv));

  printPlotWithArrows("cohesionAnimation/cohesionAnimationTest_0_90_4.png", "cohesionAnimationTest", 0, 90, 1,
                      vg.getRobotGraph(), std::function(&overallPotential), vg.getRobotGraph(), vg.getObstacleGraph(),
                      v);

  vg.getRobotGraph()[r1_desc].updatePosition();
  vg.getRobotGraph()[r2_desc].updatePosition();
  vg.getRobotGraph()[r3_desc].updatePosition();
  vg.getRobotGraph()[r1_desc].setSpeedDirection(getConstrainedDirectedSpeed(vg.getRobotGraph()[r1_desc], vg, vv));
  vg.getRobotGraph()[r2_desc].setSpeedDirection(getConstrainedDirectedSpeed(vg.getRobotGraph()[r2_desc], vg, vv));
  vg.getRobotGraph()[r3_desc].setSpeedDirection(getConstrainedDirectedSpeed(vg.getRobotGraph()[r3_desc], vg, vv));

  printPlotWithArrows("cohesionAnimation/cohesionAnimationTest_0_90_5.png", "cohesionAnimationTest", 0, 90, 1,
                      vg.getRobotGraph(), std::function(&overallPotential), vg.getRobotGraph(), vg.getObstacleGraph(),
                      v);
}

TEST(animatedLeaderTest, ShouldPass)
{
  Variables v = Variables();
  v.setParam("robots_avoidance_distance", 1.0);
  v.setParam("obstacles_avoidance_distance", 1.0);
  v.setParam("los_clearance_distance", 0.2);
  v.setParam("los_clearance_care_distance", 0.4);
  v.setParam("neighbourhood_distance", 2.0);
  v.setParam("edge_deletion_distance", 0.1);
  v.setParam("obstacle_care_distance", 3.0);
  v.setParam("desired_distance", 1.5);
  v.setParam("sensing_distance", 10.0);
  v.setParam("robot_max_speed", 0.1);
  v.setParam("k1", 10);
  v.setParam("k2", 10);
  v.setParam("c1", 0.0);
  v.setParam("c2", 0.0);
  v.setParam("c3", 0.0);
  v.setParam("c4", 30.0);

  ValidatedVariables vv(v);

  Robot r1(Vector_t(5.0, 3.0));
  Robot r2(Vector_t(5.0, 10.0));
  Robot r3(Vector_t(0.0, 3.0));
  auto rg = std::make_unique<RobotGraph>();
  auto r1_desc = boost::add_vertex(r1, *rg);
  auto r2_desc = boost::add_vertex(r2, *rg);
  auto r3_desc = boost::add_vertex(r3, *rg);
  boost::add_edge(r1_desc, r2_desc, *rg);
  boost::add_edge(r2_desc, r3_desc, *rg);

  auto og = std::make_unique<ObstacleGraph>();

  ValidatedGraphs vg(std::move(rg), std::move(og), v);

  auto leaderV = Vector_t(0.5, 0.5);
  vg.getRobotGraph()[r1_desc].setSpeedDirection(leaderV);
  vg.getRobotGraph()[r2_desc].setSpeedDirection(getConstrainedDirectedSpeed(vg.getRobotGraph()[r2_desc], vg, vv));
  vg.getRobotGraph()[r3_desc].setSpeedDirection(getConstrainedDirectedSpeed(vg.getRobotGraph()[r3_desc], vg, vv));

  boost::filesystem::create_directories("leaderAnimation");
  printPlotWithArrows("leaderAnimation/leaderAnimationTest_0_90_1.png", "leaderAnimationTest", 0, 90, 1,
                      vg.getRobotGraph(), std::function(&overallPotential), vg.getRobotGraph(), vg.getObstacleGraph(),
                      v);

  vg.getRobotGraph()[r1_desc].updatePosition();
  vg.getRobotGraph()[r2_desc].updatePosition();
  vg.getRobotGraph()[r3_desc].updatePosition();
  vg.getRobotGraph()[r2_desc].setSpeedDirection(getConstrainedDirectedSpeed(vg.getRobotGraph()[r2_desc], vg, vv));
  vg.getRobotGraph()[r3_desc].setSpeedDirection(getConstrainedDirectedSpeed(vg.getRobotGraph()[r3_desc], vg, vv));

  printPlotWithArrows("leaderAnimation/leaderAnimationTest_0_90_2.png", "leaderAnimationTest", 0, 90, 1,
                      vg.getRobotGraph(), std::function(&overallPotential), vg.getRobotGraph(), vg.getObstacleGraph(),
                      v);

  vg.getRobotGraph()[r1_desc].updatePosition();
  vg.getRobotGraph()[r2_desc].updatePosition();
  vg.getRobotGraph()[r3_desc].updatePosition();
  vg.getRobotGraph()[r2_desc].setSpeedDirection(getConstrainedDirectedSpeed(vg.getRobotGraph()[r2_desc], vg, vv));
  vg.getRobotGraph()[r3_desc].setSpeedDirection(getConstrainedDirectedSpeed(vg.getRobotGraph()[r3_desc], vg, vv));

  printPlotWithArrows("leaderAnimation/leaderAnimationTest_0_90_3.png", "leaderAnimationTest", 0, 90, 1,
                      vg.getRobotGraph(), std::function(&overallPotential), vg.getRobotGraph(), vg.getObstacleGraph(),
                      v);

  vg.getRobotGraph()[r1_desc].updatePosition();
  vg.getRobotGraph()[r2_desc].updatePosition();
  vg.getRobotGraph()[r3_desc].updatePosition();
  vg.getRobotGraph()[r2_desc].setSpeedDirection(getConstrainedDirectedSpeed(vg.getRobotGraph()[r2_desc], vg, vv));
  vg.getRobotGraph()[r3_desc].setSpeedDirection(getConstrainedDirectedSpeed(vg.getRobotGraph()[r3_desc], vg, vv));

  printPlotWithArrows("leaderAnimation/leaderAnimationTest_0_90_4.png", "leaderAnimationTest", 0, 90, 1,
                      vg.getRobotGraph(), std::function(&overallPotential), vg.getRobotGraph(), vg.getObstacleGraph(),
                      v);

  vg.getRobotGraph()[r1_desc].updatePosition();
  vg.getRobotGraph()[r2_desc].updatePosition();
  vg.getRobotGraph()[r3_desc].updatePosition();
  vg.getRobotGraph()[r2_desc].setSpeedDirection(getConstrainedDirectedSpeed(vg.getRobotGraph()[r2_desc], vg, vv));
  vg.getRobotGraph()[r3_desc].setSpeedDirection(getConstrainedDirectedSpeed(vg.getRobotGraph()[r3_desc], vg, vv));

  printPlotWithArrows("leaderAnimation/leaderAnimationTest_0_90_5.png", "leaderAnimationTest", 0, 90, 1,
                      vg.getRobotGraph(), std::function(&overallPotential), vg.getRobotGraph(), vg.getObstacleGraph(),
                      v);

  vg.getRobotGraph()[r1_desc].updatePosition();
  vg.getRobotGraph()[r2_desc].updatePosition();
  vg.getRobotGraph()[r3_desc].updatePosition();
  vg.getRobotGraph()[r2_desc].setSpeedDirection(getConstrainedDirectedSpeed(vg.getRobotGraph()[r2_desc], vg, vv));
  vg.getRobotGraph()[r3_desc].setSpeedDirection(getConstrainedDirectedSpeed(vg.getRobotGraph()[r3_desc], vg, vv));

  printPlotWithArrows("leaderAnimation/leaderAnimationTest_0_90_6.png", "leaderAnimationTest", 0, 90, 1,
                      vg.getRobotGraph(), std::function(&overallPotential), vg.getRobotGraph(), vg.getObstacleGraph(),
                      v);

  vg.getRobotGraph()[r1_desc].updatePosition();
  vg.getRobotGraph()[r2_desc].updatePosition();
  vg.getRobotGraph()[r3_desc].updatePosition();
  vg.getRobotGraph()[r2_desc].setSpeedDirection(getConstrainedDirectedSpeed(vg.getRobotGraph()[r2_desc], vg, vv));
  vg.getRobotGraph()[r3_desc].setSpeedDirection(getConstrainedDirectedSpeed(vg.getRobotGraph()[r3_desc], vg, vv));

  printPlotWithArrows("leaderAnimation/leaderAnimationTest_0_90_7.png", "leaderAnimationTest", 0, 90, 1,
                      vg.getRobotGraph(), std::function(&overallPotential), vg.getRobotGraph(), vg.getObstacleGraph(),
                      v);

  vg.getRobotGraph()[r1_desc].updatePosition();
  vg.getRobotGraph()[r2_desc].updatePosition();
  vg.getRobotGraph()[r3_desc].updatePosition();
  vg.getRobotGraph()[r2_desc].setSpeedDirection(getConstrainedDirectedSpeed(vg.getRobotGraph()[r2_desc], vg, vv));
  vg.getRobotGraph()[r3_desc].setSpeedDirection(getConstrainedDirectedSpeed(vg.getRobotGraph()[r3_desc], vg, vv));

  printPlotWithArrows("leaderAnimation/leaderAnimationTest_0_90_8.png", "leaderAnimationTest", 0, 90, 1,
                      vg.getRobotGraph(), std::function(&overallPotential), vg.getRobotGraph(), vg.getObstacleGraph(),
                      v);

  vg.getRobotGraph()[r1_desc].updatePosition();
  vg.getRobotGraph()[r2_desc].updatePosition();
  vg.getRobotGraph()[r3_desc].updatePosition();
  vg.getRobotGraph()[r2_desc].setSpeedDirection(getConstrainedDirectedSpeed(vg.getRobotGraph()[r2_desc], vg, vv));
  vg.getRobotGraph()[r3_desc].setSpeedDirection(getConstrainedDirectedSpeed(vg.getRobotGraph()[r3_desc], vg, vv));

  printPlotWithArrows("leaderAnimation/leaderAnimationTest_0_90_9.png", "leaderAnimationTest", 0, 90, 1,
                      vg.getRobotGraph(), std::function(&overallPotential), vg.getRobotGraph(), vg.getObstacleGraph(),
                      v);

  vg.getRobotGraph()[r1_desc].updatePosition();
  vg.getRobotGraph()[r2_desc].updatePosition();
  vg.getRobotGraph()[r3_desc].updatePosition();
  vg.getRobotGraph()[r2_desc].setSpeedDirection(getConstrainedDirectedSpeed(vg.getRobotGraph()[r2_desc], vg, vv));
  vg.getRobotGraph()[r3_desc].setSpeedDirection(getConstrainedDirectedSpeed(vg.getRobotGraph()[r3_desc], vg, vv));

  printPlotWithArrows("leaderAnimation/leaderAnimationTest_0_90_10.png", "leaderAnimationTest", 0, 90, 1,
                      vg.getRobotGraph(), std::function(&overallPotential), vg.getRobotGraph(), vg.getObstacleGraph(),
                      v);

  vg.getRobotGraph()[r1_desc].updatePosition();
  vg.getRobotGraph()[r2_desc].updatePosition();
  vg.getRobotGraph()[r3_desc].updatePosition();
  vg.getRobotGraph()[r2_desc].setSpeedDirection(getConstrainedDirectedSpeed(vg.getRobotGraph()[r2_desc], vg, vv));
  vg.getRobotGraph()[r3_desc].setSpeedDirection(getConstrainedDirectedSpeed(vg.getRobotGraph()[r3_desc], vg, vv));

  printPlotWithArrows("leaderAnimation/leaderAnimationTest_0_90_11.png", "leaderAnimationTest", 0, 90, 1,
                      vg.getRobotGraph(), std::function(&overallPotential), vg.getRobotGraph(), vg.getObstacleGraph(),
                      v);

  vg.getRobotGraph()[r1_desc].updatePosition();
  vg.getRobotGraph()[r2_desc].updatePosition();
  vg.getRobotGraph()[r3_desc].updatePosition();
  vg.getRobotGraph()[r2_desc].setSpeedDirection(getConstrainedDirectedSpeed(vg.getRobotGraph()[r2_desc], vg, vv));
  vg.getRobotGraph()[r3_desc].setSpeedDirection(getConstrainedDirectedSpeed(vg.getRobotGraph()[r3_desc], vg, vv));

  printPlotWithArrows("leaderAnimation/leaderAnimationTest_0_90_12.png", "leaderAnimationTest", 0, 90, 1,
                      vg.getRobotGraph(), std::function(&overallPotential), vg.getRobotGraph(), vg.getObstacleGraph(),
                      v);
}

TEST(animatedLeaderTest2, ShouldPass)
{
  Variables v = Variables();
  v.setParam("robots_avoidance_distance", 1.0);
  v.setParam("obstacles_avoidance_distance", 1.0);
  v.setParam("los_clearance_distance", 0.2);
  v.setParam("los_clearance_care_distance", 0.4);
  v.setParam("neighbourhood_distance", 2.0);
  v.setParam("edge_deletion_distance", 0.1);
  v.setParam("obstacle_care_distance", 3.0);
  v.setParam("desired_distance", 1.5);
  v.setParam("sensing_distance", 10.0);
  v.setParam("robot_max_speed", 0.1);
  v.setParam("k1", 10);
  v.setParam("k2", 10);
  v.setParam("c1", 10.0);
  v.setParam("c2", 1.0);
  v.setParam("c3", 5.0);
  v.setParam("c4", 10.0);

  ValidatedVariables vv(v);

  Robot r1(Vector_t(5.0, 3.0));
  Robot r2(Vector_t(5.0, 10.0));
  Robot r3(Vector_t(0.0, 3.0));
  auto rg = std::make_unique<RobotGraph>();
  auto r1_desc = boost::add_vertex(r1, *rg);
  auto r2_desc = boost::add_vertex(r2, *rg);
  auto r3_desc = boost::add_vertex(r3, *rg);
  boost::add_edge(r1_desc, r2_desc, *rg);
  boost::add_edge(r2_desc, r3_desc, *rg);

  auto og = std::make_unique<ObstacleGraph>();

  ValidatedGraphs vg(std::move(rg), std::move(og), v);

  auto leaderV = Vector_t(0.3, 0.3);
  vg.getRobotGraph()[r1_desc].setSpeedDirection(leaderV);
  vg.getRobotGraph()[r2_desc].setSpeedDirection(0.9*getConstrainedDirectedSpeed(vg.getRobotGraph()[r2_desc], vg, vv));
  vg.getRobotGraph()[r3_desc].setSpeedDirection(0.9*getConstrainedDirectedSpeed(vg.getRobotGraph()[r3_desc], vg, vv));

  boost::filesystem::create_directories("leaderAnimation2");
  printPlotWithArrows("leaderAnimation2/leaderAnimationTest2_0_90_1.png", "leaderAnimationTest", 0, 90, 1,
                      vg.getRobotGraph(), std::function(&overallPotential), vg.getRobotGraph(), vg.getObstacleGraph(),
                      v);

  vg.getRobotGraph()[r1_desc].updatePosition();
  vg.getRobotGraph()[r2_desc].updatePosition();
  vg.getRobotGraph()[r3_desc].updatePosition();
  vg.getRobotGraph()[r2_desc].setSpeedDirection(0.9*getConstrainedDirectedSpeed(vg.getRobotGraph()[r2_desc], vg, vv));
  vg.getRobotGraph()[r3_desc].setSpeedDirection(0.9*getConstrainedDirectedSpeed(vg.getRobotGraph()[r3_desc], vg, vv));

  printPlotWithArrows("leaderAnimation2/leaderAnimationTest2_0_90_2.png", "leaderAnimationTest", 0, 90, 1,
                      vg.getRobotGraph(), std::function(&overallPotential), vg.getRobotGraph(), vg.getObstacleGraph(),
                      v);

  vg.getRobotGraph()[r1_desc].updatePosition();
  vg.getRobotGraph()[r2_desc].updatePosition();
  vg.getRobotGraph()[r3_desc].updatePosition();
  vg.getRobotGraph()[r2_desc].setSpeedDirection(0.9*getConstrainedDirectedSpeed(vg.getRobotGraph()[r2_desc], vg, vv));
  vg.getRobotGraph()[r3_desc].setSpeedDirection(0.9*getConstrainedDirectedSpeed(vg.getRobotGraph()[r3_desc], vg, vv));

  printPlotWithArrows("leaderAnimation2/leaderAnimationTest2_0_90_3.png", "leaderAnimationTest", 0, 90, 1,
                      vg.getRobotGraph(), std::function(&overallPotential), vg.getRobotGraph(), vg.getObstacleGraph(),
                      v);

  vg.getRobotGraph()[r1_desc].updatePosition();
  vg.getRobotGraph()[r2_desc].updatePosition();
  vg.getRobotGraph()[r3_desc].updatePosition();
  vg.getRobotGraph()[r2_desc].setSpeedDirection(0.9*getConstrainedDirectedSpeed(vg.getRobotGraph()[r2_desc], vg, vv));
  vg.getRobotGraph()[r3_desc].setSpeedDirection(0.9*getConstrainedDirectedSpeed(vg.getRobotGraph()[r3_desc], vg, vv));

  printPlotWithArrows("leaderAnimation2/leaderAnimationTest2_0_90_4.png", "leaderAnimationTest", 0, 90, 1,
                      vg.getRobotGraph(), std::function(&overallPotential), vg.getRobotGraph(), vg.getObstacleGraph(),
                      v);

  vg.getRobotGraph()[r1_desc].updatePosition();
  vg.getRobotGraph()[r2_desc].updatePosition();
  vg.getRobotGraph()[r3_desc].updatePosition();
  vg.getRobotGraph()[r2_desc].setSpeedDirection(0.9*getConstrainedDirectedSpeed(vg.getRobotGraph()[r2_desc], vg, vv));
  vg.getRobotGraph()[r3_desc].setSpeedDirection(0.9*getConstrainedDirectedSpeed(vg.getRobotGraph()[r3_desc], vg, vv));

  printPlotWithArrows("leaderAnimation2/leaderAnimationTest2_0_90_5.png", "leaderAnimationTest", 0, 90, 1,
                      vg.getRobotGraph(), std::function(&overallPotential), vg.getRobotGraph(), vg.getObstacleGraph(),
                      v);

  vg.getRobotGraph()[r1_desc].updatePosition();
  vg.getRobotGraph()[r2_desc].updatePosition();
  vg.getRobotGraph()[r3_desc].updatePosition();
  vg.getRobotGraph()[r2_desc].setSpeedDirection(0.9*getConstrainedDirectedSpeed(vg.getRobotGraph()[r2_desc], vg, vv));
  vg.getRobotGraph()[r3_desc].setSpeedDirection(0.9*getConstrainedDirectedSpeed(vg.getRobotGraph()[r3_desc], vg, vv));

  printPlotWithArrows("leaderAnimation2/leaderAnimationTest2_0_90_6.png", "leaderAnimationTest", 0, 90, 1,
                      vg.getRobotGraph(), std::function(&overallPotential), vg.getRobotGraph(), vg.getObstacleGraph(),
                      v);

  vg.getRobotGraph()[r1_desc].updatePosition();
  vg.getRobotGraph()[r2_desc].updatePosition();
  vg.getRobotGraph()[r3_desc].updatePosition();
  vg.getRobotGraph()[r2_desc].setSpeedDirection(0.9*getConstrainedDirectedSpeed(vg.getRobotGraph()[r2_desc], vg, vv));
  vg.getRobotGraph()[r3_desc].setSpeedDirection(0.9*getConstrainedDirectedSpeed(vg.getRobotGraph()[r3_desc], vg, vv));

  printPlotWithArrows("leaderAnimation2/leaderAnimationTest2_0_90_7.png", "leaderAnimationTest", 0, 90, 1,
                      vg.getRobotGraph(), std::function(&overallPotential), vg.getRobotGraph(), vg.getObstacleGraph(),
                      v);

  vg.getRobotGraph()[r1_desc].updatePosition();
  vg.getRobotGraph()[r2_desc].updatePosition();
  vg.getRobotGraph()[r3_desc].updatePosition();
  vg.getRobotGraph()[r2_desc].setSpeedDirection(0.9*getConstrainedDirectedSpeed(vg.getRobotGraph()[r2_desc], vg, vv));
  vg.getRobotGraph()[r3_desc].setSpeedDirection(0.9*getConstrainedDirectedSpeed(vg.getRobotGraph()[r3_desc], vg, vv));

  printPlotWithArrows("leaderAnimation2/leaderAnimationTest2_0_90_8.png", "leaderAnimationTest", 0, 90, 1,
                      vg.getRobotGraph(), std::function(&overallPotential), vg.getRobotGraph(), vg.getObstacleGraph(),
                      v);

  vg.getRobotGraph()[r1_desc].updatePosition();
  vg.getRobotGraph()[r2_desc].updatePosition();
  vg.getRobotGraph()[r3_desc].updatePosition();
  vg.getRobotGraph()[r2_desc].setSpeedDirection(0.9*getConstrainedDirectedSpeed(vg.getRobotGraph()[r2_desc], vg, vv));
  vg.getRobotGraph()[r3_desc].setSpeedDirection(0.9*getConstrainedDirectedSpeed(vg.getRobotGraph()[r3_desc], vg, vv));

  printPlotWithArrows("leaderAnimation2/leaderAnimationTest2_0_90_9.png", "leaderAnimationTest", 0, 90, 1,
                      vg.getRobotGraph(), std::function(&overallPotential), vg.getRobotGraph(), vg.getObstacleGraph(),
                      v);

  vg.getRobotGraph()[r1_desc].updatePosition();
  vg.getRobotGraph()[r2_desc].updatePosition();
  vg.getRobotGraph()[r3_desc].updatePosition();
  vg.getRobotGraph()[r2_desc].setSpeedDirection(0.9*getConstrainedDirectedSpeed(vg.getRobotGraph()[r2_desc], vg, vv));
  vg.getRobotGraph()[r3_desc].setSpeedDirection(0.9*getConstrainedDirectedSpeed(vg.getRobotGraph()[r3_desc], vg, vv));

  printPlotWithArrows("leaderAnimation2/leaderAnimationTest2_0_90_10.png", "leaderAnimationTest", 0, 90, 1,
                      vg.getRobotGraph(), std::function(&overallPotential), vg.getRobotGraph(), vg.getObstacleGraph(),
                      v);

  vg.getRobotGraph()[r1_desc].updatePosition();
  vg.getRobotGraph()[r2_desc].updatePosition();
  vg.getRobotGraph()[r3_desc].updatePosition();
  vg.getRobotGraph()[r2_desc].setSpeedDirection(0.9*getConstrainedDirectedSpeed(vg.getRobotGraph()[r2_desc], vg, vv));
  vg.getRobotGraph()[r3_desc].setSpeedDirection(0.9*getConstrainedDirectedSpeed(vg.getRobotGraph()[r3_desc], vg, vv));

  printPlotWithArrows("leaderAnimation2/leaderAnimationTest2_0_90_11.png", "leaderAnimationTest", 0, 90, 1,
                      vg.getRobotGraph(), std::function(&overallPotential), vg.getRobotGraph(), vg.getObstacleGraph(),
                      v);

  vg.getRobotGraph()[r1_desc].updatePosition();
  vg.getRobotGraph()[r2_desc].updatePosition();
  vg.getRobotGraph()[r3_desc].updatePosition();
  vg.getRobotGraph()[r2_desc].setSpeedDirection(0.9*getConstrainedDirectedSpeed(vg.getRobotGraph()[r2_desc], vg, vv));
  vg.getRobotGraph()[r3_desc].setSpeedDirection(0.9*getConstrainedDirectedSpeed(vg.getRobotGraph()[r3_desc], vg, vv));

  printPlotWithArrows("leaderAnimation2/leaderAnimationTest2_0_90_12.png", "leaderAnimationTest", 0, 90, 1,
                      vg.getRobotGraph(), std::function(&overallPotential), vg.getRobotGraph(), vg.getObstacleGraph(),
                      v);
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
