#include <gtest/gtest.h>
#include "../include/headers/matplotlibcpp.h"  // uses this library https://github.com/lava/matplotlib-cpp to draw plots

#include "../include/headers/classes.h"
#include "../include/headers/helper_functions.h"

TEST(closestDetectedObstacleTest, ShouldPass)
{
  Variables& v = Variables::getInstance();
  v.setParam("obstacle_care_distance", 3.0);
  v.setParam("obstacles_avoidance_distance", 1.5);
  v.setParam("small_positive_constant", 0.2);

  Vector_t v1, v2, v3;
  v1 << 10.0, 10.0;
  v2 << 5.0, 5.0;
  Obstacle o1(v1);
  Obstacle o2(v2);
  ObstacleGraph og;
  boost::add_vertex(o1, og);
  boost::add_vertex(o2, og);

  v3 << 10.0, 6.0;
  Robot r(v3);
  Obstacle o = closestDetectedObstacle(r, og, v).value();
  EXPECT_EQ(o1.getPosition(), o.getPosition());

  v3 << 0.0, 0.0;
  r.setPosition(v3);
  o = closestDetectedObstacle(r, og, v).value();
  EXPECT_EQ(o2.getPosition(), o.getPosition());

  v3 << 8.0, 8.0;
  r.setPosition(v3);
  o = closestDetectedObstacle(r, og, v).value();
  EXPECT_EQ(o1.getPosition(), o.getPosition());
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
