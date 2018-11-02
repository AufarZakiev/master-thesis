#include <gtest/gtest.h>
#include "../include/headers/DakaiAlgo.h"
#include "../include/headers/matplotlibcpp.h"  // uses this library https://github.com/lava/matplotlib-cpp to draw plots

TEST(partialInterrobotCollisionPotentialTest, ShouldPass)
{
  namespace plt = matplotlibcpp;

  Variables& v = Variables::getInstance();
  v.setParam("neighbourhood_distance", 1.0f);
  v.setParam("robots_avoidance_distance", 0.3f);
  v.setParam("desired_distance", 0.65f);
  double EQUALITY_CASE;
  v.getParam("equality_case", EQUALITY_CASE);
  // Prepare data.
  size_t n = 20000;
  std::vector<double> x(n), y(n);
  for (size_t i = 0; i < n; ++i)
  {
    x.at(i) = 0.3 + 0.7 * double(i) / (n - 1);
    y.at(i) = partialInterrobotCollisionPotential(0.3 + 0.7 * double(i) / (n - 1), v);
  }

  EXPECT_NEAR(y[0], 10, EQUALITY_CASE);
  EXPECT_NEAR(y[n / 2], 0, EQUALITY_CASE);
  EXPECT_NEAR(y[n - 1], 10, EQUALITY_CASE);

  // Set the size of output image = 1200x780 pixels
  plt::figure_size(1200, 780);
  // Plot line from given x and y data. Color is selected automatically.
  plt::named_plot("Phi Interrobot", x, y);
  // Set x-axis to interval [0,1000000]
  plt::xlim(0.25, 1.05);
  plt::ylim(-1, 12);
  // Enable legend.
  plt::legend();
  // Save the image (file format is determined by the extension)
  plt::save("./interrobot_potential.png");
}

TEST(partialObstacleCollisionPotentialTest, ShouldPass)
{
  namespace plt = matplotlibcpp;

  Variables& v = Variables::getInstance();
  v.setParam("obstacle_care_distance", 0.2f);
  v.setParam("obstacles_avoidance_distance", 0.1f);
  v.setParam("small_positive_constant", 0.1f);
  double EQUALITY_CASE;
  v.getParam("equality_case", EQUALITY_CASE);

  // Prepare data.
  size_t n = 20000;
  std::vector<double> x(n), y(n);
  for (size_t i = 0; i < n; ++i)
  {
    x.at(i) = 0.1 + 0.4 * double(i) / (n - 1);
    y.at(i) = partialObstacleCollisionPotential(0.1 + 0.4 * double(i) / (n - 1), v);
  }

  EXPECT_GT(y[0], 8);
  EXPECT_NEAR(y[n - 1], 0, EQUALITY_CASE);

  // Set the size of output image = 1200x780 pixels
  plt::figure_size(1200, 780);
  // Plot line from given x and y data. Color is selected automatically.
  plt::named_plot("Phi Obstacle", x, y);
  // Set x-axis to interval [0,1000000]
  plt::xlim(0.0, 0.5);
  plt::ylim(0, 10);
  // Enable legend.
  plt::legend();
  // Save the image (file format is determined by the extension)
  plt::save("./obstacle_potential.png");
}

TEST(partialLOSPreservePotentialTest, ShouldPass)
{
  namespace plt = matplotlibcpp;

  Variables& v = Variables::getInstance();
  v.setParam("los_clearance_care_distance", 0.2f);
  v.setParam("los_clearance_distance", 0.1f);
  v.setParam("small_positive_constant", 0.1f);
  double EQUALITY_CASE;
  v.getParam("equality_case", EQUALITY_CASE);

  // Prepare data.
  size_t n = 20000;
  std::vector<double> x(n), y(n);
  for (size_t i = 0; i < n; ++i)
  {
    x.at(i) = 0.1 + 0.4 * double(i) / (n - 1);
    y.at(i) = partialLOSPreservePotential(0.1 + 0.4 * double(i) / (n - 1), v);
  }

  EXPECT_GT(y[0], 8);
  EXPECT_NEAR(y[n - 1], 0, EQUALITY_CASE);

  // Set the size of output image = 1200x780 pixels
  plt::figure_size(1200, 780);
  // Plot line from given x and y data. Color is selected automatically.
  plt::named_plot("Phi LOS Clearance", x, y);
  // Set x-axis to interval [0,1000000]
  plt::xlim(0.0, 0.5);
  plt::ylim(0, 10);
  // Enable legend.
  plt::legend();
  // Save the image (file format is determined by the extension)
  plt::save("./los_clearance_potential.png");
}

TEST(partialCohesionPotentialTest, ShouldPass)
{
  namespace plt = matplotlibcpp;

  Variables& v = Variables::getInstance();
  v.setParam("neighbourhood_distance", 1.0f);
  double EQUALITY_CASE;
  v.getParam("equality_case", EQUALITY_CASE);

  // Prepare data.
  size_t n = 20000;
  std::vector<double> x(n), y(n);
  for (size_t i = 0; i < n; ++i)
  {
    x.at(i) = 0.1 + 4.9 * double(i) / (n - 1);
    y.at(i) = partialCohesionPotential(0.1 + 4.9 * double(i) / (n - 1), v);
  }

  EXPECT_GT(y[n - 1], 4);
  EXPECT_NEAR(y[0], 0, EQUALITY_CASE);

  // Set the size of output image = 1200x780 pixels
  plt::figure_size(1200, 780);
  // Plot line from given x and y data. Color is selected automatically.
  plt::named_plot("Phi Cohesion", x, y);
  // Set x-axis to interval [0,1000000]
  plt::xlim(0.0, 5.1);
  plt::ylim(0, 10);
  // Enable legend.
  plt::legend();
  // Save the image (file format is determined by the extension)
  plt::save("./cohesion_potential.png");
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
