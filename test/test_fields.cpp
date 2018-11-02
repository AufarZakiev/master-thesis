#include <gtest/gtest.h>
#include "../include/headers/DakaiAlgo.h"

#include "../include/gnuplot-iostream/gnuplot-iostream.h"

TEST(CohesionPotentialTest_4_robots, ShouldPass)
{
  Variables& v = Variables::getInstance();
  v.setParam("neighbourhood_distance", 1.0f);

  Vector_t v1, v2,v3, v4;
  v1 << 5, 5;
  v2 << 10, 10;
  v3 << 10, 5;
  v4 << 5, 10;
  Robot r1(v1);
  Robot r2(v2);
  Robot r3(v3);
  Robot r4(v4);
  RigidGraph rg;
  boost::add_vertex(r1, rg);
  boost::add_vertex(r2, rg);
  boost::add_vertex(r3, rg);
  boost::add_vertex(r4, rg);

  std::vector<std::vector<std::tuple<double, double, double>>> frame(60);
  for (int i = 0; i < 60; i++)
  {
    frame[i].resize(60);
    for (int j = 0; j < 60; j++)
    {
      Vector_t temp;
      temp << i/4.0, j/4.0;
      RigidObject point(temp);
      frame[i][j] = std::make_tuple(temp(0,0),temp(1,0), cohesionPotential(point, rg, v));
    }
  }

  Gnuplot gp;
  gp << "set term png\n";
  gp << "set output \"Cohesion_field_4_robots_uniform.png\"\n";
  gp << "set view 0, 90, 1, 1\n";
  gp << "set samples 51, 51\n";
  gp << "set style data lines\n";
  gp << "set pm3d\n";
  gp << "set title \"Cohesion field\"\n";
  gp << "set xlabel \"X axis\"\n";
  gp << "set xlabel  offset character -3, -2, 0 font \"\" textcolor lt -1 norotate\n";
  gp << "set xrange [ 0.00000 : 15.00000 ] noreverse nowriteback\n";
  gp << "set ylabel \"Y axis\"\n";
  gp << "set ylabel  offset character 3, -2, 0 font \"\" textcolor lt -1 rotate\n";
  gp << "set yrange [ 0.00000 : 15.00000 ] noreverse nowriteback\n";
  gp << "set zlabel \"Z axis\"\n";
  gp << "set zlabel  offset character -5, 0, 0 font \"\" textcolor lt -1 norotate\n";
  gp << "set zrange [ -0.50000 : 450.00000 ] noreverse nowriteback\n";
  gp << "splot [0:15] [0:15] '-' \n";
  gp.send2d(frame);
  gp.flush();
}

TEST(CohesionPotentialTest_2_robots_offset, ShouldPass)
{
  Variables& v = Variables::getInstance();
  v.setParam("neighbourhood_distance", 1.0f);

  Vector_t v1, v2;
  v1 << 0, 0;
  v2 << 10, 10;
  Robot r1(v1);
  Robot r2(v2);
  RigidGraph rg;
  boost::add_vertex(r1, rg);
  boost::add_vertex(r2, rg);

  std::vector<std::vector<std::tuple<double, double, double>>> frame(60);
  for (int i = 0; i < 60; i++)
  {
    frame[i].resize(60);
    for (int j = 0; j < 60; j++)
    {
      Vector_t temp;
      temp << i/4.0, j/4.0;
      RigidObject point(temp);
      frame[i][j] = std::make_tuple(temp(0,0),temp(1,0), cohesionPotential(point, rg, v));
    }
  }

  Gnuplot gp;
  gp << "set term png\n";
  gp << "set output \"Cohesion_field_2_robots_offset.png\"\n";
  gp << "set view 0, 90, 1, 1\n";
  gp << "set samples 51, 51\n";
  gp << "set style data lines\n";
  gp << "set pm3d\n";
  gp << "set title \"Cohesion field\"\n";
  gp << "set xlabel \"X axis\"\n";
  gp << "set xlabel  offset character -3, -2, 0 font \"\" textcolor lt -1 norotate\n";
  gp << "set xrange [ 0.00000 : 15.00000 ] noreverse nowriteback\n";
  gp << "set ylabel \"Y axis\"\n";
  gp << "set ylabel  offset character 3, -2, 0 font \"\" textcolor lt -1 rotate\n";
  gp << "set yrange [ 0.00000 : 15.00000 ] noreverse nowriteback\n";
  gp << "set zlabel \"Z axis\"\n";
  gp << "set zlabel  offset character -5, 0, 0 font \"\" textcolor lt -1 norotate\n";
  gp << "set zrange [ -0.50000 : 450.00000 ] noreverse nowriteback\n";
  gp << "splot [0:15] [0:15] '-' \n";
  gp.send2d(frame);
  gp.flush();
}

TEST(CohesionPotentialTest_2_robots_cohesive, ShouldPass)
{
  Variables& v = Variables::getInstance();
  v.setParam("neighbourhood_distance", 20.0f);

  Vector_t v1, v2;
  v1 << 0, 0;
  v2 << 10, 10;
  Robot r1(v1);
  Robot r2(v2);
  RigidGraph rg;
  boost::add_vertex(r1, rg);
  boost::add_vertex(r2, rg);

  std::vector<std::vector<std::tuple<double, double, double>>> frame(60);
  for (int i = 0; i < 60; i++)
  {
    frame[i].resize(60);
    for (int j = 0; j < 60; j++)
    {
      Vector_t temp;
      temp << i/4.0, j/4.0;
      RigidObject point(temp);
      frame[i][j] = std::make_tuple(temp(0,0),temp(1,0), cohesionPotential(point, rg, v));
    }
  }

  Gnuplot gp;
  gp << "set term png\n";
  gp << "set output \"Cohesion_field_2_robots_cohesive.png\"\n";
  gp << "set view 60, 30, 1, 1\n";
  gp << "set samples 51, 51\n";
  gp << "set style data lines\n";
  gp << "set pm3d\n";
  gp << "set title \"Cohesion field\"\n";
  gp << "set xlabel \"X axis\"\n";
  gp << "set xlabel  offset character -3, -2, 0 font \"\" textcolor lt -1 norotate\n";
  gp << "set xrange [ 0.00000 : 15.00000 ] noreverse nowriteback\n";
  gp << "set ylabel \"Y axis\"\n";
  gp << "set ylabel  offset character 3, -2, 0 font \"\" textcolor lt -1 rotate\n";
  gp << "set yrange [ 0.00000 : 15.00000 ] noreverse nowriteback\n";
  gp << "set zlabel \"Z axis\"\n";
  gp << "set zlabel  offset character -5, 0, 0 font \"\" textcolor lt -1 norotate\n";
  gp << "set zrange [ -0.50000 : 450.00000 ] noreverse nowriteback\n";
  gp << "splot [0:15] [0:15] '-' \n";
  gp.send2d(frame);
  gp.flush();
}

TEST(InterrobotPotentialTest_2_robots, ShouldPass)
{
  Variables& v = Variables::getInstance();
  v.setParam("robots_avoidance_distance", 1.0);
  v.setParam("desired_distance", 3.0);
  v.setParam("neighbourhood_distance", 4.0);
  v.setParam("k1", 10);
  v.setParam("k2", 10);

  Vector_t v1, v2;
  v1 << 5.5, 5.5;
  v2 << 9.5, 9.5;
  Robot r1(v1);
  Robot r2(v2);
  RigidGraph rg;
  boost::add_vertex(r1, rg);
  boost::add_vertex(r2, rg);

  std::vector<std::vector<std::tuple<double, double, double>>> frame(60);
  for (int i = 0; i < 60; i++)
  {
    frame[i].resize(60);
    for (int j = 0; j < 60; j++)
    {
      Vector_t temp;
      temp << i/4.0, j/4.0;
      RigidObject point(temp);
      frame[i][j] = std::make_tuple(temp(0,0),temp(1,0), interrobotCollisionPotential(point, rg, v));
    }
  }

  Gnuplot gp;
  gp << "set term png\n";
  gp << "set output \"Interrobot_field_2_robots.png\"\n";
  gp << "set view 60, 30, 1, 1\n";
  gp << "set samples 51, 51\n";
  gp << "set style data lines\n";
  gp << "set pm3d\n";
  gp << "set title \"Interrobot collision field\"\n";
  gp << "set xlabel \"X axis\"\n";
  gp << "set xlabel  offset character -3, -2, 0 font \"\" textcolor lt -1 norotate\n";
  gp << "set xrange [ 0.00000 : 15.00000 ] noreverse nowriteback\n";
  gp << "set ylabel \"Y axis\"\n";
  gp << "set ylabel  offset character 3, -2, 0 font \"\" textcolor lt -1 rotate\n";
  gp << "set yrange [ 0.00000 : 15.00000 ] noreverse nowriteback\n";
  gp << "set zlabel \"Z axis\"\n";
  gp << "set zlabel  offset character -5, 0, 0 font \"\" textcolor lt -1 norotate\n";
  gp << "set zrange [ -2.00000 : 450.00000 ] noreverse nowriteback\n";
  gp << "splot [0:15] [0:15] '-' \n";
  gp.send2d(frame);
  gp.flush();
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
