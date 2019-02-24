#ifndef MULTIPLE_TURTLEBOTS_HELPER_FUNCTIONS_H
#define MULTIPLE_TURTLEBOTS_HELPER_FUNCTIONS_H

#include "classes.h"
#include "../gnuplot-iostream/gnuplot-iostream.h"
#include <optional>
#include "geometric_functions.h"

void getNotifiedParam(ros::NodeHandle& n_, const std::string& param_name, Variables& v);

std::pair<Obstacle, double> closestObstacleToLOS(const Robot& i, const Robot& j,
                                                 const ObstacleGraph& detected_obstacle_graph_in_D_set);

Robot j_star_compute(const Robot& i, const RobotGraph& robots_near_preserved,
                     const ObstacleGraph& detected_obstacle_graph_in_D_set);

std::optional<Obstacle> closestDetectedObstacle(const RigidObject& position, const ObstacleGraph& obstacles_detected,
                                                const Variables& v);

std::pair<RobotGraph, RobotGraph>
separateNeighbourRobotsBehindAndFront(const Robot& robot, const RobotGraph& neighbourhood_preserved_robots);

std::pair<RobotGraph, RobotGraph>
separateDetectedRobotsBehindAndFront(const Robot& robot, const RobotGraph& neighbourhood_preserved_robots);

ObstacleGraph closingObstaclesInDSpace(const Robot& robot_i, const Robot& robot_j,
                                       const ObstacleGraph& detected_obstacles);

std::optional<Obstacle> closestObstacleToLOSinDSpaceAtFront(const Robot& i, const Robot& j,
                                                            const ObstacleGraph& detected_obstacles);

std::optional<double> farthestRobotDistance(const Robot& position, const RobotGraph& robots);

std::optional<double> closestRobotDistance(const Robot& position, const RobotGraph& robots);

std::optional<double> minimumAngleNeighbour(const Robot& position, const RobotGraph& near_front_robots);

RobotGraph getNeighbourRobots(const Robot &robot, const RobotGraph &detected_robots, const Variables &v);

RobotGraph getNeighbourPreservedRobots(const Robot &robot, const RobotGraph &neighbour_robots, const Variables &v);

void printPlot(const std::vector<std::vector<std::tuple<double, double, double>>>& frame, const std::string& filename,
               const std::string& title, int rot_x_angle,
               int rot_z_angle);  // printing helper function

template <typename... Args, typename... Args2>
void printPlotWithArrows(const std::string& filename, const std::string& title, int rot_x_angle, int rot_z_angle,
                         std::vector<Robot> robots, std::function<double(Args...)> func, Args2&&... args)
{
  std::vector<std::vector<std::tuple<double, double, double>>> frame(120);

  for (int i = 0; i < 120; i++)
  {
    frame[i].resize(120);
    for (int j = 0; j < 120; j++)
    {
      Vector_t temp;
      temp << i / 8.0, j / 8.0;
      Robot point(temp);

      frame[i][j] =
          std::make_tuple(temp(0, 0), temp(1, 0), func(Robot(Position_t(temp(0, 0), temp(1, 0))), args...));
    }
  }

  Gnuplot gp;
  gp << "set term png\n";
  gp << "set output \"";
  gp << filename.c_str();
  gp << "\"\n";
  gp << "set view " << rot_x_angle << ", " << rot_z_angle << ", 1, 1\n";
  gp << "set samples 120, 120\n";
  gp << "set style data lines\n";
  gp << "set pm3d\n";
  gp << "set title \"";
  gp << title.c_str();
  gp << "\"\n";
  gp << "set xlabel \"X axis\"\n";
  gp << "set xlabel  offset character -3, -2, 0 font \"\" textcolor lt -1 norotate\n";
  gp << "set ylabel \"Y axis\"\n";
  gp << "set ylabel  offset character 3, -2, 0 font \"\" textcolor lt -1 rotate\n";
  gp << "set zlabel \"Z axis\"\n";
  gp << "set zlabel  offset character -5, 0, 0 font \"\" textcolor lt -1 norotate\n";
  for (auto& robot : robots)
  {
    gp << "set arrow from " << robot.getPosition()(0, 0) << "," << robot.getPosition()(1, 0) << ","
       << std::get<2>(frame[robot.getPosition()(0, 0) * 8][robot.getPosition()(1, 0) * 8]) + 0.01 << " to "
       << robot.getPosition()(0, 0) + robot.getSpeedDirection()(0, 0) << ","
       << robot.getPosition()(1, 0) + robot.getSpeedDirection()(1, 0) << ","
       << std::get<2>(frame[(robot.getPosition()(0, 0) + robot.getSpeedDirection()(0, 0)) * 8]
                           [(robot.getPosition()(1, 0) + robot.getSpeedDirection()(1, 0)) * 8]) +
              0.01
       << " filled front lw 2\n";
    std::cout << "set arrow from " << robot.getPosition()(0, 0) << "," << robot.getPosition()(1, 0) << ","
              << std::get<2>(frame[robot.getPosition()(0, 0) * 8][robot.getPosition()(1, 0) * 8]) + 0.01 << " to "
              << robot.getPosition()(0, 0) + robot.getSpeedDirection()(0, 0) << ","
              << robot.getPosition()(1, 0) + robot.getSpeedDirection()(1, 0) << ","
              << std::get<2>(frame[(robot.getPosition()(0, 0) + robot.getSpeedDirection()(0, 0)) * 8]
                                  [(robot.getPosition()(1, 0) + robot.getSpeedDirection()(1, 0)) * 8]) +
                     0.01
              << " filled front lw 2\n";
  }
  gp << "splot [0:15] [0:15] '-' \n";
  gp.send2d(frame);
  gp.flush();
}

#endif  // MULTIPLE_TURTLEBOTS_HELPER_FUNCTIONS_H
