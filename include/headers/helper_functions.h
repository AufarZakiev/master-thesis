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

ObstacleGraph closingObstaclesInDSpace(const Robot& robot_i, const Robot& robot_j, const ObstacleGraph& detected_obstacles);

std::optional<Obstacle> closestObstacleToLOSinDSpaceAtFront(const Robot &i, const Robot &j,
                                                            const ObstacleGraph &detected_obstacles);

std::optional<double> closestRobotDistance(const Robot& position, const RobotGraph& robots);

std::optional<double> minimumAngleNeighbour(const Robot &position, const RobotGraph &near_front_robots);

void printPlot(const std::vector<std::vector<std::tuple<double, double, double>>>& frame, const std::string& filename,
               const std::string& title, int rot_x_angle,
               int rot_z_angle);  // printing helper function

#endif  // MULTIPLE_TURTLEBOTS_HELPER_FUNCTIONS_H
