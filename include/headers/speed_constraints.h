#ifndef PROJECT_SPEED_CONSTRAINTS_H
#define PROJECT_SPEED_CONSTRAINTS_H

#include "helper_functions.h"

double maximumDistanceConstraint(const Robot& robot, const RobotGraph& neighbourhood_preserved_robots,
                                 const Variables& v);

double maximumDistanceConstraint2(const Robot& robot, const RobotGraph& neighbourhood_preserved_robots,
                                  const Variables& v);

double interrobotAvoidanceConstraint(const Robot& robot, const RobotGraph& detected_robots, const Variables& v);

double obstacleAvoidanceConstraint(const Robot& i, const ObstacleGraph& detected_obstacles, const Variables& v,
                                   double discretization);

double LOSPreservationConstraint(const Robot& robot, const RobotGraph& robots, const ObstacleGraph& detected_obstacles,
                                 const Variables& v);

#endif  // PROJECT_SPEED_CONSTRAINTS_H
