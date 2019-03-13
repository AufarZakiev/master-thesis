#ifndef PROJECT_SPEED_CONSTRAINTS_H
#define PROJECT_SPEED_CONSTRAINTS_H

#include "helper_functions.h"
#include "geometric_functions.h"

double maximumDistanceConstraint(const Robot& robot, const RobotGraph& neighbourhood_preserved_robots,
                                 const Variables& v);

double maximumDistanceConstraint2(const Robot& robot, const RobotGraph& neighbourhood_preserved_robots);

double interrobotAvoidanceConstraint(const Robot& robot, const RobotGraph& detected_robots, const Variables& v);

double obstacleAvoidanceConstraint(const Robot& i, const ObstacleGraph& detected_obstacles, const Variables& v,
                                   double discretization);

double LOSUnitPreservationConstraint(const Robot &i, const ObstacleGraph &closing_obstacles_in_front_in_D_space,
                                     const Variables &v, const RobotGraph &neighbourhood_preserved_robots);

double LOSPreservationConstraint(const Robot& i, const ObstacleGraph& detected_obstacles, const Variables& v,
                                 const RobotGraph& neighbourhood_preserved_robots);

Vector_t getConstrainedSpeed(const Robot &robot, const RobotGraph &detected_robots,
                             const ObstacleGraph &detected_obstacles,
                             const Variables &v);

#endif  // PROJECT_SPEED_CONSTRAINTS_H
