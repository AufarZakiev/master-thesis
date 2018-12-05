#ifndef PROJECT_SPEED_CONSTRAINTS_H
#define PROJECT_SPEED_CONSTRAINTS_H

#include "helper_functions.h"

double maximumDistanceConstraint(const Robot& robot, const RobotGraph& neighbourhood_preserved_robots,
                                 const Variables& v);

double maximumDistanceConstraint2(const Robot &robot, const RobotGraph &neighbourhood_preserved_robots,
                                  const Variables &v);

double interrobotAvoidanceConstraint();

double obstacleAvoidanceConstraint();

double LOSPreservationConstraint();

#endif //PROJECT_SPEED_CONSTRAINTS_H
