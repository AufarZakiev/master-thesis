#ifndef MULTIPLE_TURTLEBOTS_FIELD_H
#define MULTIPLE_TURTLEBOTS_FIELD_H

#include "classes.h"
#include "partial_functions.h"
#include "geometric_functions.h"

double obstacleCollisionPotential(const RigidObject &position, const ObstacleGraph &detected_obstcles,
                                  const Variables &v);

double interrobotCollisionPotential(const RigidObject& position, const RobotGraph& robots_near_preserved, const Variables& v);

double LOSPreservePotential(const Robot& position, const RobotGraph& neighbourhood_robots,
                            const ObstacleGraph& detected_obstacle_graph_in_D_set, const Variables& v);

double cohesionPotential(const RigidObject& position, const RobotGraph& all_robots, const Variables& v);

#endif //MULTIPLE_TURTLEBOTS_FIELD_H
