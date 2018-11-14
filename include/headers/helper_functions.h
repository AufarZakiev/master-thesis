#ifndef MULTIPLE_TURTLEBOTS_HELPER_FUNCTIONS_H
#define MULTIPLE_TURTLEBOTS_HELPER_FUNCTIONS_H

#include "DakaiAlgo.h"
#include <optional>

std::pair<Obstacle, double> closestObstacleToLOS(const Robot& i, const Robot& j,
                                                 const ObstacleGraph& detected_obstacle_graph_in_D_set);

Robot j_star_compute(const Robot& i, const RobotGraph& robots_near_preserved,
                     const ObstacleGraph& detected_obstacle_graph_in_D_set);

std::optional<Obstacle> closestDetectedObstacle(const RigidObject& position, const ObstacleGraph& obstacles_detected,
                                                const Variables& v);

#endif //MULTIPLE_TURTLEBOTS_HELPER_FUNCTIONS_H
