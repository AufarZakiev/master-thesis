#include "../include/headers/helper_functions.h"

std::pair<Obstacle, double> closestObstacleToLOS(const Robot& i, const Robot& j,
                                                 const ObstacleGraph& detected_obstacle_graph_in_D_set)
{
  Vector_t ji = getRelativePosition(i, j);
  auto min = std::numeric_limits<double>::max();
  Obstacle min_obstacle = detected_obstacle_graph_in_D_set[0];
  for (auto id = 0; id < boost::num_vertices(detected_obstacle_graph_in_D_set); ++id)
  {
    if (getVectorLength(getProjectionPhi(getRelativePosition(i, detected_obstacle_graph_in_D_set[id]), ji)) < min)
    {
      min = getVectorLength(getProjectionPhi(getRelativePosition(i, detected_obstacle_graph_in_D_set[id]), ji));
      min_obstacle = detected_obstacle_graph_in_D_set[id];
    }
  }
  return std::make_pair(min_obstacle, min);
}

Robot j_star_compute(const Robot& i, const RobotGraph& robots_near_preserved,
                     const ObstacleGraph& detected_obstacle_graph_in_D_set)
{
  auto min = closestObstacleToLOS(i, robots_near_preserved[0], detected_obstacle_graph_in_D_set);
  auto min_id = 0;
  for (auto id = 0; id < boost::num_vertices(detected_obstacle_graph_in_D_set); ++id)
  {
    if (closestObstacleToLOS(i, robots_near_preserved[id], detected_obstacle_graph_in_D_set).second < min.second)
    {
      min = closestObstacleToLOS(i, robots_near_preserved[id], detected_obstacle_graph_in_D_set);
      min_id = id;
    }
  }
  return robots_near_preserved[min_id];
}


std::optional<Obstacle> closestDetectedObstacle(const RigidObject& position, const ObstacleGraph& obstacles_detected,
                                                const Variables& v)
// TODO: change all trivial loops and std::algorithm
{
  if (boost::num_vertices(obstacles_detected) == 0)
  {
    return std::nullopt;  // TODO: Find the point to filter this case
  }
  auto min = getVectorLength(getRelativePosition(obstacles_detected[0], position));
  Obstacle min_obstacle = obstacles_detected[0];
  for (auto id = 0; id < boost::num_vertices(obstacles_detected); id++)
  {
    if (getVectorLength(getRelativePosition(obstacles_detected[id], position)) < min)
    {
      min = getVectorLength(getRelativePosition(obstacles_detected[id], position));
      min_obstacle = obstacles_detected[id];
    }
  }
  return min_obstacle;
}
