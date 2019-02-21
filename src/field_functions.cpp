#include "../include/headers/field_functions.h"

double cohesionPotential(const RigidObject& position, const RobotGraph& all_robots, const Variables& v)
{
  double sum = 0;
  for (RobotDesc id = 0; id < boost::num_vertices(all_robots); ++id)
  {
    double arg = getVectorLength(getRelativePosition(position, all_robots[id]));
    sum += partialCohesionPotential(arg, v);
  }
  return sum;
}

double LOSPreservePotential(const Robot& position, const RobotGraph& neighbourhood_robots,
                            const ObstacleGraph& detected_obstacle_graph, const Variables& v)
{
  ObstacleGraph detected_obstacle_graph_in_D_space;
  for (auto id = 0; id < boost::num_vertices(detected_obstacle_graph); id++)
  {
    for (auto id_neighbour = 0; id_neighbour < boost::num_vertices(neighbourhood_robots); id_neighbour++)
    {
      if (!isObjectInDSpace(detected_obstacle_graph[id], position, neighbourhood_robots[id_neighbour]))
      {
        break;
      }
      boost::add_vertex(detected_obstacle_graph[id], detected_obstacle_graph_in_D_space);
    }
  }
  if (boost::num_vertices(detected_obstacle_graph_in_D_space) == 0)
  {
    return 0;  // TODO: Find the point to filter this case
  }
  Robot j_star = j_star_compute(position, neighbourhood_robots, detected_obstacle_graph_in_D_space);
  return partialLOSPreservePotential(
          getVectorLength(getProjectionPhi(
                  getRelativePosition(closestObstacleToLOS(position, j_star, detected_obstacle_graph_in_D_space).first,
                                      position),
                  getRelativePosition(j_star, position))),
          v);
}

double obstacleCollisionPotential(const RigidObject& position, const ObstacleGraph& detected_obstacles,
                                  const Variables& v)
{
  auto closest_obstacle = closestDetectedObstacle(position, detected_obstacles, v);
  if (closest_obstacle)
  {
    Vector_t io = getRelativePosition(position, closest_obstacle.value());
    return partialObstacleCollisionPotential(getVectorLength(io), v);
  }
  return 0;
}

double interrobotCollisionPotential(const RigidObject& position, const RobotGraph& robots_near_preserved,
                                    const Variables& v)
{
  double sum = 0;
  for (RobotDesc id = 0; id < boost::num_vertices(robots_near_preserved); ++id)
  {
    double arg = getVectorLength(getRelativePosition(position, robots_near_preserved[id]));
    sum += partialInterrobotCollisionPotential(arg, v);
  }
  return sum;
}
