#include "../include/headers/field_functions.h"

double cohesionPotential(const Robot& position, const RobotGraph& detected_robots,
                         const ObstacleGraph& detected_obstacles, const Variables& v)
{
  double sum = 0;
  if (boost::num_vertices(detected_obstacles) != 0)  // in case of obstacles presence, no cohesion exists
  {
    return 0;
  }
  for (RobotDesc id = 0; id < boost::num_vertices(detected_robots); ++id)
  {
    double arg = getVectorLength(getRelativePosition(position, detected_robots[id]));
    sum += partialCohesionPotential(arg, v);
  }
  return sum;
}

double LOSPreservePotential(const Robot& position, const RobotGraph& neighbourhood_robots,
                            const ObstacleGraph& detected_obstacle_graph, const Variables& v)
{
  ObstacleGraph detected_obstacle_graph_in_D_space;
  for (size_t id = 0; id < boost::num_vertices(detected_obstacle_graph); id++)
  {
    for (size_t id_neighbour = 0; id_neighbour < boost::num_vertices(neighbourhood_robots); id_neighbour++)
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
    return 0;
  }
  Robot j_star = j_star_compute(position, neighbourhood_robots, detected_obstacle_graph_in_D_space);
  auto closestObstacleToLOS = getClosestObstacleToLOS(position, j_star, detected_obstacle_graph_in_D_space).first;
  return partialLOSPreservePotential(
      getVectorLength(getProjectionPhi(getRelativePosition(closestObstacleToLOS, position),
                                       getRelativePosition(j_star, position))) -
          closestObstacleToLOS.getRadius(),
      v);
}

double obstacleCollisionPotential(const Robot& position, const ObstacleGraph& detected_obstacles, const Variables& v)
{
  auto closest_obstacle = closestDetectedObstacle(position, detected_obstacles);
  if (closest_obstacle)
  {
    Vector_t io = getRelativePosition(position, closest_obstacle.value());
    double ioLenWithRadius = getVectorLength(io) - closest_obstacle.value().getRadius();
    return partialObstacleCollisionPotential(ioLenWithRadius, v);
  }
  return 0;
}

double interrobotCollisionPotential(const Robot& position, const RobotGraph& robots_near_preserved, const Variables& v)
{
  double sum = 0;
  for (RobotDesc id = 0; id < boost::num_vertices(robots_near_preserved); ++id)
  {
    double arg = getVectorLength(getRelativePosition(position, robots_near_preserved[id]));
    sum += partialInterrobotCollisionPotential(arg, v);
  }
  return sum;
}

double overallPotential(const Robot& position, const RobotGraph& detected_robots,
                        const ObstacleGraph& detected_obstacles, const Variables& v)
{
  auto neighbour_robots = getNeighbourRobots(position, detected_robots, v);
  auto neighbour_robots_preserved = getNeighbourPreservedRobots(position, neighbour_robots, v);
  double interrobot_potential = interrobotCollisionPotential(position, neighbour_robots_preserved, v);
  double obstacle_potential = obstacleCollisionPotential(position, detected_obstacles, v);
  double cohesion_potential = cohesionPotential(position, detected_robots, detected_obstacles, v);
  double LOS_potential = LOSPreservePotential(position, neighbour_robots, detected_obstacles, v);
  double c1, c2, c3, c4;
  v.getParam("c1", c1);
  v.getParam("c2", c2);
  v.getParam("c3", c3);
  v.getParam("c4", c4);
  double result = c1 * interrobot_potential + c2 * obstacle_potential + c3 * LOS_potential + c4 * cohesion_potential;
  return result;
}
