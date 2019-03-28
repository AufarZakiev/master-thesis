#include "../include/headers/helper_functions.h"

std::pair<Obstacle, double> closestObstacleToLOS(const Robot& i, const Robot& j,
                                                 const ObstacleGraph& detected_obstacle_graph_in_D_set)
{
  Vector_t ji = getRelativePosition(i, j);
  auto min = std::numeric_limits<double>::max();
  Obstacle min_obstacle = detected_obstacle_graph_in_D_set[0];
  for (size_t id = 0; id < boost::num_vertices(detected_obstacle_graph_in_D_set); ++id)
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
  size_t min_id = 0;
  for (size_t id = 0; id < boost::num_vertices(detected_obstacle_graph_in_D_set); ++id)
  {
    if (closestObstacleToLOS(i, robots_near_preserved[id], detected_obstacle_graph_in_D_set).second < min.second)
    {
      min = closestObstacleToLOS(i, robots_near_preserved[id], detected_obstacle_graph_in_D_set);
      min_id = id;
    }
  }
  return robots_near_preserved[min_id];
}

std::optional<Obstacle> closestDetectedObstacle(const RigidObject& position, const ObstacleGraph& obstacles_detected)
{
  // TODO: change all trivial loops and std::algorithm
  if (boost::num_vertices(obstacles_detected) == 0)
  {
    return std::nullopt;
  }
  auto minWithRadius =
      getVectorLength(getRelativePosition(obstacles_detected[0], position)) - obstacles_detected[0].getRadius();
  Obstacle min_obstacle = obstacles_detected[0];
  for (size_t id = 0; id < boost::num_vertices(obstacles_detected); id++)
  {
    auto distanceWithRadius =
        getVectorLength(getRelativePosition(obstacles_detected[id], position)) - obstacles_detected[id].getRadius();
    if (distanceWithRadius < minWithRadius)
    {
      minWithRadius = distanceWithRadius;
      min_obstacle = obstacles_detected[id];
    }
  }
  return min_obstacle;
}

std::pair<RobotGraph, RobotGraph>
separateNeighbourRobotsBehindAndFront(const Robot& robot, const RobotGraph& neighbourhood_preserved_robots)
{
  RobotGraph behind, front;
  for (size_t id = 0; id < boost::num_vertices(neighbourhood_preserved_robots); id++)
  {
    Vector_t ji = getRelativePosition(robot, neighbourhood_preserved_robots[id]);
    if (robot.getSpeedDirection().dot(ji) <= 0)
    {
      boost::add_vertex(neighbourhood_preserved_robots[id], behind);
    }
    else
    {
      boost::add_vertex(neighbourhood_preserved_robots[id], front);
    }
  }
  return std::make_pair(behind, front);
}

std::pair<RobotGraph, RobotGraph> separateDetectedRobotsBehindAndFront(const Robot& robot,
                                                                       const RobotGraph& detected_robots)
{
  RobotGraph detected_behind, detected_front;
  for (size_t id = 0; id < boost::num_vertices(detected_robots); id++)
  {
    Vector_t ji = getRelativePosition(robot, detected_robots[id]);
    if (robot.getSpeedDirection().dot(ji) <= 0)
    {
      boost::add_vertex(detected_robots[id], detected_behind);
    }
    else
    {
      boost::add_vertex(detected_robots[id], detected_front);
    }
  }
  return std::make_pair(detected_behind, detected_front);
}

ObstacleGraph closingObstaclesInDSpace(const Robot& robot_i, const Robot& robot_j,
                                       const ObstacleGraph& detected_obstacles)
{
  ObstacleGraph result;
  for (size_t id = 0; id < boost::num_vertices(detected_obstacles); id++)
  {
    if (isObjectInDSpace(detected_obstacles[id], robot_i, robot_j))
    {
      Vector_t oi = getRelativePosition(robot_i, detected_obstacles[id]);
      Vector_t ji = getRelativePosition(robot_i, robot_j);
      if (robot_i.getSpeedDirection().dot(getProjectionPhi(oi, ji)) > 0)
      {
        boost::add_vertex(detected_obstacles[id], result);
      }
    }
  }
  return result;
}

std::optional<Obstacle> closestObstacleToLOSinDSpaceAtFront(const Robot& i, const Robot& j,
                                                            const ObstacleGraph& detected_obstacles)
{
  ObstacleGraph detected_in_DSpace;
  for (size_t id = 0; id < boost::num_vertices(detected_obstacles); ++id)
  {
    if (isObjectInDSpace(detected_obstacles[id], i, j))
    {
      boost::add_vertex(detected_obstacles[id], detected_in_DSpace);
    }
  }
  if (boost::num_vertices(detected_in_DSpace) == 0)
  {
    return std::nullopt;
  }
  Vector_t ji = getRelativePosition(i, j);
  auto min = std::numeric_limits<double>::max();
  Obstacle min_obstacle = detected_in_DSpace[0];
  for (size_t id = 0; id < boost::num_vertices(detected_in_DSpace); ++id)
  {
    if (getVectorLength(getProjectionPhi(getRelativePosition(i, detected_in_DSpace[id]), ji)) < min)
    {
      min = getVectorLength(getProjectionPhi(getRelativePosition(i, detected_in_DSpace[id]), ji));
      min_obstacle = detected_in_DSpace[id];
    }
  }
  return min_obstacle;
}

std::optional<double> farthestRobotDistance(const Robot& position, const RobotGraph& robots)
{
  if (boost::num_vertices(robots) == 0)
  {
    return std::nullopt;
  }
  auto max = getVectorLength(getRelativePosition(position, robots[0]));
  for (size_t id = 0; id < boost::num_vertices(robots); id++)
  {
    if (getVectorLength(getRelativePosition(position, robots[id])) > max)
    {
      max = getVectorLength(getRelativePosition(position, robots[id]));
    }
  }
  return max;
}

std::optional<double> closestRobotDistance(const Robot& position, const RobotGraph& robots)
{
  if (boost::num_vertices(robots) == 0)
  {
    return std::nullopt;
  }
  auto min = getVectorLength(getRelativePosition(position, robots[0]));
  for (size_t id = 0; id < boost::num_vertices(robots); id++)
  {
    if (getVectorLength(getRelativePosition(position, robots[id])) < min)
    {
      min = getVectorLength(getRelativePosition(position, robots[id]));
    }
  }
  return min;
}

std::optional<double> minimumAngleNeighbour(const Robot& position,
                                            const RobotGraph& near_front_robots)  // TODO: wtf does this function do?
{
  if (boost::num_vertices(near_front_robots) == 0)
  {
    return std::nullopt;
  }
  auto min = position.getSpeedDirection().dot(getRelativePosition(position, near_front_robots[0]));
  for (size_t id = 0; id < boost::num_vertices(near_front_robots); id++)
  {
    if (position.getSpeedDirection().dot(getRelativePosition(position, near_front_robots[id])) < min)
    {
      min = position.getSpeedDirection().dot(getRelativePosition(position, near_front_robots[id]));
    }
  }
  return min;
}

RobotGraph getNeighbourRobots(const Robot& robot, const RobotGraph& detected_robots, const Variables& v)
{
  double NEIGHBOORHOOD_DISTANCE;
  v.getParam("neighbourhood_distance", NEIGHBOORHOOD_DISTANCE);
  RobotGraph neighbourhood_robots;
  for (size_t i = 0; i < num_vertices(detected_robots); i++)
  {
    if (robot.getRobotID() != detected_robots[i].getRobotID() &&
        getVectorLength(getRelativePosition(robot, detected_robots[i])) < NEIGHBOORHOOD_DISTANCE)
    {
      boost::add_vertex(detected_robots[i], neighbourhood_robots);
    }
  }
  return neighbourhood_robots;
}

RobotGraph getNeighbourPreservedRobots(const Robot& robot, const RobotGraph& neighbour_robots, const Variables& v)
{
  RobotGraph neighbourhood_preserved_robots;
  for (size_t i = 0; i < num_vertices(neighbour_robots); i++)
  {
    if (isEdgePreserved(robot, neighbour_robots[i], neighbour_robots, v))
    {
      boost::add_vertex(neighbour_robots[i], neighbourhood_preserved_robots);
    }
  }
  return neighbourhood_preserved_robots;
}

void getNotifiedParam(ros::NodeHandle& n_, const std::string& param_name, Variables& v)
{
  double param_variable;
  if (n_.getParam(param_name, param_variable))
  {
    ROS_INFO("Got param %s: %f", param_name.c_str(), param_variable);
    v.setParam(param_name, param_variable);
  }
  else
  {
    ROS_ERROR("Failed to get param %s. Setting to default value", param_name.c_str());
  }
}