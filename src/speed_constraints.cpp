#include "../include/headers/speed_constraints.h"
#include "../include/headers/field_functions.h"

double maximumDistanceConstraint(const Robot& robot, const RobotGraph& neighbourhood_preserved_robots,
                                 const Variables& v)
{
  double NEIGHBOURHOOD_DISTANCE;
  v.getParam("neighbourhood_distance", NEIGHBOURHOOD_DISTANCE);
  RobotGraph neighbours_behind, neighbours_front;

  std::pair p = separateNeighbourRobotsBehindAndFront(robot, neighbourhood_preserved_robots);
  neighbours_behind = p.first;
  neighbours_front = p.second;
  auto farthestRobotDist = farthestRobotDistance(robot, neighbours_behind);
  if (farthestRobotDist)
  {
    return 0.5 * (NEIGHBOURHOOD_DISTANCE - farthestRobotDist.value());
  }
  else
  {
    return std::numeric_limits<double>::max();
  }
};

double maximumDistanceConstraint2(const Robot& robot, const RobotGraph& neighbourhood_preserved_robots)
{
  RobotGraph neighbours_behind, neighbours_front;

  std::pair p = separateNeighbourRobotsBehindAndFront(robot, neighbourhood_preserved_robots);
  neighbours_behind = p.first;
  neighbours_front = p.second;
  auto neighbour = minimumAngleNeighbour(robot, neighbours_front);
  if (neighbour)
  {
    return neighbour.value() / getVectorLength(robot.getSpeedDirection());
  }
  else
  {
    return std::numeric_limits<double>::max();
  }
};

double interrobotAvoidanceConstraint(const Robot& robot, const RobotGraph& detected_robots, const Variables& v)
{
  double ROBOTS_AVOIDANCE_DISTANCE;
  v.getParam("robots_avoidance_distance", ROBOTS_AVOIDANCE_DISTANCE);

  RobotGraph detected_behind, detected_front;

  std::pair p = separateDetectedRobotsBehindAndFront(robot, detected_robots);
  detected_behind = p.first;
  detected_front = p.second;
  auto closestRobotDist = closestRobotDistance(robot, detected_front);
  if (closestRobotDist)
  {
    return 0.5 * (closestRobotDist.value() - ROBOTS_AVOIDANCE_DISTANCE);
  }
  else
  {
    return std::numeric_limits<double>::max();
  }
};

double obstacleAvoidanceConstraint(const Robot& i, const ObstacleGraph& detected_obstacles, const Variables& v,
                                   double discretization)  // TODO: change for non-circular obstacles
{
  double OBS_AVOIDANCE_DISTANCE;
  v.getParam("obstacles_avoidance_distance", OBS_AVOIDANCE_DISTANCE);

  double min_speed = std::numeric_limits<double>::max();
  for (size_t id = 0; id < boost::num_vertices(detected_obstacles); id++)
  {
    double l_c =
        getVectorLength(getProjectionPhi(getRelativePosition(i, detected_obstacles[id]), i.getSpeedDirection()));
    double l_v = sqrt(getVectorLength(getRelativePosition(i, detected_obstacles[id])) *
                          getVectorLength(getRelativePosition(i, detected_obstacles[id])) -
                      l_c * l_c);

    double speed = l_v - sqrt((OBS_AVOIDANCE_DISTANCE + detected_obstacles[id].getRadius()) *
                                  (OBS_AVOIDANCE_DISTANCE + detected_obstacles[id].getRadius()) -
                              l_c * l_c);
    if (speed < min_speed)
    {
      min_speed = speed;
    }
  }

  return min_speed;
};

double LOSUnitPreservationConstraint(const Robot& i, const ObstacleGraph& closing_obstacles_in_front_in_D_space,
                                     const Variables& v, const RobotGraph& neighbourhood_preserved_robots)
{
  double LOS_CLEARANCE_DISTANCE;
  v.getParam("los_clearance_distance", LOS_CLEARANCE_DISTANCE);

  double min_speed = std::numeric_limits<double>::max();
  for (size_t id = 0; id < boost::num_vertices(closing_obstacles_in_front_in_D_space); id++)
  {
    auto closest_obstacle = closestObstacleToLOSinDSpaceAtFront(i, neighbourhood_preserved_robots[id],
                                                                closing_obstacles_in_front_in_D_space);
    if (closest_obstacle)
    {
      double angle = angleBetweenVectorsInRadians(i.getSpeedDirection(),
                                                  getRelativePosition(neighbourhood_preserved_robots[id], i));
      angle = angle < 0 ? (M_PI * 2 + angle) : angle;
      angle = angle > M_PI ? (M_PI * 2 - angle) : angle;
      double speed = (getVectorLength(getProjectionPhi(getRelativePosition(i, closest_obstacle.value()),
                                                       getRelativePosition(i, neighbourhood_preserved_robots[id]))) -
                      LOS_CLEARANCE_DISTANCE) /
                     sin(angle);
      if (speed < min_speed)
      {
        min_speed = speed;
      }
    }
  }
  return min_speed;
}

double LOSPreservationConstraint(const Robot& i, const ObstacleGraph& detected_obstacles, const Variables& v,
                                 const RobotGraph& neighbourhood_preserved_robots)
{
  double min = std::numeric_limits<double>::max();
  for (size_t id = 0; id < boost::num_vertices(detected_obstacles); id++)
  {
    ObstacleGraph closing_obstacles_in_D_space =
        closingObstaclesInDSpace(i, neighbourhood_preserved_robots[id], detected_obstacles);
    double maybe_min =
        LOSUnitPreservationConstraint(i, closing_obstacles_in_D_space, v, neighbourhood_preserved_robots);
    if (maybe_min < min)
    {
      min = maybe_min;
    }
  }
  return min;
}

Vector_t getConstrainedSpeed(const Robot& robot, const RobotGraph& detected_robots,
                             const ObstacleGraph& detected_obstacles, const Variables& v)
{
  RobotGraph neighbourhood_preserved_robots = getNeighbourPreservedRobots(robot, detected_robots, v);
  double calc_min = std::min({ maximumDistanceConstraint(robot, neighbourhood_preserved_robots, v),
                               maximumDistanceConstraint2(robot, neighbourhood_preserved_robots),
                               interrobotAvoidanceConstraint(robot, detected_robots, v),
                               obstacleAvoidanceConstraint(robot, detected_obstacles, v, 0.0),
                               LOSPreservationConstraint(robot, detected_obstacles, v, neighbourhood_preserved_robots),
                               robot.getMaxSpeedValue() });

  Vector_t gradientSpeed =
      gradientPotential(robot.getPosition(), overallPotential, v, detected_robots, detected_obstacles);

  if (getVectorLength(gradientSpeed) > calc_min)
  {
    return robot.getSpeedDirection() * calc_min / getVectorLength(gradientSpeed);
  }
  else
  {
    return robot.getSpeedDirection();
  }
}