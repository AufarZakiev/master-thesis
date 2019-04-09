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

    bool movesCloserToObstacle = getRelativePosition(i, detected_obstacles[id]).dot(i.getSpeedDirection()) > 0.0;
    bool cond = (l_c - detected_obstacles[id].getRadius()) < OBS_AVOIDANCE_DISTANCE;

    if (movesCloserToObstacle && cond && speed > 0.0 && (speed < min_speed))
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
  for (size_t id = 0; id < boost::num_vertices(neighbourhood_preserved_robots); id++)
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
                      closest_obstacle.value().getRadius() - LOS_CLEARANCE_DISTANCE) /
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
  for (size_t id = 0; id < boost::num_vertices(neighbourhood_preserved_robots); id++)
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

Vector_t getConstrainedDirectedSpeed(const Robot& robot, ValidatedGraphs& vg, const ValidatedVariables& vv)
{
  double MAX_SPEED;
  vv.getParam("robot_max_speed", MAX_SPEED);
  Variables v = Variables(vv);
  ObstacleGraph detected_obstacles = getDetectedObstacles(robot, vg.getObstacleGraph(), v);
  RobotGraph detected_robots = getDetectedRobots(robot, vg.getRobotGraph(), detected_obstacles, v);

  boost::remove_vertex(findRobotInGraph(robot, detected_robots).value(), detected_robots);

  RobotGraph neighbour_robots = getNeighbourRobots(robot, detected_robots, v);
  RobotGraph neighbourhood_preserved_robots = getNeighbourPreservedRobots(robot, neighbour_robots, v);

  Vector_t gradientSpeed = gradientPotentialOnly(robot, detected_robots, detected_obstacles, v);
  Robot temp(robot);
  temp.setSpeedDirection(gradientSpeed);

  double calc_min =
      std::min({ maximumDistanceConstraint(temp, neighbourhood_preserved_robots, v),
                 maximumDistanceConstraint2(temp, neighbourhood_preserved_robots),
                 interrobotAvoidanceConstraint(temp, detected_robots, v),
                 obstacleAvoidanceConstraint(temp, detected_obstacles, v, 0.0),
                 LOSPreservationConstraint(temp, detected_obstacles, v, neighbourhood_preserved_robots), MAX_SPEED });

  calc_min = std::max({ calc_min, 0.0 });

  if (calc_min < 0.0)
  {
    std::cout << "Minimum speed is negative" << std::endl;
  }

  if (getVectorLength(gradientSpeed) > calc_min)
  {
    return calc_min * gradientSpeed / getVectorLength(gradientSpeed);
  }
  else
  {
    return gradientSpeed;
  }
}

double getConstrainedLeaderSpeed(const Robot& robot, ValidatedGraphs& vg, const ValidatedVariables& vv)
{
  double MAX_SPEED;
  vv.getParam("robot_max_speed", MAX_SPEED);
  Variables v = Variables(vv);
  ObstacleGraph detected_obstacles = getDetectedObstacles(robot, vg.getObstacleGraph(), v);
  RobotGraph detected_robots = getDetectedRobots(robot, vg.getRobotGraph(), detected_obstacles, v);

  boost::remove_vertex(findRobotInGraph(robot, detected_robots).value(), detected_robots);

  RobotGraph neighbour_robots = getNeighbourRobots(robot, detected_robots, v);
  RobotGraph neighbourhood_preserved_robots = getNeighbourPreservedRobots(robot, neighbour_robots, v);

  double calc_min = std::min({ maximumDistanceConstraint(robot, neighbourhood_preserved_robots, v),
                               maximumDistanceConstraint2(robot, neighbourhood_preserved_robots),
                               interrobotAvoidanceConstraint(robot, detected_robots, v),
                               obstacleAvoidanceConstraint(robot, detected_obstacles, v, 0.0),
                               LOSPreservationConstraint(robot, detected_obstacles, v, neighbourhood_preserved_robots),
                               MAX_SPEED * (1.0 / 3.0) });

  calc_min = std::max({ calc_min, 0.0 });

  if (calc_min < 0.0)
  {
    std::cout << "Minimum speed is negative" << std::endl;
  }

  return calc_min;
}