#include "../include/headers/speed_constraints.h"

double maximumDistanceConstraint(const Robot& robot, const RobotGraph& neighbourhood_preserved_robots,
                                 const Variables& v)
{
  double NEIGHBOURHOOD_DISTANCE;
  v.getParam("neighbourhood_distance", NEIGHBOURHOOD_DISTANCE);
  RobotGraph neighbours_behind, neighbours_front;

  std::pair p = separateNeighbourRobotsBehindAndFront(robot, neighbourhood_preserved_robots);
  neighbours_behind = p.first;
  neighbours_front = p.second;
  auto closestRobotDist = closestRobotDistance(robot, neighbours_behind);
  if (closestRobotDist)
  {
    return 0.5 * (NEIGHBOURHOOD_DISTANCE - closestRobotDist.value());
  }
  else
  {
    return std::numeric_limits<double>::max();  // TODO: good point to avoid exceptions?
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
    return std::numeric_limits<double>::max();  // TODO: good point to avoid exceptions?
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
    return std::numeric_limits<double>::max();  // TODO: good point to avoid exceptions?
  }
};

double obstacleAvoidanceConstraint(const Robot& i, const ObstacleGraph& detected_obstacles, const Variables& v,
                                   double discretization)  // TODO: change for non-circular obstacles
{
  double OBS_AVOIDANCE_DISTANCE;
  v.getParam("obstacles_avoidance_distance", OBS_AVOIDANCE_DISTANCE);

  double min_speed = std::numeric_limits<double>::max();
  for (auto id = 0; id < boost::num_vertices(detected_obstacles); id++)
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

double LOSPreservationConstraint(const Robot& robot, const RobotGraph& robots, const ObstacleGraph& detected_obstacles,
                                 const Variables& v)
{
  double LOS_CLEARANCE_DISTANCE;
  v.getParam("los_clearance_distance", LOS_CLEARANCE_DISTANCE);

  double min_speed = std::numeric_limits<double>::max();
  for (auto id = 0; id < boost::num_vertices(detected_obstacles); id++)
  {
    auto closest_obstacle = closestObstacleToLOSinDSpaceAtFront(robot, robots[id], detected_obstacles);
    double angle = angleBetweenVectorsInRadians(robot.getSpeedDirection(), getRelativePosition(robots[id], robot));
    angle = angle > M_PI ? (M_PI * 2 - angle) : angle;
    double speed = (getVectorLength(getProjectionPhi(getRelativePosition(robot, closest_obstacle.value()),
                                                     getRelativePosition(robot, robots[id]))) -
                    LOS_CLEARANCE_DISTANCE) /
                   sin(angle);
    if (speed < min_speed)
    {
      min_speed = speed;
    }
  }
  return min_speed;
}
